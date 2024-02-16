"""
gnssmonitor.py

CLI, that can monitor Jamming, Spoofing and GNSS Fix of a UBX RCB-F9T receiver on a Timecard.
It can also dump GNSS raw and radio frequency data from said receiver to CSV files

Version: 1.2
Date: 07.02.2024
Author: Simon Huber
Copyright: 2024 ETH Zurich
License: BSD 3-Clause

Acknowledgments/Credits: Some of the basic structure was taken from: https://github.com/semuconsulting/pygnssutils/blob/main/src/pygnssutils/gnssdump.py
"""

import os
from argparse import ArgumentParser
from datetime import datetime
import time
from serial import Serial
import csv
import pyubx2.exceptions as ube
from pyubx2 import (
    UBXMessage,
    UBXReader,
    POLL
)




class ACKError(Exception):
    """
    Custom Exception Class that will be raised if NAK or no ACK-* msg is received
    """
    def __init__(self, message="Receiver responded with ACK-NAK or neither ACK-ACK nor ACK-NAK was received within the receiver timelimit"):
        self.message = message
        super().__init__(self.message)




class gnssmonitor:

    def __init__(self, **kwargs):
        """
        parm: path ttypath: path to tty stream like: "/dev/ttyS5"
        parm: int baudrate: one of these baudrates: 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800
        parm: int serialtimeout: timeout of serial port
        parm: int receivertimeout: timeout for the receiver messages
        parm: path rawpath: path to the folder where the raw and rf dump file should be placed
        parm: flag verbose: indicates if CLI should log to stdout even if all is fine
        """
        self._stream = None
        self._reader = None
        self._ttypath = kwargs.get("ttypath", None)
        
        self._baudrate = int(kwargs.get("baudrate", 115200))
        self._stream_timeout = int(kwargs.get("serialtimeout", 3))
        self._receiver_timeout = int(kwargs.get("receivertimeout", 5))

        self._verbose = kwargs.get("verbose")

        self._last_UTC_date = None
        self._last_UTC_date_valid = False
        self._last_UTC_tod = None #tod is short for time of day
        self._last_UTC_nano = None
        self._last_UTC_tod_valid = False

        self._cfg_data = [
            ('CFG_MSGOUT_UBX_NAV_PVT_UART1', 1), #set NAV-PVT msg to be output every navigation epoch
            ('CFG_MSGOUT_UBX_NAV_STATUS_UART1', 1), #set NAV-STATUS msg to be output every navigation epoch
            ('CFG_MSGOUT_UBX_MON_RF_UART1', 1), #set MON-RF msg to be output every navigation epoch
            ('CFG_ITFM_ENABLE', 1) #enable jamming monitor
        ]
        self._cfg_data_save = None
        
        self._ubxmsg_to_handler = { #dict to get the right handler given the msg Class and ID concatenated
            b'\x01\x07': self._NAV_PVT_handler,
            b'\x01\x03': self._NAV_STATUS_handler,
            b'\x0a\x38': self._MON_RF_handler
        }

        #raw dump things
        self._rawpath = kwargs.get("rawpath", None)
        if self._rawpath is not None:
            #augment handlers and receiver configuration
            self._cfg_data = self._cfg_data + [
                ('CFG_MSGOUT_UBX_RXM_RAWX_UART1', 1), #set RXM_RAWX msg to be output every navigation epoch
                ('CFG_MSGOUT_UBX_RXM_SFRBX_UART1', 1) #set RXM_SFRBX msg to output broadcast navigation data very time a complete subframe is decoded
            ]
            self._ubxmsg_to_handler.update({
                                    b'\x02\x15': self._RXM_RAWX_handler,
                                    b'\x02\x13': self._RXM_SFRBX_handler
            })
            #create the headers for raw and rf dump (names of the columns in the csv)
            self._RAWX_header = [
                'lastUTC',
                'rcvTow',
                'week',
                'leapS',
                'leapSec',
                'clkReset',
                'prMes',
                'cpMes',
                'doMes',
                'gnssId',
                'svId',
                'sigId',
                'prStd',
                'cpStd',
                'doStd',
                'prValid',
                'cpValid',
                'halfCyc',
                'subHalfCyc'
            ]
            self._SFRBX_header = [
                'lastUTC',
                'gnssId',
                'sigId',
                'freqId',
                'chn',
                'version'
            ] + [(lambda x: 'dwrd_' + str(x).zfill(2))(i) for i in range(1, 11)]
            self._RF_header = [
                'lastUTC',
                'blockId',
                'jammingState',
                'antStatus',
                'antPower',
                'postStatus',
                'noisePerMS',
                'agcCnt',
                'jamInd',
                'ofsI',
                'magI',
                'ofsQ',
                'magQ'
            ]
            self._PVT_header = [
                'lastUTC',
                'iTOW', 
                'year', 
                'month', 
                'day', 
                'hour', 
                'min', 
                'second', 
                'validDate', 
                'validTime', 
                'fullyResolved', 
                'validMag', 
                'tAcc', 
                'nano', 
                'fixType', 
                'gnssFixOk', 
                'difSoln', 
                'psmState', 
                'headVehValid', 
                'carrSoln', 
                'confirmedAvai', 
                'confirmedDate', 
                'confirmedTime', 
                'numSV', 
                'lon', 
                'lat', 
                'height', 
                'hMSL', 
                'hAcc', 
                'vAcc', 
                'velN', 
                'velE', 
                'velD', 
                'gSpeed', 
                'headMot', 
                'sAcc', 
                'headAcc', 
                'pDOP', 
                'invalidLlh', 
                'lastCorrectionAge', 
                'headVeh', 
                'magDec', 
                'magAcc'
            ]
        



    '''
    helper functions
    '''
    def _readmsg(self):
        """
        Reads UBX messages from _stream and handles related errors accordingly

        Returns:
            parsed data (UBXMessage object)
        """
        try:
            (raw_data, parsed_data) = self._reader.read()
            return parsed_data
        except (
            ube.UBXMessageError,
            ube.UBXParseError,
            ube.UBXStreamError,
            ube.UBXTypeError
        ) as err:
            print(f'While reading from {self._ttypath} there was the error: {err}')
        

    def _ACK_helper(self, msg, clsID, msgID):
        """
        Checks if a UBXMessage is an Acknowledgement and reacts like this:
                returns False if the msg is neither ACK-ACK nor ACK-NAK
                returns True if the msg is ACK-ACK
                raises an ACKError if the msg is ACK-NAK

        Args:
            param: UBXMessage msg: the UBXMessage that should be checked
            param: int clsID: UBX Message Class the ACK-* should be for (respond to)
            param: int msgID: UBX Message ID the ACK-* should be for (respond to)
        
        Returns:
            bool (see description above)
        """
        if (msg.msg_cls == b'\x05'): #if msg is an ACK-* message
            if( msg.clsID.to_bytes(1, byteorder='little') == clsID and 
                msg.msgID.to_bytes(1, byteorder='little') == msgID ): #Check if this is a ACK-* from to our request
                if msg.msg_id == b'\x01': #if ACK-ACK
                    return True
                else: #if ACK-NAK (in ACK class there are only ACK-ACK and ACK-NAK which has msg id b'\x00')
                    raise ACKError('Receiver responded with ACK-NAK')
            else:
                return False #not the right ACK-* msg
        else:
            return False


    def _cfg_receiver_RAM(self, cfg_data):
        """
        Sets a list of configuration items on the receiver RAM (UBX-CFG-VALSET)

        Args:
            param: list cfg_data: list of tuples (configuration Item string, value)
        """
        outmsg = UBXMessage.config_set(1, 0, cfg_data) #first arg stands for receiver RAM
        start_time = time.time()
        self._stream.write(outmsg.serialize())
        while True:
            msg = self._readmsg()
            if self._ACK_helper(msg, b'\x06', b'\x8a'):
                break
            if time.time() - start_time > self._receiver_timeout:
                raise ACKError('neither ACK-ACK nor ACK-NAK was received within the timelimit')


    def _getattr_helper(self, msg, attr_name, index):
        """
        easy access for UBXMessage attributes that are part of repeated blocks

        Args:
            param: UBXMessage msg: the UBXMessage the attribute is part of
            param: str attr_name: "base name" of the attribute (everything except the numbering)
            param: int index: number of the attribute (which repeated block does is belong to)
        
        Returns:
            attribute (or None if it doesn't exist)
        """
        return getattr(msg, attr_name + str(index).zfill(2), None)


    def _get_last_UTC(self):
        """
        Returns string with last UTC (set by the last NAV-PVT in this implementation).
        Only valid Dates and Times are output (confirmedDate,... flag set in NAV-PVT in this implementation)

        Returns:
            string with latest UTC
        """
        last_UTC = ''
        if self._last_UTC_date_valid:
            last_UTC += self._last_UTC_date
        else:
            last_UTC += "no_valid_date"
        last_UTC += ' '
        if self._last_UTC_tod_valid:
            last_UTC += self._last_UTC_tod + ' n=' + self._last_UTC_nano 
        else:
            last_UTC += "no_valid_time"
        return last_UTC




    '''
    handler functions
    '''
    def _NAV_PVT_handler(self, msg):
        """
        Updates the last UTC attributes and logs gnssFixOk to stdout

        Args:
            param: UBXMessage msg: UBX-NAV-PVT UBXMessage object
        """
        #update last UTC
        if msg.confirmedDate == 1:
            self._last_UTC_date_valid = True
            self._last_UTC_date = f'{str(msg.year).zfill(4)}-{str(msg.month).zfill(2)}-{str(msg.day).zfill(2)}'
        else:
            self._last_UTC_date_valid = False

        if msg.confirmedTime == 1:
            self._last_UTC_tod_valid = True
            self._last_UTC_tod = f'{str(msg.hour).zfill(2)}:{str(msg.min).zfill(2)}:{str(msg.second).zfill(2)}' #lib has attribute "seconds" not "sec" as outlined in the TIM 2.20 Interface Description
            self._last_UTC_nano = str(msg.nano)
        else:
            self._last_UTC_tod_valid = False

        #log gnssFixOk to stdout
        match msg.gnssFixOk:
                case 0:
                    print(self._get_last_UTC() + f': No valid fix')
                case 1:
                    if self._verbose:
                        print(self._get_last_UTC() + f': Valid fix')
        

        if self._rawpath is not None:
            row = [
                self._get_last_UTC(),
                msg.iTOW, 
                msg.year, 
                msg.month, 
                msg.day, 
                msg.hour, 
                msg.min, 
                msg.second, 
                msg.validDate, 
                msg.validTime, 
                msg.fullyResolved, 
                msg.validMag, 
                msg.tAcc, 
                msg.nano, 
                msg.fixType, 
                msg.gnssFixOk, 
                msg.difSoln, 
                msg.psmState, 
                msg.headVehValid, 
                msg.carrSoln, 
                msg.confirmedAvai, 
                msg.confirmedDate, 
                msg.confirmedTime, 
                msg.numSV, 
                msg.lon, 
                msg.lat, 
                msg.height, 
                msg.hMSL, 
                msg.hAcc, 
                msg.vAcc, 
                msg.velN, 
                msg.velE, 
                msg.velD, 
                msg.gSpeed, 
                msg.headMot, 
                msg.sAcc, 
                msg.headAcc, 
                msg.pDOP, 
                msg.invalidLlh, 
                msg.lastCorrectionAge, 
                msg.headVeh, 
                msg.magDec, 
                msg.magAcc
            ]
        
            with open(self._NAV_PVT_dump_filepath , 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(row)
        



    def _NAV_STATUS_handler(self, msg):
        """
        Logs spoofDetState to stdout

        Args:
            param: UBXMessage msg: UBX-NAV-STATUS UBXMessage object
        """
        match msg.spoofDetState:
                case 0:
                    print(self._get_last_UTC() + f': Unknown spoofing detection state')
                case 1:
                    if self._verbose:
                        print(self._get_last_UTC() + f': No spooﬁng indicated')
                case 2:
                    print(self._get_last_UTC() + f': Spooﬁng indicated')
                case 3:
                    print(self._get_last_UTC() + f': Multiple spooﬁng indications')


    def _MON_RF_handler(self, msg):
        """
        Logs jammingState to stdout and dump data from MON_RF msg

        Args:
            param: UBXMessage msg: UBX-MON-RF UBXMessage object
        """
        bands = {
                0:"L1",
                1:"L2 or L5"
            }
        #in an ideal world one would handle this raw dump or not logic with the handler dict and more functions (maintainability) but it should be fine like this for now
        if self._rawpath is not None:
            data = []

        for i in range(1, msg.nBlocks+1):
            blockId = self._getattr_helper(msg, 'blockId_', i)
            jammingState = self._getattr_helper(msg, 'jammingState_', i)
            match jammingState:
                case 0:
                    print(self._get_last_UTC() + f': jamming state unknown on {bands[blockId]} band')
                case 1:
                    if self._verbose:
                        print(self._get_last_UTC() + f': ok - no significant jamming on {bands[blockId]} band')
                case 2:
                    print(self._get_last_UTC() + f': warning - interference visible on {bands[blockId]} band but ﬁx OK')
                case 3:
                    print(self._get_last_UTC() + f': critical - interference visible on {bands[blockId]} band and no ﬁx')
            
            if self._rawpath is not None:
                row = [
                    self._get_last_UTC(),
                    self._getattr_helper(msg, 'blockId_', i),
                    self._getattr_helper(msg, 'jammingState_', i),
                    self._getattr_helper(msg, 'antStatus_', i),
                    self._getattr_helper(msg, 'antPower_', i),
                    self._getattr_helper(msg, 'postStatus_', i),
                    self._getattr_helper(msg, 'noisePerMS_', i),
                    self._getattr_helper(msg, 'agcCnt_', i),
                    self._getattr_helper(msg, 'jamInd_', i),
                    self._getattr_helper(msg, 'ofsI_', i),
                    self._getattr_helper(msg, 'magI_', i),
                    self._getattr_helper(msg, 'ofsQ_', i),
                    self._getattr_helper(msg, 'magQ_', i),
                ]
                data.append(row)
        
        if self._rawpath is not None:
            with open(self._MON_RF_dump_filepath, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerows(data)
            


    def _RXM_RAWX_handler(self, msg):
        """
        Dumps gnss raw data to csv file

        Args:
            param: UBXMessage msg: UBX-RXM-RAWX UBXMessage object
        """
        data = []
        for i in range(1, msg.numMeas+1):
            row = [
                self._get_last_UTC(),
                msg.rcvTow,
                msg.week,
                msg.leapS,
                msg.leapSec,
                msg.clkReset,
                self._getattr_helper(msg, 'prMes_', i),
                self._getattr_helper(msg, 'cpMes_', i),
                self._getattr_helper(msg, 'doMes_', i),
                self._getattr_helper(msg, 'gnssId_', i),
                self._getattr_helper(msg, 'svId_', i),
                self._getattr_helper(msg, 'sigId_', i),
                self._getattr_helper(msg, 'prStd_', i),
                self._getattr_helper(msg, 'cpStd_', i),
                self._getattr_helper(msg, 'doStd_', i),
                self._getattr_helper(msg, 'prValid_', i),
                self._getattr_helper(msg, 'cpValid_', i),
                self._getattr_helper(msg, 'halfCyc_', i),
                self._getattr_helper(msg, 'subHalfCyc_', i)
            ]
            data.append(row)

        with open(self._RXM_RAWX_dump_filepath, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerows(data)


    def _RXM_SFRBX_handler(self, msg):
        """
        Dumps gnss raw subframe data to csv file

        Args:
            param: UBXMessage msg: UBX-RXM-SFRBX UBXMessage object
        """
        row = [
                self._get_last_UTC(),
                msg.gnssId,
                msg.sigId,
                msg.freqId,
                msg.chn,
                msg.version
        ]
        for i in range(1, msg.numWords+1):
            row.append(self._getattr_helper(msg, 'dwrd_', i))
        
        with open(self._RXM_SFRBX_dump_filepath , 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(row)


    def _other_handler(self, msg):
        """
        "empty" handler for all messages that dont need to be "handled"

        Args:
            param: UBXMessage msg: UBXMessage object
        """
        return


    def _get_handler(self, msg):
        """
        Return the appropriate handler for UBXMessage

        Args:
            param: UBXMessage msg: UBXMessage object that needs to be handled
        
        Returns:
            handler method
        """
        key = msg.msg_cls + msg.msg_id
        return self._ubxmsg_to_handler.get(key, self._other_handler)




    '''
    setup and teardown functions
    '''
    def _firmware_hardware_version_check(self):
        """
        small check to alert the user if their receiver Hardware or Software version is not the one that this CLI was developed for
        """
        outmsg = UBXMessage(0x0a, 0x04, POLL) #create the pool request UBX-MON-VER
        start_time = time.time()
        self._stream.write(outmsg.serialize())
        while True:
            msg = self._readmsg()
            if msg.msg_cls == b'\x0a' and msg.msg_id == b'\x04':
                if msg.swVersion != b'\x45\x58\x54\x20\x43\x4f\x52\x45\x20\x31\x2e\x30\x30\x20\x28\x37\x31\x62\x32\x30\x63\x29\x00\x00\x00\x00\x00\x00\x00\x00':
                    print("WARNING: The Software Version of your receiver doesn't match the one this CLI was developed for and tested on, if you experience unexpected behavior, this might be the reason")
                if msg.hwVersion != b'\x30\x30\x31\x39\x30\x30\x30\x30\x00\x00':
                    print("WARNING: The Hardware Version of your receiver doesn't match the one this CLI was developed for and tested on,  if you experience unexpected behavior, this might be the reason")
                break
            if time.time() - start_time > self._receiver_timeout: 
                print(f'WARNING: During Firmware and Hardware check, no response was received after {self._receiver_timeout} (ACKtimeout) seconds (ACKtimeout), this is not good')
                break
                

    def _receiver_setup(self):
        """
        saves receiver RAM configuration item values from the receiver
        and sets these configuration item values to the ones that are needed for monitoring
        """
        #save existing config of the receiver
        cfg_items = [cfg_item for (cfg_item, value) in self._cfg_data]
        cfgpoolmsg = UBXMessage.config_poll(0, 0, cfg_items)
        start_time = time.time()
        self._stream.write(cfgpoolmsg.serialize())
        count = 0
        while count < 2:
            msg = self._readmsg()
            if msg.msg_cls == b'\x06' and msg.msg_id == b'\x8b':
                self._cfg_data_save = [(cfg_item, getattr(msg, cfg_item)) for (cfg_item, value) in self._cfg_data]
                count += 1
            if self._ACK_helper(msg, b'\x06', b'\x8b'):
                count += 1
            if time.time() - start_time > self._receiver_timeout:
                raise ACKError('neither ACK-ACK nor ACK-NAK was received within the timelimit')
        print("Old receiver RAM config saved")

        #write the needed config to the receiver   
        self._cfg_receiver_RAM(self._cfg_data)
        print("Receiver RAM config changed")


    def _logfiles_setup(self):
        """
        Creates the raw and rf dump csv files and writes their headers
        """
        #format raw and rf dump filenames
        currenttimedate = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self._RXM_RAWX_dump_filepath = os.path.join(self._rawpath, f'RXM_RAWX_dump_{currenttimedate}.csv')
        self._RXM_SFRBX_dump_filepath = os.path.join(self._rawpath, f'RXM_SFRBX_dump_{currenttimedate}.csv')
        self._MON_RF_dump_filepath = os.path.join(self._rawpath, f'MON_RF_dump_{currenttimedate}.csv')
        self._NAV_PVT_dump_filepath = os.path.join(self._rawpath, f'NAV_PVT_dump_{currenttimedate}.csv')
        #create the raw and rf dump files and write the headers
        with open(self._RXM_RAWX_dump_filepath , 'w', newline='') as RAWX_csv, open(self._RXM_SFRBX_dump_filepath , 'w', newline='') as SFRBX_csv, open(self._MON_RF_dump_filepath , 'w', newline='') as RF_csv, open(self._NAV_PVT_dump_filepath , 'w', newline='') as PVT_csv:
            RAWX_csv_writer = csv.writer(RAWX_csv)
            SFRBX_csv_writer = csv.writer(SFRBX_csv)
            RF_csv_writer = csv.writer(RF_csv)
            PVT_csv_writer = csv.writer(PVT_csv)
            RAWX_csv_writer.writerow(self._RAWX_header)
            SFRBX_csv_writer.writerow(self._SFRBX_header)
            RF_csv_writer.writerow(self._RF_header)
            PVT_csv_writer.writerow(self._PVT_header)


    def __enter__(self):
        """
        (Context Manager Enter Method)
        Coordinates firmware/hardware check and, setup of the receiver and raw and rf dump files.
        """
        #set up the tty to read from it
        print("------------------------------------------")
        print("Setup start")
        self._stream = Serial(self._ttypath, self._baudrate, timeout=self._stream_timeout)
        self._reader = UBXReader(
            self._stream,
            validate=1    
        )

        #check receiver firmware version
        self._firmware_hardware_version_check()

        #create log files (rawpath)
        if self._rawpath is not None:
            self._logfiles_setup()

        #configure the receiver if needed
        self._receiver_setup()
        
        print("Setup complete")
        print("------------------------------------------")
        return self


    def __exit__(self, exc_type, exc_value, exc_traceback):
        """
        (Context Manager Exit Method)
        Catches Exceptions that are not specifically handles somewhere else (not Base Exceptions)
        and restores the old receiver configuration
        """
        if issubclass(exc_type, Exception): #Only Exception (not BaseExceptions) should be caught here
            print(f'During Monitoring this exception occurred: {exc_value}')
        #restore the receiver RAM config 
        try:
            self._cfg_receiver_RAM(self._cfg_data_save)
            print("------------------------------------------")
            print("Old receiver RAM config restored")
            print("------------------------------------------")
        except ACKError as ae:
            print("------------------------------------------")
            print("Failed to restore old receiver RAM config")
            print("------------------------------------------")
            print(ae)




    '''
    monitor
    '''
    def monitor(self):
        """
        Central loop, that continuously reads new messages and calls the appropriate handler  
        """
        while (True):
            #read from serial
            msg = self._readmsg()
            #get handler and call it
            handler = self._get_handler(msg)
            handler(msg)





'''  
main function
'''
def main():
    """
    Creates the Argument Parser (CLI functionality)
    and starts the CLI
    """
    arp = ArgumentParser(
        description="This CLI monitors gnssFix, jamming and spoofing. In addition it can also dump raw gnss and rf data to csv log files",
    )

    arp.add_argument(
        '--ttypath', 
        required=True, 
        help='Path to tty, for ex: /dev/ttyS4'
    )

    arp.add_argument(
        "--baudrate",
        required=False,
        help="Serial baud rate",
        type=int,
        choices=[4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800],
        default=115200,
    )

    arp.add_argument(
        "--serialtimeout",
        required=False,
        help="Serial timeout in seconds (if the timeout is very high, configuring the receiver may take very long, consider setting the --no-conf-receiver flag)",
        type=int,
        default=3,
    )

    arp.add_argument(
        "--receivertimeout",
        required=False,
        help="Receiver Timeout in seconds (how long the CLI should wait for a response from the receiver)",
        type=int,
        default=5,
    )

    arp.add_argument(
        '--rawpath',
        required=False,
        help='if specified the raw and rf data will be dumped at this path, tip: if you pass "." the data will be dumped in the current working directory'
        )

    arp.add_argument(
        "--verbose",
        required=False,
        action='store_true',
        help="if this flag is set the CLI will log to stdout even if all is fine (no jamming, spoofing or lack of fix is detected)",
    )

    kwargs = vars(arp.parse_args())

    try:
        with gnssmonitor(**kwargs) as gm:
            gm.monitor()

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()