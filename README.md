# GNSS Monitor

CLI, that can monitor Jamming, Spoofing and GNSS Fix of a UBX RCB-F9T receiver on a Timecard.
It can also dump GNSS raw and radio frequency data from said receiver to CSV files

## Requirements/Installation

- Timecard with UBX RCB-F9T receiver
- Timecard Linux driver installed
- Python 3.11.4+ (or at least 3.10+ or else the match will certainly not work)
- ```pyubx2``` python lib installed (https://github.com/semuconsulting/pyubx2/tree/master) (recommended via ```pip``` in a virtual environment)
- this repo cloned
- depending on the privileges the CLI runs with, you might have the permissions to access the serial port. There are two ways to do this:
	- Add your use to the dialout group (recommended) (or just the group the serial port belongs to, which is generally dialout) this can be done like this: ```sudo usermod -aG dialout your_username``` (you might have to restart for this to take effect)
	- Change the read and write permissions of your Serial Port (```/dev/tty*```). Tip: If you for example want to give read and write permission to everyone you can use this command: ```(sudo) chmod o+rw /dev/tty*```.

## Usage
```
python gnssmonitor.py -h    #for help
python gnssmonitor.py --ttypath /dev/ttyS5 --rawpath . --verbose    #example usage
```
If you use a virtual environment you can either activate you environment and run the CLI like above:
```
source path/to/your/venv/bin/activate
python gnssmonitor.py --ttypath /dev/ttyS5 --rawpath . --verbose
deactivate
```
or you can directly call the python executable form your venv:
```
path/to/your/venv/bin/python gnssmonitor.py --ttypath /dev/ttyS5 --rawpath . --verbose
```

### Running in the background

If you want to run monitor in the backround you can do it like this:
```
nohup path/to/your/venv/bin/python gnssmonitor.py --ttypath /dev/ttyS5 --rawpath raw --verbose > logs/logfilename.log 2>&1 &
```
This should print the \<PID> (process ID), which you can use later to terminate the program like this:
```
kill -INT <PID>
```