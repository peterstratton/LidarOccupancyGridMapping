# LidarOccupancyGridMapping
-------------------------------------------------------------------------------------------------------------------------------
Occupancy Grid Mapping simulation using lidar and odometry data and VTKPlotter for display.
-------------------------------------------------------------------------------------------------------------------------------

Author: Peter Stratton 

Emails: pstratt@umich.edu, pstratto@ucsd.edu

<img src="https://github.com/ExistentialRobotics/MARL-Coverage/blob/main/BSA.gif" width="450">

## Installation

First, install python executing this in a terminal:
```
sudo apt-get update
sudo apt-get install python3.6
```

Next, install pip by running:
```
sudo apt install python3-pip
```

We want to use a virtual environment to isolate the required packages for 
the repo, so use this command to install venv, a package used to create and
manage virtual environments:
```
pip install virtualenv
```

Next, run this command to start the virtual environement and install the dependancies:
```
./easy_install.sh
```

After that, execute this command to run the simulation:
```
python main.py
```
