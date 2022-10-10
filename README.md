# LidarOccupancyGridMapping
-------------------------------------------------------------------------------------------------------------------------------
Occupancy Grid Mapping simulation using lidar and odometry data and VTKPlotter for display.
-------------------------------------------------------------------------------------------------------------------------------

Author: Peter Stratton 

Emails: pstratt@umich.edu, pstratto@ucsd.edu

<img src="https://github.com/peterstratton/LidarOccupancyGridMapping/blob/master/Lidar_gif.gif" width="450">

## Installation

First, install python executing this in a terminal:
```
sudo apt-get update
sudo apt-get install python
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
The program will first show a window with the robot and lidar scan from the first timestep. After clicking 'x' to close that initial window, the rest of the scans will render automatically. 
