# Script to run and install the virtual environment
python3 -m venv env
source env/bin/activate
pip3 install requests
pip3 install numpy
pip3 install vedo
deactivate
