
### Install torcs
sudo apt-get install libglib2.0-dev libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev libplib-dev libopenal-dev libalut-dev libxi-dev libxmu-dev libxrender-dev libxrandr-dev libpng12-dev libvorbis-dev
./configure
make -j4
sudo make install
sudo make datainstall
sudo apt-get install xautomation

copy four folders in tracks to /usr/local/share/games/torcs/tracks/road

### Setup Python2.7 virtual environment 
sudo apt-get update
sudo apt-get install virtualenv
virtualenv -p python2.7 torcs-env
source torcs-venv/bin/activate
pip install tensorflow==1.11.0 sysv_ipc opencv-python

### Running the agent
python main.py

