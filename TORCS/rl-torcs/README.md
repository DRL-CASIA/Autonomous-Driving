## Reinforcement Learning and Deep Learning Based Lateral Control for Autonomous Driving

This is Tensorflow implementation of our IEEE CIM 2019 paper on [Reinforcement Learning and Deep Learning Based Lateral Control for Autonomous Driving](https://ieeexplore.ieee.org/document/8686348). If you use this work, please cite:

```txt
@article{LiZhaoZhangChen2019Reinforcment,
  author={D. {Li} and D. {Zhao} and Q. {Zhang} and Y. {Chen}},
  journal={IEEE Computational Intelligence Magazine}, 
 title={Reinforcement Learning and Deep Learning Based Lateral Control for Autonomous Driving [Application Notes]}, 
  year={2019},
  volume={14},
  number={2},
  pages={83-98},
  doi={10.1109/MCI.2019.2901089},
  ISSN={1556-6048},
  month={May},}
```



#### Installation

Please note this implement use Python2.7, and is tested on Ubuntu 16.04.

```shell
https://github.com/DRL-CASIA/rl-torcs.git
cd rl-torcs

# Install TORCS dependencies
sudo apt-get update
sudo apt-get install libglib2.0-dev libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev libplib-dev libopenal-dev libalut-dev libxi-dev libxmu-dev libxrender-dev libxrandr-dev libpng12-dev libvorbis-dev xautomation

# Install VisualTorcs
# This will install torcs to: 
# 	/usr/local/lib/torcs
#	/usr/local/bin/torcs
#	/usr/local/share/games/torcs
cd torcs-1.3.7
./configure
make -j4
sudo make install
sudo make datainstall
# Copy the customized track files to torcs tracks
cp -a ../tracks/* /usr/local/share/games/torcs/tracks/road/

# Create a python2.7 virtual environment
sudo apt-get install virtualenv
virtualenv -p python2.7 torcs-env
source torcs-env/bin/activate
# Install tensorflow-gpu if preferred.
pip install tensorflow==1.11.0 sysv_ipc opencv-python 
```



#### Demo

Download the model weights from [Google Drive](https://drive.google.com/open?id=17iIbm9Va-eN7aOxtbO-Qap-onTd3F6oK), and uncompress to the directory `drl_torcs/`.

**Note:** The following operation is necessary since the agent is based on the *first-person view*. At the first run of torcs, the default perspective is the third-person perspective. Press `F2` for twice to switch to first-person perspective. Then press `2~9` to toggle the floating driving status, and `m` to toggle map.

```shell
# Run the agent with visual input
python main.py --mode play --vision
# Run the agent with low-dimensional sensor measurement
python main.py --mode play
```



#### Training

The RL control module can be trained by the following. The default track is G-track-3. For training on different TORCS tracks: First open TORCS by execute `torcs` in terminal, Second select `Practice -> Configure` to select tracks like Forza and Alpine-2. 

```shell
cd drl_torcs
# Train the RL control module
python main.py --mode train
```

The environment perception network is based on AlexNet and trained by using Caffee. The trained model is converted to the tensorflow model by using [caffe-tensorflow converter](https://github.com/ethereon/caffe-tensorflow).



#### Credits

- The VTORCS implementation is inspired by Chenyi's [DeepDriving implementation](https://deepdriving.cs.princeton.edu/).
- The customized tracks are from DeepDriving.

