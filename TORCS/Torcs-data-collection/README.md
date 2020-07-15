Data Collector for TORCS
=====
### Description  
Highly customized collector used to collect image and the corresponding labels in TORCS simulator. The data is saved in hdf5 format and can be easily used across platforms and programming languages.   
Here is an example for data tree  
```
g-track3.h5
    \host_data
        \frame_0
            image: driver-view RGB image (res. 640x480)
            pos_x: X coordinate in global frame
            pos_y: Y coordinate in global frame
            vel_x: velocity along vehicle heading direction
            vel_y: velocity opposite to vehicle heading direction
            dist_to_center: distance to track center
            dist_raced: distance has raced
            yaw: angle difference between vehicel heading and track orientation
            steer: steering angle normalized by steering ratio (21 degree)
            accel: normalized acceleration
            brake: normalized brake
        \frame_1
        ...
    \dc1_data
    ...
```

**Configure the track**  
Change inot the directory `/usr/local/share/games/torcs/tracks/road`, select and go into a desired track, modify the `.acc` file to change the related track image.


### Installation
Install TORCS, official [reference](http://torcs.sourceforge.net/index.php?name=Sections&op=viewarticle&artid=3#linux-src-all)  
```shell
## Install dependencies
sudo apt install libplib-dev libopenal-dev libalut-dev libvorbis-dev libxxf86vm-dev libxmu-dev
cd torcs-1.3.7
./configure CXXFLAGS=-std=c++11
make -j4
sudo make install
sudo make datainstall
```

### Usage
Set the following variables in `main.py` as needed BEFORE running:
- `step_limit`: Total running steps for collection task in a track.
- `track_name`: The track which are used in TORCS.
After setting the above variables, then issue the commands:
```shell
## Asume in the project root directory.
python main.py
```
The collected dataset will be stored in directory `./dataset/xxx.h5` while `xxx` is the track name.