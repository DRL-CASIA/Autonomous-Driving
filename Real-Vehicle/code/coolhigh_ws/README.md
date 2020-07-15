### 控制模块readme

#### 包含３部分
- 1.　kvaser驱动
- 2.　dbw_node
- 3.　controller

#### 使用方法

1.打开驱动　

```
roslaunch kvaser_interface kvaser_can_bridge.launch
```
接收can_rx和can_tx

2. 打开dbw_node

```
roslaunch dbw_coolhigh_can dbw.launch
```
接收brake_cmd,throttle_cmd,steering_cmd，发送can_rx和can_tx

3. 打开控制
```
roslaunch twist_controller dbw.launch
rostopic pub -r 10 /vehicle/enable std_msgs/Empty "{}" 
rostopic pub -r 10 /vehicle/dbw_enabled std_msgs/Empty "{}"  
```
接收velocity_cmd，发送throttle_cmd, brake_cmd.


#### 远程控制模块
ros包为coolhigh_ws
硬件部分:
- G29方向盘,　kvaser canbus, 车底盘，工控机等

软件部分:
1. 远程端
2. 车端
3. 数据通信

使用方法:

1.注册蒲公英vpn, 安装打开蒲公英vpn

```
sudo pgyvpn
```
查看主从机的ipi地址
- 账号 `20697075:001`
- 密码: `123456`

2.　设置主机和从机的/ect/hosts和~/.bashrc文件，将多机进行连机，测试ping
在/ect/hosts文件中加入以下几行代码　例如:

```
172.xx.xx.xx drl
172.xx.xx.xx gao
```
告诉ROS系统，谁才是老大！分别在主机和从机上使用vim或其他编辑器修改 ~/.bashrc 文件，在末尾加上

```
export ROS_HOSTNAME=drl
exprot ROS_MASTER_URI=http://drl:11311
```
其中，ROS_HOSTNAME分别对应主从机的hostname，ROS_MASTER_URI全部对应为主机的ip，你需要替换成自己的。

3.　在底盘工控机，打开kvaser驱动和dbw，使其能接收brake_cmd等

```
roslaunch remote dbw_all.launch #打开kvaser驱动，dbw模块
```

4.　在笔记本打开joy驱动和remote_control_node，其首先将串口数据解析为joy_msgs,
后利用remote_control_node解析为brake_cmd命令发送给主机实现控制


```
rosrun joy joy_node #开启joy
rosrun remote_control remote_control_node # 开启remote_controlan节点
```

5.开启uvc相机

```
rosrun uvc_camera uvc_camera_node #开启uvc相机节点
rosrun rqt_reconfigure rqt_reconfigure #调整压缩图像分辨率
```



   
// TODO 
- 增加倒车模块
- 融合到autoware中，逻辑
- 安装相机

