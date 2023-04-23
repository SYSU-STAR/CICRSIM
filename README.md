# CICRSIM

## 仿真世界加载二维码
Clone our project and put [apriltag](/apriltag/) in /home/${USER_NAME}/.gazebo/models, so you can see apriltags on the wall in the simulation world.

## 仿真器相关话题
We provide a basic quadrotor model with a depth camera attached to it. Users can subscribe to odom topic and depth topic published in the simulation. Some topics are as follows:
```
odom_topic: /ardrone/ground_truth/odometry

depth_topic: /camera/depth/image_raw

quadrotor control topic: /gazebo/set_model_state
```
## 仿真器编译
This project has been tested on 18.04(ROS Melodic) and 20.04(ROS Noetic). Take Ubuntu 18.04 as an example, run the following commands to install required tools:

```
sudo apt-get install ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox ros-melodic-mavros
```
Then simply clone and compile our package (using ssh here):
```
cd ${YOUR_WORKSPACE_PATH}/src
git clone git@github.com:SYSU-STAR/CICRSIM.git
cd ../ 
catkin_make
```
## 键盘控制节点依赖
For keyboard, you should install ```pygame``` first, by:
```
sudo apt-get install python-pygame
```
## 仿真启动
请按照以下步骤依次启动仿真，否则可能导致随机地图无法刷新
```
source devel/setup.bash

roslaunch uav_simulation env_simulation.launch
```
等待随机地图环境加载完成后，启动仿真无人机
```
source devel/setup.bash

roslaunch uav_simulation uav_simulation.launch
```
最后启动键盘控制节点，控制方式：方向键控制无人机前后左右的速度, W和S控制飞机上下, A和D控制飞机yaw角朝向
```
source devel/setup.bash

rosrun uav_simulation keyboard_control.py
```

## Acknowledegments
We use [RotorS](https://github.com/ethz-asl/rotors_simulator) to generate a quadrotor and odometry information, [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros.git) to detect Apriltags. We really appreciate these open source projects!


