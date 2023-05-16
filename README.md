# CICRSIM

## 仿真世界加载二维码
Clone our project and put [apriltag](/apriltag/) in /home/${USER_NAME}/.gazebo/models, so you can see apriltags on the wall in the simulation world.

~~~
sudo cp -r ./apriltag ~/.gazebo/models
~~~

Extract the models.zip and copy to /home/${USER_NAME}/.gazebo/models.

~~~
sudo cp -r ./models/* ~/.gazebo/models
~~~

## 下载RotorS
Information about downloading RotorS could be found in https://github.com/ethz-asl/rotors_simulator.
If you're using ubuntu20.04, you may meet "catkin build failed, command not found:catkin", try this to solve:
```
sudo apt-get install python3-catkin-tools
```

## 仿真器相关话题

We provide a basic quadrotor model with a depth camera attached to it. Users can subscribe to odom topic and depth topic published in the simulation. Some topics are as follows:
```
odom_topic: /ardrone/ground_truth/odometry

depth_topic: /camera/depth/image_raw

quadrotor control topic: /gazebo/set_model_state
```
## 仿真器编译
This project has been tested on 18.04(ROS Melodic) and 20.04(ROS Noetic). Take Ubuntu 18.04 as an example, simply clone and compile our package (using ssh here):
```
cd ${YOUR_WORKSPACE_PATH}/src
git clone git@github.com:SYSU-STAR/CICRSIM.git
cd ../ 
catkin_make
```
## 键盘控制节点依赖
For keyboard, you should install ```pygame``` first, by:
```
sudo install python-pygame
```
For ubuntu20.04, you may use:

~~~
pip install pygame
~~~

将[files](/files)放入工作空间的根目录(和src、build、devel一级)，如果键盘控制节点无法运行，请检查[files](/files)的位置

## 键盘仿真启动
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
最后启动控制消息转换程序和键盘控制节点，控制方式：方向键控制无人机前后左右的速度, W和S控制飞机上下, A和D控制飞机yaw角朝向
```
source devel/setup.bash

rosrun uav_simulation command_process.py

source devel/setup.bash

rosrun uav_simulation keyboard_control.py
```
## 打分系统测试(单独启动)
会在终端输出当前时刻的分数和剩余时间
```
source devel/setup.bash

rosrun uav_simulation referee_system
```
## Known Issues
> 无人机在碰撞到场地道具后出现姿态不稳定的翻转

将无人机降落到地面上再起飞即可恢复正常。

> 如果遇到std成员函数导致的编译问题

将所有CmakeLists.txt中的-std=c++11改为-std=c++17。

> 如果遇到Gazebo打开卡住的问题

将电脑网络断开。

## Acknowledegments

We use [RotorS](https://github.com/ethz-asl/rotors_simulator) to generate a quadrotor and odometry information, [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros.git) to detect Apriltags. We really appreciate these open source projects!

