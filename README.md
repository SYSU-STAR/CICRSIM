# CICRSIM

## 仿真器依赖安装
以Ubuntu18.04系统为例
```
$ sudo apt-get install ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-mavros
```
如果使用Ubuntu20.04，遇到 "catkin build failed, command not found:catkin"的问题，可以尝试如下方法进行解决:
```
sudo apt-get install python3-catkin-tools
```
## 仿真世界模型设置
将 [apriltag](/apriltag/) 文件夹放在 /home/${USER_NAME}/.gazebo/models 目录下

~~~
sudo cp -r ./apriltag ~/.gazebo/models
~~~

解压 [models.zip](/models.zip) 到 /home/${USER_NAME}/.gazebo/models 目录下

~~~
sudo cp -r ./models/* ~/.gazebo/models
~~~



## 仿真器相关话题

里程计话题
```
odom_topic: /ardrone/ground_truth/odometry
```
深度相机话题
```
depth_topic: /camera/depth/image_raw
```
若需要修改相关话题，可在 [_d435.gazebo.xacro](/cicr2023_simulator/uav_gazebo/urdf/_d435.gazebo.xacro)中修改；若需要激光雷达等传感器，请参赛选手自行添加
## 仿真器编译
我们在Ubuntu 18.04(ROS Melodic) and 20.04(ROS Noetic)上对仿真器进行了测试，请按照如下命令编译仿真器:
```
cd ${YOUR_WORKSPACE_PATH}/src
git clone git@github.com:SYSU-STAR/CICRSIM.git
cd ../ 
catkin_make
```
## 键盘控制节点依赖
首先安装 ```pygame```，命令如下:
```
sudo install python-pygame
```
对于ubuntu20.04，可以尝试使用如下命令:

~~~
pip install pygame
~~~

注意：在启动节点之前，将[files](/files)放入工作空间的根目录(和src、build、devel一级)，如果键盘控制节点无法运行，请检查[files](/files)的位置

## 仿真启动
在启动仿真之前，将启动脚本 [start_simulation.sh](/start_simulation.sh) 放在工作空间目录下，并按照脚本中的顺序依次启动仿真，否则可能导致随机地图无法刷新
```
./start_simulation.sh
```

## 已知问题
> 无人机在碰撞到场地道具后出现姿态不稳定的翻转

将无人机降落到地面上再起飞即可恢复正常。

> 如果遇到std成员函数导致的编译问题

将所有CmakeLists.txt中的-std=c++11改为-std=c++17。

> 如果遇到Gazebo打开卡住的问题

将电脑网络断开。


## 说明
我们使用 [RotorS](https://github.com/ethz-asl/rotors_simulator) 来设计了仿真无人机及传感器，感谢丰富的开源资料 

