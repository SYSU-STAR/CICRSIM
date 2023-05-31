# SYSU CICR Simulator

![cicr](files/cicr.png)

中山大学人工智能学院承办的第一届逸仙勇士杯协同机器人大赛现已启动。本仓库致力于为参与**仿真赛**项目的开发者提供关键仿真环境和代码接口。

仿真赛的主要任务内容是：复杂场景中存在多个位置未知的目标二维码和随机障碍物，无人机需要实时感知环境和规划运动，尽快搜索到目标二维码并准确报告其位置坐标，发现目标越多，用时越短者获得更高的排名。比赛中，参赛队需要为无人机开发**环境感知**以及**路径规划**算法，控制无人机自主探索未知环境，寻找目标。

<p align="center">
  <img src="files/sim.gif">
</p>

# 目录

  - [1. 安装说明 ](#1-安装说明)
  - [2. 运行仿真](#2-运行仿真)
  - [3. 技术细节说明](#3-技术细节说明)
  - [4. 代码提交规范](#4-代码提交规范)
  - [5. 已知问题](#5-已知问题)

# 1. 安装说明

在开始安装依赖前，请确保当前系统已经安装了 **ROS**。如果还未安装 **ROS**，请先安装 [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) 或 [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)。

## 1.1 仿真器依赖安装
对于 Ubuntu18.04：
```
$ sudo apt-get install ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-mavros ros-melodic-mav-msgs python-pygame
```
对于 Ubuntu20.04：
```
$ sudo apt-get install ros-noetic-mavlink python3-wstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev ros-noetic-mavros python3-pygame
```
## 1.2 仿真器编译
在安装完相关依赖后，将代码克隆到一个新的工作空间中，并通过catkin_make进行编译：
```
cd ${YOUR_WORKSPACE_PATH}/src
git clone git@github.com:SYSU-STAR/CICRSIM.git
cd ../ 
catkin_make
```
## 1.3 仿真器设置
首先，为脚本添加可执行权限：
```
cd CICRSIM/
sudo chmod +x sim_env.sh start_simulation.sh
```
参赛选手根据 [sim_env.sh](/sim_env.sh) 配置仿真环境，脚本中的 ```YOUR_WORKSPACE_PATH``` 请自行替换为自己的工作空间名称。例如，对一个路径为 ```~/zhangsan_workspace``` 的工作空间来说，先将 [sim_env.sh](/sim_env.sh) 中的内容替换为如下所示的内容：
```
#!/bin/bash
sudo cp -r ./apriltag ~/.gazebo/models
sudo unzip ./models.zip -d ~/.gazebo/models
sudo cp -r ./files ~/zhangsan_workspace
sudo cp -r ./start_simulation.sh ~/zhangsan_workspace
```
接着，在终端中输入如下命令：
```
cd CICRSIM/
./sim_env.sh
```

以上为仿真器的相关依赖安装和环境配置，接下来说明如何使用官方提供的仿真器。

# 2. 运行仿真

## 2.1 仿真界面展示
启动仿真前，请确保启动脚本 [start_simulation.sh](/start_simulation.sh) 的位置在工作空间目录下，例如 ```/home/zhangsan/zhangsan_workspace```。

```
./start_simulation.sh
```
在脚本加载完成后，参赛者将会看见如下仿真界面：

![control_demo](/files/control_demo.png)

## 2.2 启动脚本功能说明
|文件名称|功能描述|
|:-|:-|
|[keyboard_control.py](/cicr2023_simulator/uav_simulation/src/keyboard_control.py)|官方提供的接盘控制节点示例，参赛选手可参考其中控制无人机的方法，根据官方预留的控制接口发布相关话题来开发自己的无人机控制器|
|[command_process.py](/cicr2023_simulator/uav_simulation/src/command_process.py)|控制器话题接收端，此文件不允许参赛选手进行改动|
|[env_simulation.launch](/cicr2023_simulator/uav_simulation/launch/env_simulation.launch)|加载仿真环境，并刷新随机障碍物和二维码|
|[uav_simulation.launch](/cicr2023_simulator//uav_simulation/launch/uav_simulation.launch)|加载仿真无人机|
|[referee_system.launch](/cicr2023_simulator/uav_simulation/launch/referee_system.launch)|裁判系统，在参赛者触发比赛标志位后，无人机方能离开地面进行探索，系统开始倒计时并计算当前得分|

# 3. 技术细节说明
## 3.1 如何无人机控制接口控制无人机
```
话题名称：/position_control
数据类型：nav_msgs/Odometry
发布示例：model_odom_pub = rospy.Publisher('/position_control',Odometry,queue_size=10)
```
可分别控制无人机的位置、姿态、线速度和角速度，示例如下：
```
odom.pose.pose.position.x = 1.0
odom.pose.pose.position.y = 1.0
odom.pose.pose.position.z = 1.0

odom.pose.pose.orientation.w = 1.0
odom.pose.pose.orientation.x = 0.0
odom.pose.pose.orientation.y = 0.0
odom.pose.pose.orientation.z = 0.0

odom.twist.twist.linear.x = 1.0
odom.twist.twist.linear.y = 1.0
odom.twist.twist.linear.z = 1.0

odom.twist.twist.angular.x = 1.0
odom.twist.twist.angular.y = 1.0
odom.twist.twist.angular.z = 1.0

model_odom_pub.publish(odom)
```
**提示：** 可参考 [keyboard_control.py](/cicr2023_simulator/uav_simulation/src/keyboard_control.py) 中控制无人机的方法进行实现

## 3.2 相关话题说明
仿真无人机上默认搭载了一个RealSense深度相机，参赛选手可获取相机的相关话题以及无人机的里程计、IMU等话题，具体说明如下：
### 3.2.1 传感器话题
|名称|类型|描述|
|:-|:-|:-|
|`/ardrone/ground_truth/odometry`|`nav_msgs/Odometry`|里程计数据，包括无人机的位置、姿态和速度信息|
|`/ardrone/ground_truth/imu`|`sensor_msgs/Imu`|IMU传感器数据，包括无人机的姿态、速度和加速度信息，由IMU收集|
|`/camera/color/camera_info`|`sensor_msgs/CameraInfo`|RGB相机内参信息|
|`/camera/color/image_raw`|`sensor_msgs/Image`|RGB彩色图像数据，从深度相机中获取|
|`/camera/depth/camera_info`|`sensor_msgs/CameraInfo`|深度相机信息|
|`/camera/depth/image_raw`|`sensor_msgs/Image`|深度图像数据|

若需要修改深度相机相关话题，可在 [_d435.gazebo.xacro](/cicr2023_simulator/uav_gazebo/urdf/_d435.gazebo.xacro)中修改；若需要激光雷达等传感器，请参赛选手自行添加。

### 3.2.2 其他接口话题
|名称|类型|描述|
|:-|:-|:-|
|`/position_control`|`nav_msgs/Odometry`|官方控制接口，可发布无人机的位置、姿态、速度信息来控制无人机|
|`/start_flag`|`std_msgs/Bool.h`|裁判系统触发接口，发布对应的消息以开始计时与计算选手当前得分

### 3.2.3 传感器话题获取示例
下面给出一段示例代码，说明如何获取相机的深度数据：
```
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
 void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // 处理深度图像消息
}
 int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_image_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 1, depthImageCallback);
    ros::spin();
}
```
## 3.3 裁判系统
比赛开始前，参赛选手需要先启动官方裁判系统，在裁判系统启动后，参赛选手的无人机才能获得允许离开地面。裁判系统的启动方式为参赛选手根据裁判系统触发接口发布对应的话题作为启动信号，具体实现示例如下：
```
#include <ros/ros.h>
#include <std_msgs/Bool.h>
ros::Publisher start_flag_pub;
void startflagPub()
{
    std_msgs::Bool my_start_flag;
    my_start_flag.data = true;
    start_flag_pub.publish(my_start_flag);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_sim");
  ros::NodeHandle nh;
  start_flag_pub = nh.advertise<std_msgs::Bool>("/start_flag",10);
  startflagPub();
}
```

裁判系统会在终端输出比赛的当前剩余时间和得分：

![referee_system](/files/referee_system.png)

# 4. 代码提交规范
## 4.1 提交文件命名规范与功能说明
在最终提交代码时，参赛选手需要提交的文件有：

|文件名称|文件属性|功能描述|
|:-|:-|:-|
|`sysu_planner`|文件夹|实现无人机自主探索的算法部分|
|`sysu_controller`|文件夹|无人机控制器部分，通过官方提供的控制接口实现对无人机的控制|
|`sysu_simulation.sh`|脚本文件|能够一键运行仿真和实现自主探索功能的脚本|
|`sysu_report.pdf`|说明文档|说明提交代码的逻辑结构，是否需要配置其他环境等|

其中，文件夹的命名规范为 **队名_planner/controller**，脚本文件的命名规范为 **队名_simulation.sh**，说明文档的命名规范为 **队名_report.pdf**。

## 4.2 具体要求说明
### 4.2.1 脚本文件
参赛选手最终提交的脚本文件的编写方式请参考 [start_simulation.sh](/start_simulation.sh)，示例如下：
```
#!/bin/bash
gnome-terminal -t "roscore" -x bash -c "roscore;exec bash;"
sleep 5s

gnome-terminal -t "uav_simulation" -x bash -c "source devel/setup.bash;roslaunch uav_simulation env_simulation.launch;exec bash;"
sleep 5s

gnome-terminal -t "uav_simulation" -x bash -c "source devel/setup.bash;roslaunch uav_simulation uav_simulation.launch;exec bash;"
sleep 3s

gnome-terminal -t "uav_simulation" -x bash -c "source devel/setup.bash;roslaunch uav_simulation referee_system.launch;exec bash;"
sleep 2s

gnome-terminal -t "uav_simulation" -x bash -c "source devel/setup.bash;roslaunch sysu_planner exploration.launch;exec bash;"
sleep 2s
```
### 4.2.2 说明文档
参赛选手最终提交的说明文档中需要说明提交代码的逻辑结构，技术创新点，是否需要配置其他环境等内容。

# 5. 已知问题

**Q**: 随机地图无法刷新或不出现无人机

**A**: 在 [start_simulation.sh](/start_simulation.sh) 文件中先通过 [env_simulation.launch](/cicr2023_simulator/uav_simulation/launch/env_simulation.launch) 加载随机地图，再通过 [uav_simulation.launch](/cicr2023_simulator//uav_simulation/launch/uav_simulation.launch) 加载仿真无人机，请确保二者在[start_simulation.sh](/start_simulation.sh) 文件中的顺序不被改变

---
**Q**: 无人机在碰撞到场地道具后出现姿态不稳定的翻转

**A**: 将无人机降落到地面上再起飞即可恢复正常

---

**Q**: 如果遇到std成员函数导致的编译问题

**A**: 将所有CmakeLists.txt中的-std=c++11改为-std=c++17

---

**Q**: 如果遇到Gazebo打开卡住的问题

**A**: 将电脑网络断开

---

**Q**: 键盘控制节点无法运行

**A**: 请检查 [files](/files) 的位置，确保 [files](/files) 被正确放置在工作空间根目录中，例如 ```/home/zhangsan/zhangsan_workspace```

---


