# CICRSIM Date: 2023.05.30
协同智能是当前智能机器人发展的重要方向之一，它弥补了单一机器人在任务执行过程中鲁棒性不足、任务完成度低等问题，正在成为救灾、侦察等任务的有效解决方案。为吸引更多未来AI人才关注参与机器人协同技术发展，中山大学人工智能学院将承办```第一届逸仙勇士杯协同机器人大赛```。

![cicr](/files/cicr.png)

本仓库为```仿真赛项目```的```无人机模拟器```，该赛项的主要内容是：复杂场景中存在多个位置未知的目标二维码和随机障碍物，无人机需要通过机载传感器和计算机感知环境和规划运动，尽快搜索到目标二维码并准确报告其位置坐标，发现目标越多，用时越短者获得更高的分数。比赛中，参赛队需要为无人机开发```环境感知```以及```路径规划```算法，控制无人机自主探索未知环境，寻找目标。

<p align="center">
  <img src="files/sim.gif">
</p>

## 仿真器依赖安装
以Ubuntu18.04系统为例
```
$ sudo apt-get install ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-mavros ros-melodic-mav-msgs
```
如果使用Ubuntu20.04，遇到 "catkin build failed, command not found:catkin"的问题，可以尝试如下方法进行解决:
```
sudo apt-get install python3-catkin-tools
```

## 仿真器编译
在安装完相关依赖后，将代码克隆到一个新的工作空间中，并通过catkin_make进行编译:
```
cd ${YOUR_WORKSPACE_PATH}/src
git clone git@github.com:SYSU-STAR/CICRSIM.git
cd ../ 
catkin_make
```
## 仿真器设置
参赛选手根据 [sim_env.sh](/sim_env.sh) 配置仿真环境，脚本中的```catkin_ws```请自行替换为自己的工作空间
```
cd CICRSIM/
./sim_env.sh
```

以上为仿真器的相关依赖安装和环境配置，接下来说明如何使用官方提供的仿真器。

## 键盘控制节点依赖安装
首先安装 ```pygame```:
```
sudo install python-pygame
```
对于ubuntu20.04，可以尝试使用如下命令:

```
pip install pygame
```
如果键盘控制节点无法运行，请检查 [files](/files) 的位置，确保 [files](/files) 被正确放置在```工作空间根目录```中

## 启动仿真
### ```仿真界面展示```
启动仿真前，请确保启动脚本 [start_simulation.sh](/start_simulation.sh) 的位置在工作空间目录下，并按照脚本中的顺序```依次```启动仿真，否则可能导致随机地图无法刷新
```
./start_simulation.sh
```
在脚本加载完成后，参赛者将会看见如下仿真界面：

![control_demo](/files/control_demo.png)
### ```仿真文件说明```
启动脚本中打开的 [referee_system.launch](/cicr2023_simulator/uav_simulation/launch/referee_system.launch)为官方的打分系统，打分系统的触发方式为在```Rviz```中使用```2D Nav Goal```选取目标点：

![rviz](/files/rviz.png)

此时，打分系统会在终端输出比赛的当前剩余时间和得分：

![referee_system](/files/referee_system.png)

启动脚本中打开的 [keyboard_control.py](/cicr2023_simulator/uav_simulation/src/keyboard_control.py) 为官方提供的键盘控制节点示例，参赛者可参考 [keyboard_control.py](/cicr2023_simulator/uav_simulation/src/keyboard_control.py) 中控制无人机的方法，根据官方预留的控制接口发布相关话题来开发自己的无人机控制器，具体实现如下：
```
话题名称：/position_control
数据类型： Type[Odometry]
发布示例：model_odom_pub = rospy.Publisher('/position_control',Odometry,queue_size=10)
```
可分别设置无人机的```位置```、```姿态```、```线速度```和```角速度```，示例如下：
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
启动脚本中打开的 [command_process.py](/cicr2023_simulator/uav_simulation/src/command_process.py) 为控制器话题发布的接收端，用于接收和处理参赛者发布的控制话题。```注意```，该文件```不允许参赛者进行更改```，最终提交的代码中仅允许包含```算法部分```和发布控制指令的```控制器```。
### ```相关话题说明```
#### 传感器话题
|名称|类型|描述|
|:-|:-|:-|
|`/ardrone/ground_truth/odometry`|`nav_msgs/Odometry`|里程计数据，包括无人机的位置、姿态和速度信息|
|`/ardrone/ground_truth/imu`|`sensor_msgs/Imu`|IMU传感器数据，包括无人机的姿态、速度和加速度信息，由IMU收集|
|`/camera/color/camera_info`|`sensor_msgs/CameraInfo`|RGB相机内参信息|
|`/camera/color/image_raw`|`sensor_msgs/Image`|RGB彩色图像数据，从深度相机中获取|
|`/camera/depth/camera_info`|`sensor_msgs/CameraInfo`|深度相机信息|
|`/camera/depth/image_raw`|`sensor_msgs/Image`|深度图像数据|

若需要修改深度相机相关话题，可在 [_d435.gazebo.xacro](/cicr2023_simulator/uav_gazebo/urdf/_d435.gazebo.xacro)中修改；若需要激光雷达等传感器，请参赛选手自行添加。

#### 控制接口话题
|名称|类型|描述|
|:-|:-|:-|
|`/position_control`|`nav_msgs/Odometry`|官方控制接口，可发布无人机的位置、姿态、速度信息来控制无人机|

### 传感器话题获取示例
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
## 已知问题
> 无人机在碰撞到场地道具后出现姿态不稳定的翻转

将无人机降落到地面上再起飞即可恢复正常。

> 如果遇到std成员函数导致的编译问题

将所有CmakeLists.txt中的-std=c++11改为-std=c++17。

> 如果遇到Gazebo打开卡住的问题

将电脑网络断开。

