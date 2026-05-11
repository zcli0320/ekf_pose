# AGENTS.md

## 项目背景

这是一个 ROS1 Noetic catkin 项目，主要实现 EKF 数据融合，融合 IMU、里程计和 GPS 或 MAVROS 全局定位数据。

## 环境

系统主要运行在 WSL2 Ubuntu 20.04。
ROS 版本是 Noetic。
工作空间路径是 \~/catkin\_ws。
包名是 ekf。

## 常用命令

编译：
source /opt/ros/noetic/setup.bash
cd \~/catkin\_ws
catkin build ekf
source \~/catkin\_ws/devel/setup.bash

查看 topic：
rostopic list
rostopic echo /topic\_name
rostopic hz /topic\_name

查看 bag：
rosbag info path/to/file.bag
rosbag play --clock path/to/file.bag

## 修改代码规则

优先保持现有代码结构。
修改 EKF 相关代码前，先解释状态量、输入量、观测量和协方差矩阵的对应关系。
不要随意重命名 ROS topic、frame\_id 和消息类型。
修改 CMakeLists.txt 或 package.xml 后，需要说明为什么修改。
修改后优先运行 catkin build ekf。

