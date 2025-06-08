#!/bin/bash

# 获取版本参数
ros_version=$1

# 检查版本参数是否为空
if [ -z "$ros_version" ]; then
  echo "未提供版本参数"
  exit 1
fi

# 安装 ROS 的依赖
sudo apt-get update
echo "----------------------------------------------------"
sudo apt-get install -y ros-$ros_version-rosbash
echo "----------------------------------------------------"
sudo apt-get install -y ros-$ros_version-xacro
echo "----------------------------------------------------"
sudo apt-get install -y ros-$ros_version-rviz
echo "----------------------------------------------------"
sudo apt-get install -y ros-$ros_version-gazebo* ros-$ros_version-controller-interface ros-$ros_version-gazebo-ros-control
echo "----------------------------------------------------"
sudo apt-get install -y ros-$ros_version-robot-state-publisher ros-$ros_version-joint-state-controller 
echo "----------------------------------------------------"
sudo apt-get install -y ros-$ros_version-position-controllers ros-$ros_version-effort-controllers ros-$ros_version-jointjectory-controller
echo "----------------------------------------------------"
sudo apt-get install -y ros-$ros_version-joint-state-publisher-gui
echo "----------------------------------------------------"
# 更新环境变量
source ~/.bashrc
echo " ----------------------------------------------------"
source /opt/ros/$ros_version/setup.bash
echo " ----------------------------------------------------"
echo "安装完成"

