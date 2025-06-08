#!/bin/bash

# 获取版本参数
ros_version=$1

# 检查版本参数是否为空
if [ -z "$ros_version" ]; then
  echo "未提供 ROS 版本参数"
  exit 1
fi

# 安装依赖项
sudo apt update
echo " ----------------------------------------------------"
sudo apt install -y libyaml-cpp-dev
echo " ----------------------------------------------------"
sudo apt install -y libeigen3-dev
echo " ----------------------------------------------------"
sudo apt install -y liblcm-dev
echo " ----------------------------------------------------"
sudo apt install -y libglm-dev
echo " ----------------------------------------------------"
sudo apt install -y ros-$ros_version-rosbash
echo " ----------------------------------------------------"
sudo apt install -y ros-$ros_version-xacro
echo " ----------------------------------------------------"
sudo apt install -y ros-$ros_version-rviz
echo " ----------------------------------------------------"
sudo apt install -y ros-$ros_version-controller-interface
echo " ----------------------------------------------------"
sudo apt install -y ros-$ros_version-gazebo-ros-control
echo " ----------------------------------------------------" 
sudo apt install -y ros-$ros_version-joint-state-controller
echo " ----------------------------------------------------" 
sudo apt install -y ros-$ros_version-effort-controllers
echo " ----------------------------------------------------" 
sudo apt install -y ros-$ros_version-joint-trajectory-controller
echo " ----------------------------------------------------"
# 更新环境变量
source ~/.bashrc
echo " ----------------------------------------------------"
source /opt/ros/$ros_version/setup.bash
echo " ----------------------------------------------------"
echo "安装完成"