Lite3_MotionSDK 与 Gazebo 联合仿真
=====

## 环境和依赖库
请提前安装ROS和Gazebo
[ROS](https://www.ros.org/blog/getting-started/#)
[Gazebo11](https://gazebosim.org/docs)


对于**ROS melodic**，请安装如下功能包
```
sudo apt-get install ros-melodic-controller-interface  ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
```

对于**ROS noetic**，请安装如下功能包
```
sudo apt-get install ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
```

## 仿真步骤
  ### 1.代码编译 ###

```
cd ${your_workspace}
catkin_make
source ${your_workspace}/devel/setup.bash
```

  ### 2.加载模型 ###
打开一个新的终端
```
roslaunch lite3_motion_sim lite3_gazebo_simulation_control.launch
```

  ### 3.控制启动 ###
打开一个新的终端
```
rosrun lite3_motion_sim lite3_motion_sim
```



## RViz查看模型
打开一个新的终端
```
roslaunch lite3_motion_sim lite3_rviz_display.launch
```
