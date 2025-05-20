# Simulation Project Based on **Lite3 MotionSDK** and **Gazebo**

## Dependencies
Please install [ROS1](https://www.ros.org/blog/getting-started/#) and [Gazebo11](https://gazebosim.org/docs) before compilation.

If you use **ROS melodic**, please install the following dependencies：
```
sudo apt-get install ros-melodic-rosbash
sudo apt-get install ros-melodic-xacro
sudo apt-get install ros-melodic-rviz
sudo apt-get install ros-melodic-gazebo* ros-melodic-controller-interface 
sudo apt-get install ros-melodic-robot-state-publisher ros-melodic-joint-state-controller 
sudo apt-get install ros-melodic-position-controllers ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
sudo apt-get install ros-melodic-joint-state-publisher-gui
```

If you use **ROS noetic**, please install the following dependencies：
```
sudo apt-get install ros-noetic-rosbash
sudo apt-get install ros-noetic-xacro
sudo apt-get install ros-noetic-rviz
sudo apt-get install ros-noetic-gazebo* ros-noetic-controller-interface
sudo apt-get install ros-noetic-robot-state-publisher ros-noetic-joint-state-controller 
sudo apt-get install ros-noetic-position-controllers ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
sudo apt-get install ros-noetic-joint-state-publisher-gui
```  

A script for installing these dependencies is provided. Before running the script, please use `chmod +x lib_install.sh` to add a permission.  
For **ROS melodic**:
```
./lib_install.sh melodic
```

For **ROS noetic**:
```
./lib_install.sh noetic
```

## Compilation
Place the ***src*** folder from the ***basic_level_sim*** directory into ${your_workspace}.  
Open a new terminal and enter:
```bash
cd ${your_workspace}
catkin_make
source devel/setup.bash
```

## View the Model in RViz
Open a new terminal and enter:
```bash
cd ${your_workspace}
source devel/setup.bash
roslaunch lite3_motion_sim lite3_rviz_display.launch
```

## Simulation
### 1. Load the model
Open a new terminal and enter:
```bash
cd ${your_workspace}
source devel/setup.bash
roslaunch lite3_motion_sim lite3_gazebo_simulation_control.launch
```

### 2. Start the controller
Open a new terminal and enter:
```bash
cd ${your_workspace}/basic_level_sim
source devel/setup.bash
rosrun lite3_motion_sim lite3_motion_sim
```