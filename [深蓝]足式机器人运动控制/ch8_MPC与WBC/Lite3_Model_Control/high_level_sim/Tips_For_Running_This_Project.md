# Tips for running this project

### Tip1
```bash
gazebo_gui-3  process has died [pid 2255, exit code 139, cmd /opt/ros/melodic/lib/gazebo_ros/gzclient __name:=gazebo_gui __log
```
If the above error occurs, you can refer to this [tutorial](https://blog.csdn.net/gls_nuaa/article/details/122142356) to try to solve it.  
If the error still cannot be resolved, try unstalling Gazebo, then installing Gazebo 11 following this [tutorial](https://zhuanlan.zhihu.com/p/526057704).

### Tip2
```bash
Could not find a package configuration file provided by "Python3" with any of the following names: Python3Config.cmake  python3-config.cmake
```
 If the above error occurs, please upgrade CMake to version 3.12 or above refer to this [tutorial](https://blog.csdn.net/qq_27350133/article/details/121994229).

### Tip3 
```bash
error 3 usr/bin/ld: cannot find -llcm collect2: error: ld returned 1 exit status
```
If the above error occurs, please install the dependency:
```
sudo apt install liblcm-dev
```

### Tip4
```bash
Resource not found: xacro
```
If the above error occurs, please install the dependency:
```
sudo apt-get install ros-melodic-xacro
```

### Tip5 
```bash
sh: 1: rosrun: not found
```
If the above error occurs, please install the dependency:
```
sudo apt install ros-melodic-rosbash
```
### Tip6
If you need to install or upgrade Python, you can use the following script:
```
sudo apt-get install python3.8 python3.8-dev python3.8-distutils python3.8-venv
```




