# ELMO_MAXON_ROS
This is a C++ library for elmo driver, providing a high-level interface for controlling elmo motor or maxon.
## BUILD THE SOURCE 
```shell
mkdir src
cd src
git clone git@github.com:zouyuelin/elmo_control_ros.git
cd ..
catkin_make
```
## USAGE
```shell
sudo su
source devel/setup.sh
rosrun elmo_control elmo_control_ros path_to/Setup.yaml
```
Publish the topic /elmo_velocity or /elmo_torque

## CONFIG
You can also change the yaml file for matching your own motor's features.
