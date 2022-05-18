# beginner_tutorials
ROS Beginner Tutorials<br />
<br />
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
## Overview
This project contains a ROS package named "beginner_tutorials".<br />
This project covers a tutorial on how to write a simple publisher and subscriber node in C++.<br />
The package and code is taken directly from [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials).<br />
## Dependencies
### ROS
ROS should be installed on the system. This package is tested on Ubuntu 16.04 LTS with [ROS Kinetic Distribution](http://wiki.ros.org/kinetic).<br />
Installation Instructions can be found [here](http://wiki.ros.org/kinetic/Installation).
### catkin
catkin is a Low-level build system macros and infrastructure for ROS.<br />
catkin is included by default when ROS is installed. It can also be installed with apt-get
```
sudo apt-get install ros-kinetic-catkin
```
## Build Instructions
### Creating a catkin workspace
Create a catkin workspace using following instructions:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Running catkin_make command the first time in your workspace will create a CMakeLists.txt link in your 'src' folder. Before continuing source your new setup.*sh file:
```
$ source devel/setup.bash
```
### Building the Package inside catkin workspace
Clone the package in src folder of catkin workspace using following commands:
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/VBot2410/beginner_tutorials.git
```
Then build the package using following commands:
```
$ cd ~/catkin_ws/
$ catkin_make
```
## Running The Demo
Open a terminal and run following commands:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch beginner_tutorials Tutorial.launch
```
You may also specify custom message and Frequency by using **arg:=value**. For example, to launch with Message "Hi!" & Frequency 100Hz,
```
roslaunch beginner_tutorials Tutorial.launch Message:="Hi!" Frequency:=100
```
For the custom message to take effect, open a new terminal & follow these instructions:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun beginner_tutorials modify
```
### Using rosservice to call the service
Open a new terminal after following the above instructions for running the Demo
```
$ cd ~/catkin_ws
$ source devel/setup.bash
rosservice call /String_Modify Message:"Modified String"
```
The message broadcasted by talker will be changed to "Modified String". You may use any custom string while using above command.
