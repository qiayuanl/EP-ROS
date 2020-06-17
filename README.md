# EP-ROS
ROS Wrapped software stack and some simple example for RoboMaster EP.

Dependencies:
- ROS Melodic
- apriltag_ros
- pybind11
- opus
- avcodec
- swscale

# Getting started 
## Install dependencies
- ROS - http://wiki.ros.org/ROS
- others:
```
pip install pybind11 pyyaml
sudo apt-get install libopus-dev libavcodec-dev libswscale-dev 
sudo apt-get install ros-melodic-apriltag-ros
```
## Download and build code
In your catkin_workspace:
```
git clone git@github.com:QiayuanLiao/EP-ROS.git
cd ./EP-ROS
git submodule update --init --recursive
```
