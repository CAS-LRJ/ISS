# ISS
Intelligent Self-driving System (ISS) is an autonomous driving framework for research. The code is tested on **Ubuntu 20.04**.

## Installation
Refer [here](Install/INSTALL.md) for installation instructions.

## Usage
If using ROS-Noetic, first build the workspace by
```
cd ros1_ws && catkin build
source devel/setup.bash # or setup.zsh
```
If using CARLA, first launch the CARLA server, then do
```
roslaunch carla_bridge carla_demo.launch 
```
If using Gazebo, do
```
roslaunch robot_gazebo gazebo_demo.launch
```

## Documentation
Refer [here](https://tis.ios.ac.cn/iss/) for the documentation.

## Conventions
1. This repository utilizes **ROS's right-handed** coordinate system. This is distinct from **CARLA's left-handed** coordinate system. The ``carla_bridge`` node is responsible for handling the necessary conversions between these two coordinate systems.
2. Angles are in -pi to pi.
