# ISS
This repo provides ROS-noetic support for the ISS project, compatible with both CARLA 0.9.13 and Gazebo Classic. The development of this repository is ongoing.

## Installation
- Install (ROS Noetic)[http://wiki.ros.org/noetic/Installation]
- Install (CARLA 0.9.13)[https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz]
- Install this repository:
```
git clone --recursive https://github.com/CAS-LRJ/ISS.git 
cd ISS
git checkout ros1-dev-reframe
python3 setup.py install
cd ros1_ws && catkin build
source devel/setup.bash
```

## Usage
If using CARLA simulator, please launch CARLA simulator firstly. Then do
```
roslaunch carla_bridge carla_demo.launch
```

If using Gazebo simulator, simply do
```
roslaunch robot_gazebo gazebo_demo.launch
```

## Conventions
1. This repository utilizes **ROS's right-handed** coordinate system. This is distinct from **CARLA's left-handed** coordinate system. The ``carla_bridge`` node is responsible for handling the necessary conversions between these two coordinate systems.
2. Angles are in -pi to pi.