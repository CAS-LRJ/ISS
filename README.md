# ISS
This repo provides ROS-noetic support for the ISS project, compatible with CARLA 0.9.13. The development of this repository is ongoing.

**Note**: This repository utilizes **ROS's right-handed** coordinate system. This is distinct from **CARLA's left-handed** coordinate system. The ``carla_bridge`` node is responsible for handling the necessary conversions between these two coordinate systems.

## Installation
Basically,
```
git clone --recursive https://github.com/CAS-LRJ/ISS.git 
cd ISS
git checkout ros1-dev-reframe
python3 setup.py install
cd ros1_ws && catkin build
source devel/setup.bash
```

## Usage
1. Launch CARLA server
2. `roslaunch iss_manager demo.launch`