# ISS
This branch is for adding end-to-end learning methods to the ISS framework. The code is tested on CARLA 0.9.10.1 and ROS Noetic.

## Installation
- Install [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- Install CARLA 0.9.10.1
- Create a virtual environment for this repository:
```
conda create -n iss_py37 python=3.7
conda activate iss_py37
```
- Install this repository:
```
git clone --recursive https://github.com/CAS-LRJ/ISS.git 
cd ISS
git checkout ros1-dev-reframe
pip3 install -r requirements.txt
python3 setup.py develop
cd ros1_ws && catkin build
source devel/setup.bash
```

## Conventions
1. This repository utilizes **ROS's right-handed** coordinate system. This is distinct from **CARLA's left-handed** coordinate system. The ``carla_bridge`` node is responsible for handling the necessary conversions between these two coordinate systems.
2. Angles are in -pi to pi.