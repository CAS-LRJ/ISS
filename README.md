# ISS
Intelligent Self-driving System (ISS) is an autonomous driving framework for research.

## Installation
- Install [ROS Noetic](http://wiki.ros.org/noetic/Installation), together with the dependencies for building ROS packages:
```
sudo apt-get install ros-noetic-navigation ros-noetic-gmapping ros-noetic-teb-local-planner ros-noetic-ackermann-msgs ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-joint-state-publisher-gui ros-noetic-ros-control ros-noetic-ros-controllers
```
- Install [CARLA 0.9.13](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz)
- Create a virtual environment for this repository:
```
conda create -n iss python=3.8
conda activate iss
```
- Install PyTorch and torch-scatter:
```
pip3 install torch==1.7.1+cu110 torchvision==0.8.2+cu110 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html
pip3 install install/torch_scatter-2.0.7-cp38-cp38-linux_x86_64.whl
```
- Install this repository:
```
git clone --recursive https://github.com/CAS-LRJ/ISS.git 
cd ISS && git checkout ros1-dev-reframe
pip3 install -r install/requirements.txt
python3 setup.py develop
```

## Usage
If using ROS-Noetic, first build the workspace by
```
cd ros1_ws && catkin build
source devel/setup.bash
```
If using CARLA, do
```
roslaunch carla_bridge carla_demo.launch 
```

## Conventions
1. This repository utilizes **ROS's right-handed** coordinate system. This is distinct from **CARLA's left-handed** coordinate system. The ``carla_bridge`` node is responsible for handling the necessary conversions between these two coordinate systems.
2. Angles are in -pi to pi.