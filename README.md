# ISS
This branch is for adding end-to-end learning methods to the ISS framework. The code is tested on CARLA 0.9.13 and ROS Noetic.

## Installation
- Install [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- Install CARLA 0.9.13
- Create a virtual environment for this repository:
```
conda create -n iss_py38 python=3.8
conda activate iss_py38
```
- Install PyTorch and torch-scatter:
```
pip3 install torch==1.7.1+cu110 torchvision==0.8.2+cu110 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html
pip3 install install/torch_scatter-2.0.7-cp38-cp38-linux_x86_64.whl
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