# Installation

- Install [ROS Noetic](http://wiki.ros.org/noetic/Installation), together with the dependencies for building ROS packages:
```
sudo apt-get install ros-noetic-navigation ros-noetic-gmapping ros-noetic-teb-local-planner ros-noetic-ackermann-msgs ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-joint-state-publisher-gui ros-noetic-ros-control ros-noetic-ros-controllers
```
- Install [CARLA 0.9.13](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz) and set the environment variables:
```
export CARLA_ROOT=</path/to/carla>
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
```
- Create a virtual environment for this repository:
```
conda create -n iss python=3.8
conda activate iss
```
- Install [git-lfs](https://git-lfs.github.com/)
- Install this repository:
```
git clone --recurse-submodules https://github.com/CAS-LRJ/ISS.git && cd ISS
pip3 install -r Install/requirements.txt
python3 Install/setup.py develop
```
- Install PyTorch and torch-scatter:
```
pip3 install torch==1.7.1+cu110 torchvision==0.8.2+cu110 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html
pip3 install Install/torch_scatter-2.0.7-cp38-cp38-linux_x86_64.whl
```
- Install [CUDA Toolkit 11.8](https://developer.nvidia.com/cuda-11-8-0-download-archive)
- Install [MMDetection](https://mmdetection.readthedocs.io/en/latest/get_started.html)
