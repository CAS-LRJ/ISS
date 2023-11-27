# ISS
This repo provides ROS-noetic support for the ISS project, compatible with CARLA 0.9.13. The development of this repository is ongoing.

**Note**: This repository utilizes **ROS's right-handed** coordinate system. This is distinct from **CARLA's left-handed** coordinate system. The ``carla_bridge`` node is responsible for handling the necessary conversions between these two coordinate systems.

## Install
- Install [ROS-noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and [CARLA 0.9.13](https://carla.readthedocs.io/en/0.9.13/start_quickstart/).
- Anaconda can be used for installation. Run following command to create a virtual environment:
```
conda create --name iss python=3.8
pip3 install -r requirements.txt
conda activate iss
```
- Install this repo:
```
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone -b ros1-dev <this repo>
cd .. && catkin_make
```
## Run tasks
- Run CARLA server:
```
./<CARLA root>/CarlaUE4.sh -quality-level=Low
```
- Run example:
```
roslaunch carla_bridge carla_bridge.launch simple_agent_demo:=true
``` 