# ISS
This repo provides ROS-noetic support for the ISS project. The ISS project works with CARLA 0.9.13.

**Node**: In this repo, we follow the right-handed coordinate system, which is different from the left-handed coordinate system in CARLA. The conversion between these two coordinate systems is done in the `carla_bridge` node.

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
cd <CARLA root>/CarlaUE4.sh
```
- Run example:
```
roslaunch carla_bridge carla_bridge.launch
``` 