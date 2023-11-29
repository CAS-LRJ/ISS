# ISS
This repo provides ROS-noetic support for the ISS project, compatible with CARLA 0.9.13. The development of this repository is ongoing.

**Note**: This repository utilizes **ROS's right-handed** coordinate system. This is distinct from **CARLA's left-handed** coordinate system. The ``carla_bridge`` node is responsible for handling the necessary conversions between these two coordinate systems.

## Install
- Install [ROS-noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) 
- Install [CARLA 0.9.13](https://carla.readthedocs.io/en/0.9.13/start_quickstart/), also the [additional maps](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.13.tar.gz). Simply put the additional maps zip file into the `<CARLA_ROOT>/Import` folder and run `bash <CARLA_ROOT>/ImportAssets.sh` to import the maps.
- Install this repo:
```
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone -b ros1-dev <this repo>
cd <this repo>
```
- Anaconda can be used for installation. Run following command to create a virtual environment:
```
conda create --name iss python=3.8
conda activate iss
pip3 install -r requirements.txt
```
- Install other dependencies
```
sudo apt-get install ros-noetic-teleop-twist-keyboard
```
- Build:
```
cd ~/catkin_ws/src && catkin_make
source devel/setup.bash
```

## Run tasks
- Run CARLA server:
```
bash <CARLA root>/CarlaUE4.sh -quality-level=Low -windowed
```
- Run example:
```
roslaunch carla_bridge carla_bridge.launch simple_agent_demo:=true
``` 