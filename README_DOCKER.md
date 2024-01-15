# Project Setup Instructions

This README provides instructions for setting up the development environment using Docker, installing necessary dependencies, and running simulations with the CARLA or Gazebo simulator.

## 1. Prerequisites

Before proceeding with the setup, ensure the following prerequisites are met:

- **Docker:** Docker must be installed on your system. You can download and install Docker from [Docker's official website](https://www.docker.com/get-started).

- **NVIDIA GPU and CUDA:** If you are using an NVIDIA GPU, ensure that you have the appropriate NVIDIA drivers and CUDA installed on your host system. CUDA is necessary for GPU-accelerated applications. You can download CUDA from [NVIDIA's CUDA Zone](https://developer.nvidia.com/cuda-downloads).

- **NVIDIA Container Toolkit:** Ensure that the NVIDIA Container Toolkit is installed. This toolkit allows Docker containers to access the GPU for CUDA workloads. Installation instructions can be found on the [NVIDIA Container Toolkit documentation page](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).


## 2. Docker Setup

### 2.1. Pull the NVIDIA CUDA Docker Image:

Before pulling the NVIDIA CUDA Docker image, ensure that you select an image compatible with your computer's CUDA version. You can choose from various versions of NVIDIA's official devel images to match your setup.

To pull a specific version of the NVIDIA CUDA Docker image, use the following command:
```bash
docker pull nvidia/cuda:11.3.1-devel-ubuntu20.04
```

### 2.2. Run the Docker container 

```bash
sudo docker run -it -u 0 --gpus all --network="host" \
 -e GDK_SCALE \
 -e GDK_DPI_SCALE \
 -e XDG_RUNTIME_DIR=/tmp \
 -e DISPLAY=unix$DISPLAY \
 -e ROS_MASTER_URI=http://localhost:11311 \
 --privileged \
 -v /dev:/dev \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 nvidia/cuda:11.3.1-devel-ubuntu20.04
 ```


## 3. Environment Setup 

### 3.1. Update Package List and Install Essential Tools

```bash
apt-get update && apt-get install -y sudo && sudo apt-get update && sudo apt-get install -y python3 python3-pip vim git wget lsb-release && sudo pip3 install --upgrade pip
```

### 3.2. Install ROS and ROS packages:

#### Ubuntu install of ROS Noetic:

http://wiki.ros.org/noetic/Installation/Ubuntu

#### Or You can choose install fishros

```bash
wget http://fishros.com/install -O fishros && . fishros 
```

#### Install ROS packages:

```bash
sudo apt-get install ros-noetic-navigation ros-noetic-gmapping ros-noetic-teb-local-planner ros-noetic-ackermann-msgs ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-joint-state-publisher-gui ros-noetic-ros-control ros-noetic-ros-controllers
```

### 4. Install CARLA

**CARLA:** You have the option to run Carla either on your system or within a Docker environment. This flexibility is possible because, during the creation of the Docker container, the environment variable --network="host" is set, allowing for network communication with the host machine. 

If you choose to run Carla directly on your system, you need to install Carla in both your system environment and within Docker. It's advisable to use shared folders for this setup to facilitate easy management of the environment variables. You can download and install Docker from the[ CARLA Simulator wiki ](https://carla.readthedocs.io)


### 5. Setting Up the ISS Project

### 5.1. Clone the ISS repository and set up the environment

```bash
git clone --recurse-submodules https://github.com/CAS-LRJ/ISS.git
cd ISS
git checkout ros1-dev-reframe
pip3 install -r requirements.txt
python3 setup.py develop
cd ros1_ws && catkin build
source devel/setup.bash
```

## 6. Running Simulations

Before running any simulations, it's necessary to allow the Docker container to display GUI applications on your host's X server. Run the following command:

```bash
xhost +
```

#### Using CARLA Simulator
If you are using the CARLA simulator, launch CARLA first. Then, execute the following command:

```bash
roslaunch carla_bridge carla_demo.launch
```

#### Using Gazebo Simulator
For using the Gazebo simulator, run:

```bash
roslaunch robot_gazebo gazebo_demo.launch
```

## Common Issues and Solutions

For information on common issues encountered during the installation and usage of this project, along with their respective solutions or workarounds, please refer to our [Issues](https://github.com/CAS-LRJ/ISS/issues) section. 

One of our notable issues, which includes a comprehensive discussion on several key challenges, can be found here: [Issue #31](https://github.com/CAS-LRJ/ISS/issues/31).

We encourage users to review these discussions for troubleshooting tips and to gain insights on how to resolve any problems you might encounter. Additionally, feel free to contribute by reporting new issues or suggesting improvements.
