# ISS
This branch intends to adapt ISS to the CARLA Leaderboard 2.0. The code is tested on **Ubuntu 20.04**.

## Installation
Refer [here](Install/INSTALL.md) for installation instructions.

## Usage
First launch the CARLA server by 
```
carla_launch
```
Then launch the leaderboard evaluation by
```
cd ${LEADERBOARD_ROOT} && ./run_evaluation.sh
```


## Conventions
1. This repository utilizes **ROS's right-handed** coordinate system. This is distinct from **CARLA's left-handed** coordinate system. The ``carla_bridge`` node is responsible for handling the necessary conversions between these two coordinate systems.
2. Angles are in -pi to pi.
