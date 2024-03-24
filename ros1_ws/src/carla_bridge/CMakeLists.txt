cmake_minimum_required(VERSION 3.0.2)
project(carla_bridge)


find_package(catkin REQUIRED COMPONENTS
  iss_manager
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES carla_bridge
 CATKIN_DEPENDS iss_manager roscpp rospy 
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/carla_bridge/__init__.py
  scripts/carla_bridge/carla_bridge_node.py
  scripts/carla_bridge/carla_bridge_sensors_node.py
  scripts/carla_bridge/carla_visualizer.py
  scripts/carla_bridge/state_estimator.py
  scripts/carla_bridge/object_detector.py
  scripts/carla_bridge/controller_interface.py
  scripts/carla_bridge/sensor_interface.py
  scripts/carla_bridge/route_manipulation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

