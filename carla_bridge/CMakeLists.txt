cmake_minimum_required(VERSION 3.0.2)
project(carla_bridge)


find_package(catkin REQUIRED COMPONENTS
  iss_msgs
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES carla_bridge
 CATKIN_DEPENDS iss_msgs roscpp rospy
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
  scripts/carla_bridge/carla_visualizer.py
  scripts/carla_bridge/gt_state_estimator.py
  scripts/carla_bridge/gt_object_detector.py
  scripts/carla_agent/__init__.py 
  scripts/carla_agent/basic_agent.py 
  scripts/carla_agent/behavior_agent.py 
  scripts/carla_agent/behavior_types.py 
  scripts/carla_agent/carla_utils.py 
  scripts/carla_agent/controller.py 
  scripts/carla_agent/global_route_planner.py 
  scripts/carla_agent/local_planner.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

