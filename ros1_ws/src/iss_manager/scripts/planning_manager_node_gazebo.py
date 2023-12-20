#!/usr/bin/env python

import rospy
import rospkg
import os
import numpy as np
import time

import lanelet2
from lanelet2.projection import UtmProjector

from ISS.algorithms.planning.planning_utils.lanelet2_utils import get_solid_checker
from ISS.algorithms.planning.global_planner.lanelet2_planner import Lanelet2Planner
from ISS.algorithms.planning.planning_utils.trajectory import Trajectory
from ISS.algorithms.planning.motion_predictor.constant_velocity_predictor import ConstVelPredictor
from ISS.algorithms.planning.local_planner.lattice_planner import LatticePlanner

from iss_manager.data_utils import *

from nav_msgs.msg import Path
from iss_manager.msg import State, StateArray, ObjectDetection3DArray
from iss_manager.srv import SetGoal, SetGoalResponse

class LaneMap:
    def __init__(self) -> None:
        pass
    
    def check_collision(self, ego_circle_center_array, ego_radius):
        if ego_circle_center_array[1] < (-0.2 + ego_radius):
            return True
        elif ego_circle_center_array[1] > (0.5 - ego_radius):
            return True
        return False

class PlanningManagerNode:
    def __init__(self) -> None:
        self._ego_state_sub = rospy.Subscriber("ego_state_estimation", State, self._ego_state_callback)
        self._obstacle_sub = rospy.Subscriber("object_detection", ObjectDetection3DArray, self._obstacle_callback)
        self._ego_state = None
        
        # Global planner 
        self._global_planner = None
        
        # Motion predictor
        predictor_settings = rospy.get_param("prediction")
        self._motion_predictor = ConstVelPredictor(predictor_settings)
        
        # Motion Planner
        self.local_planning_frequency = rospy.get_param("local_planning")["local_planning_frequency"]
        lattice_settings = rospy.get_param("local_planning")["lattice_settings"]
        self._lattice_planner = LatticePlanner(lattice_settings)
        self._local_planner_pub = rospy.Publisher("planning/lattice_planner/trajectory", StateArray, queue_size=1)
        self._local_planner_path_pub = rospy.Publisher("planning/lattice_planner/path", Path, queue_size=1)
        self._lattice_planner_timer = None
        rospy.sleep(5.0)
        self._setup_planning()
        
    def _setup_planning(self):
        ''' Called when the target is set; Update both map info and global route'''
        lane_map = LaneMap()
        self._motion_predictor.update_map(lane_map)
        global_trajectory = [(round(x * 0.05, 2), -0.2) for x in range(0, int(8 / 0.05) + 1)]
        self._lattice_planner.update(global_trajectory)
        self._lattice_planner_timer = rospy.Timer(rospy.Duration(1.0/self.local_planning_frequency), self._local_planning_timer_callback)        
    
    def _ego_state_callback(self, state_msg):
        self._ego_state = state_msg
    
    def _obstacle_callback(self, obstacle_msg):
        self._motion_predictor.update_obstacle(obstacle_msg)    

    def _local_planning_timer_callback(self, event):
        if self._ego_state is None:
            return
        local_traj = self._lattice_planner.run_step(self._ego_state, self._motion_predictor)
        if local_traj.is_empty():
            rospy.logwarn("Local planning: Failed")
            return
        self._local_planner_path_pub.publish(traj_to_ros_msg_path(local_traj))
        self._local_planner_pub.publish(traj_to_ros_msg(local_traj))

if __name__ == "__main__":
    rospy.init_node("planning_manager_node", anonymous=True)
    planning_manager_node = PlanningManagerNode()
    rospy.spin()