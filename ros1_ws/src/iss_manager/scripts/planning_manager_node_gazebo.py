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

from iss_manager.data_utils import traj_to_ros_msg

from iss_manager.msg import State, StateArray, ObjectDetection3DArray
from iss_manager.srv import SetGoal, SetGoalResponse

class PlanningManagerNode:
    def __init__(self) -> None:
        self._ego_state_sub = rospy.Subscriber("ego_state_estimation", State, self._ego_state_callback)
        self._obstacle_sub = rospy.Subscriber("object_detection", ObjectDetection3DArray, self._obstacle_callback)
        self._ego_state = None
        
        # Global planner 
        self._global_planner = None
        
        # Motion predictor
        predictor_settings = dict()
        predictor_settings['dt'] = 0.5
        predictor_settings['MAX_T'] = 6.0
        predictor_settings['ego_veh_info'] = { # Tesla Model 3
            'length': 4.69,
            'width': 2.0,
            'wheelbase': 2.8,
            'overhang_rear': 0.978,
            'overhang_front': 0.874
        }
        self._motion_predictor = ConstVelPredictor(predictor_settings)
        
        # Motion Planner
        lattice_settings = dict()
        lattice_settings['MAX_SPEED'] = 1    # maximum speed [m/s]
        lattice_settings['MAX_ACCEL'] = 1            # maximum acceleration [m/ss], tesla model3: 6.88
        lattice_settings['MAX_CURVATURE'] = 1.0      # maximum curvature [1/m], tesla model3's turning radius: 5.8    
        lattice_settings['D_S'] = 1                  # sample Frenet d
        lattice_settings['D_ROAD_W'] = 1.0             # road width sampling length [m]
        lattice_settings['DT'] = 1                   # prediction timestep length (s)
        lattice_settings['dt'] = 0.25                   # sample time
        lattice_settings['MAX_T'] = 6.0                # max prediction time [s]
        lattice_settings['MIN_T'] = 4.0                # min prediction time [s]
        lattice_settings['TARGET_SPEED'] = 0.5  # target speed [m/s]
        lattice_settings['D_T_S'] = 0.2          # target speed sampling length [m/s]
        lattice_settings['N_S_SAMPLE'] = 4             # sampling number of target speed    
        lattice_settings['K_J'] = 0.1
        lattice_settings['K_T'] = 0.1
        lattice_settings['K_D'] = 2.0
        lattice_settings['K_LAT'] = 1.0
        lattice_settings['K_LON'] = 0.8
        lattice_settings['d_r'] = 0.5
        lattice_settings['d_l'] = 0.5
        self._lattice_planner = LatticePlanner(loadedMap, traffic_rules, lattice_settings, solid_checker)
        
        self._global_planner_pub = None
        self._local_planner_pub = rospy.Publisher("planning/lattice_planner/trajectory", StateArray, queue_size=1)
        self._set_goal_srv = None
        self._lattice_planner_timer = None
        self._set_goal_srv_callback(None)
    
    def _set_goal_srv_callback(self, req):
        while self._ego_state == None:
            rospy.sleep(0.1)
        waypoints = [(0, -0.2, 0)]
        for i in range(1, 10, 0.1):
            waypoints.append((i, -0.2, 0))
        global_traj = Trajectory()
        global_traj.update_waypoints(waypoints)
        rospy.loginfo("Global planning: Success")
        self._lattice_planner.update(global_traj.get_waypoints())
        local_planning_frequency = rospy.get_param("~local_planning_frequency")
        self._lattice_planner_timer = rospy.Timer(rospy.Duration(1.0/local_planning_frequency), self._local_planning_timer_callback)
        return SetGoalResponse(True)
    
    def _ego_state_callback(self, state_msg):
        self._ego_state = state_msg
    
    def _obstacle_callback(self, obstacle_msg):
        self._motion_predictor.update(obstacle_msg)    

    def _local_planning_timer_callback(self, event):
        local_traj = self._lattice_planner.run_step(self._ego_state, self._motion_predictor)
        if local_traj.is_empty():
            rospy.logwarn("Local planning: Failed")
            return
        self._local_planner_pub.publish(traj_to_ros_msg(local_traj))
        if self._global_planner.is_goal_reached(self._ego_state):
            self._lattice_planner_timer.shutdown()
            rospy.loginfo("Goal reached!")

if __name__ == "__main__":
    rospy.init_node("planning_manager_node", anonymous=True)
    planning_manager_node = PlanningManagerNode()
    rospy.spin()