#!/usr/bin/env python

import rospy
import rospkg
import os
import numpy as np

import lanelet2
from lanelet2.projection import UtmProjector
from planning_utils.lanelet2_utils import get_solid_checker

from global_planner.lanelet2_planner import Lanelet2Planner
from motion_predictor.constant_velocity_predictor import ConstVelPredictor
from local_planner.lattice_planner import LatticePlanner

from iss_msgs.msg import State, StateArray, ObjectDetection3DArray
from iss_msgs.srv import SetGoal

class PlanningManagerNode:
    def __init__(self) -> None:
        self._ego_state_sub = rospy.Subscriber("carla_bridge/gt_state", State._ego_state_callback)
        self._obstacle_sub = rospy.Subscriber("carla_bridge/obstacles", ObjectDetection3DArray, self._obstacle_callback)
        self._ego_state = State()
        self._obstacles = {}
        
        # Global planner 
        rospack = rospkg.RosPack()
        lanelet2_town06 = os.path.join(rospack.get_path('planning'), "maps", "Town06_hy.osm")
        projector = UtmProjector(lanelet2.io.Origin(0., 0.))
        loadedMap, load_errors = lanelet2.io.loadRobust(lanelet2_town06, projector)
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
        solid_checker = get_solid_checker(loadedMap)
        self._global_planner = Lanelet2Planner(loadedMap, traffic_rules, solid_checker)
        
        # Motion predictor
        predictor_settings = dict()
        predictor_settings['dt'] = 0.5
        predictor_settings['MAX_T'] = 6.0
        self._motion_predictor = ConstVelPredictor(predictor_settings)
        
        # Motion Planner
        lattice_settings = dict()
        lattice_settings['MAX_SPEED'] = 60.0 / 3.6     # maximum speed [m/s]
        lattice_settings['MAX_ACCEL'] = 4.0            # maximum acceleration [m/ss], tesla model3: 6.88
        lattice_settings['MAX_CURVATURE'] = 1.0      # maximum curvature [1/m], tesla model3's turning radius: 5.8    
        lattice_settings['D_S'] = 0.5                  # sample Frenet d
        lattice_settings['D_ROAD_W'] = 1.0             # road width sampling length [m]
        lattice_settings['DT'] = 1.0                   # prediction timestep length (s)
        lattice_settings['dt'] = 0.5                   # sample time
        lattice_settings['MAX_T'] = 6.0                # max prediction time [s]
        lattice_settings['MIN_T'] = 4.0                # min prediction time [s]
        lattice_settings['TARGET_SPEED'] = 15.0 / 3.6  # target speed [m/s]
        lattice_settings['D_T_S'] = 5.0 / 3.6          # target speed sampling length [m/s]
        lattice_settings['N_S_SAMPLE'] = 2             # sampling number of target speed    
        lattice_settings['ROBOT_RADIUS'] = 2.0         # robot radius [m]
        lattice_settings['K_J'] = 0.1
        lattice_settings['K_T'] = 0.1
        lattice_settings['K_D'] = 1.0
        lattice_settings['K_LAT'] = 1.0
        lattice_settings['K_LON'] = 0.8
        self._lattice_planner = LatticePlanner(loadedMap, traffic_rules, lattice_settings, solid_checker)
        
        self._trajectory_pub = rospy.Publisher("planning/lattice_planner/trajectory", StateArray, queue_size=1)
        self._set_goal_srv = rospy.Service("planning/set_goal", SetGoal, self._set_goal_srv_callback)
    
    def _set_goal_srv_callback(self, req):
        start_point = (self._ego_state.pose.pose.position.x, self._ego_state.pose.pose.position.y, self._ego_state.yaw)
        end_point = (req.goal.x, req.goal.y, req.goal.yaw)
        trajectory = self._global_planner.run_step(start_point, end_point)
        self._lattice_planner.update(trajectory.get_waypoints())
        local_planning_frequency = rospy.get_param("~local_planning_frequency", 10)
        self._lattice_planner_timer = rospy.Timer(rospy.Duration(1.0/local_planning_frequency), self._local_planning_timer_callback)
        return True
    
    def _ego_state_callback(self, state_msg):
        self._ego_state = state_msg
    
    def _obstacle_callback(self, obstacle_msg):
        self._motion_predictor.update(obstacle_msg)    

    def _local_planning_timer_callback(self, event):
        trajectory = self._lattice_planner.run_step(self._ego_state, self._motion_predictor)
        self._trajectory_pub.publish(trajectory)

if __name__ == "__main__":
    rospy.init_node("planning_manager_node", anonymous=True)
    planning_manager_node = PlanningManagerNode()
    rospy.spin()