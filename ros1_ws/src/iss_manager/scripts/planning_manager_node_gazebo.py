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
        return False
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
        rospack = rospkg.RosPack()
        hd_map = os.path.join(rospack.get_path('iss_manager'), "maps", "simple_road_v1.osm")
        projector = UtmProjector(lanelet2.io.Origin(0., 0.))
        loadedMap, load_errors = lanelet2.io.loadRobust(hd_map, projector)
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
        solid_checker = get_solid_checker(loadedMap)
        lanelet2_settings = rospy.get_param("global_planning")["lanelet2_settings"]
        self._global_planner = Lanelet2Planner(loadedMap, traffic_rules, solid_checker, lanelet2_settings)
        self._global_planner_pub = rospy.Publisher("planning/lanelet2_planner/trajectory", StateArray, queue_size=1, latch=True)
        self._global_planner_path_pub = rospy.Publisher("planning/lanelet2_planner/path", Path, queue_size=1, latch=True)
        
        # Motion predictor
        predictor_settings = rospy.get_param("prediction")
        self._motion_predictor = ConstVelPredictor(predictor_settings)
        
        # Motion Planner
        self.local_planning_frequency = rospy.get_param("local_planning")["local_planning_frequency"]
        lattice_settings = rospy.get_param("local_planning")["lattice_settings"]
        self._lattice_planner = LatticePlanner(lattice_settings)
        self._lattice_planner_timer = None
        self._local_planner_pub = rospy.Publisher("planning/lattice_planner/trajectory", StateArray, queue_size=1)
        self._local_planner_path_pub = rospy.Publisher("planning/lattice_planner/path", Path, queue_size=1)
        
        self._set_goal_srv = rospy.Service("planning/set_goal", SetGoal, self._set_goal_srv_callback)

    def _set_goal_srv_callback(self, req):
        while self._ego_state == None:
            rospy.sleep(0.1)
        start_point = (self._ego_state.x, self._ego_state.y, self._ego_state.heading_angle)
        end_point = (req.x, req.y, req.yaw)
        global_traj = self._global_planner.run_step(start_point, end_point)
        if global_traj == None:
            rospy.logerr("Global planning: Failed")
            return SetGoalResponse(False)
        rospy.loginfo("Global planning: Success")
        self._global_planner_pub.publish(traj_to_ros_msg(global_traj))
        self._global_planner_path_pub.publish(traj_to_ros_msg_path(global_traj))
        lane_map = LaneMap()
        self._motion_predictor.update_map(lane_map)
        self._lattice_planner.update(global_traj.get_waypoints())
        self._lattice_planner_timer = rospy.Timer(rospy.Duration(1.0/self.local_planning_frequency), self._local_planning_timer_callback)
        return SetGoalResponse(True)
    
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