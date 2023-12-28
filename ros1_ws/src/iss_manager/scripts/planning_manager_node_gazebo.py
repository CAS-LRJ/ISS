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

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from iss_manager.msg import State, StateArray, ObjectDetection3DArray
from iss_manager.srv import SetGoal, SetGoalResponse

class PlanningManagerNode:
    def __init__(self) -> None:
        self._ego_state_sub = rospy.Subscriber("ego_state_estimation", State, self._ego_state_callback)
        self._obstacle_sub = rospy.Subscriber("object_detection", ObjectDetection3DArray, self._obstacle_callback)
        self._ego_state = None
        
        self.vehicle_info = rospy.get_param("vehicle_info")
        
        # Global planner 
        rospack = rospkg.RosPack()
        hd_map = os.path.join(rospack.get_path('iss_manager'), "maps", "simple_road_v1.osm")
        projector = UtmProjector(lanelet2.io.Origin(0., 0.))
        loadedMap, load_errors = lanelet2.io.loadRobust(hd_map, projector)
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
        lanemap_collision_checker = get_solid_checker(loadedMap, self.vehicle_info["length"], self.vehicle_info["width"])
        lanelet2_settings = rospy.get_param("global_planning")["lanelet2_settings"]
        self._global_planner = Lanelet2Planner(loadedMap, traffic_rules, lanemap_collision_checker, lanelet2_settings)
        self._global_planner_pub = rospy.Publisher("planning/lanelet2_planner/trajectory", StateArray, queue_size=1, latch=True)
        self._global_planner_path_pub = rospy.Publisher("planning/lanelet2_planner/path", Path, queue_size=1, latch=True)
        
        # Motion predictor
        predictor_settings = rospy.get_param("prediction")
        self._motion_predictor = ConstVelPredictor(predictor_settings, lanemap_collision_checker, self.vehicle_info) # put lanemap into the predictor
        
        # Motion Planner
        self.local_planning_frequency = rospy.get_param("local_planning")["local_planning_frequency"]
        lattice_settings = rospy.get_param("local_planning")["lattice_settings"]
        self._lattice_planner = LatticePlanner(lattice_settings)
        self._lattice_planner_timer = None
        self._local_planner_pub = rospy.Publisher("planning/lattice_planner/trajectory", StateArray, queue_size=1)
        self._local_planner_path_pub = rospy.Publisher("planning/lattice_planner/path", Path, queue_size=1)
        
        self._set_goal_srv = rospy.Service("planning/set_goal", SetGoal, self._set_goal_srv_callback)

        self._lattice_planner_debug_pub = rospy.Publisher("planning/lattice_planner/debug", MarkerArray, queue_size=1)

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
        self._global_planner_pub.publish(traj_to_ros_msg(global_traj, frame_id="odom"))
        self._global_planner_path_pub.publish(traj_to_ros_msg_path(global_traj, frame_id="odom"))
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
        local_traj, all_path_vis = self._lattice_planner.run_step(self._ego_state, self._motion_predictor)
        
        all_path_vis_msg = MarkerArray()
        for i, [path_list, unsafe] in enumerate(all_path_vis):
            marker_msg = Marker()
            marker_msg.header.frame_id = rospy.get_namespace().replace("/", "") + "/odom"
            marker_msg.id = i
            marker_msg.type = Marker.LINE_STRIP
            marker_msg.action = Marker.ADD
            marker_msg.scale.x = 0.01
            marker_msg.lifetime = rospy.Duration(0.2)
            if unsafe:
                marker_msg.color.a = 1.0
                marker_msg.color.r = 1
                marker_msg.color.g = 0.0
                marker_msg.color.b = 0.0
            else:
                marker_msg.color.a = 1.0
                marker_msg.color.r = 0.0
                marker_msg.color.g = 1.0
                marker_msg.color.b = 0.0
            for x, y, yaw in path_list:
                point_msg = Point()
                point_msg.x = x
                point_msg.y = y
                point_msg.z = 0.0
                marker_msg.points.append(point_msg)
            all_path_vis_msg.markers.append(marker_msg)
        self._lattice_planner_debug_pub.publish(all_path_vis_msg)
        
        if local_traj.is_empty():
            rospy.logwarn("Local planning: Failed")
            print(self._ego_state)
            return
        self._local_planner_path_pub.publish(traj_to_ros_msg_path(local_traj, frame_id="odom"))
        self._local_planner_pub.publish(traj_to_ros_msg(local_traj, frame_id="odom"))

if __name__ == "__main__":
    rospy.init_node("planning_manager_node", anonymous=True)
    planning_manager_node = PlanningManagerNode()
    rospy.spin()