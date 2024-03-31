#!/usr/bin/env python

import rospy
import rospkg
import os
import numpy as np
import copy

import lanelet2
from lanelet2.projection import UtmProjector

from ISS.algorithms.utils.lanelet2_utils import get_solid_checker
from ISS.algorithms.planning.global_planner.lanelet2_planner import Lanelet2Planner
from ISS.algorithms.utils.trajectory import Trajectory
from ISS.algorithms.planning.motion_predictor.constant_velocity_predictor import ConstVelPredictor
from ISS.algorithms.planning.local_planner.motion_primitive.frenet_planner import FrenetPlanner
from ISS.algorithms.planning.local_planner.ilqr.ilqr_wrapper import iLQRPlanner
from ISS.algorithms.planning.local_planner.mpcc.mpcc_wrapper import MPCCPlanner

from iss_manager.data_utils import *

from nav_msgs.msg import Path

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from iss_manager.msg import State, StateArray, StateArrayArray, ObjectDetection3DArray
from iss_manager.srv import SetGoal, SetGoalResponse, EmergencyStop

DEBUG = False
DEBUG_MSGS = False
class PlanningManagerNode:
    def __init__(self) -> None:
        self._ego_state_sub = rospy.Subscriber(rospy.get_param("ego_state_topic"), State, self._ego_state_callback)
        self._ego_state = None
        
        self._world_frame = rospy.get_param("world_frame")
        self._vehicle_info = rospy.get_param("vehicle_info")
        
        # Global planner 
        rospack = rospkg.RosPack()
        hd_map = os.path.join(rospack.get_path('iss_manager'), "maps", rospy.get_param("hd_map"))
        projector = UtmProjector(lanelet2.io.Origin(0., 0.))
        loadedMap, load_errors = lanelet2.io.loadRobust(hd_map, projector)
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
        self.lanemap_collision_checker, solid_points = get_solid_checker(loadedMap, self._vehicle_info["length"], self._vehicle_info["width"])
        lanelet2_settings = rospy.get_param("global_planning")["lanelet2_settings"]
        self._global_planner = Lanelet2Planner(loadedMap, traffic_rules, self.lanemap_collision_checker, lanelet2_settings)
        self._global_planner_pub = rospy.Publisher("planning/global_planner/trajectory", StateArray, queue_size=1, latch=True)
        self._goal_point = None
        
        # Motion predictor
        predictor_settings = rospy.get_param("prediction")
        self._motion_predictor = ConstVelPredictor(predictor_settings, self.lanemap_collision_checker, self._vehicle_info) # put lanemap into the predictor
        self._obstacle_sub = rospy.Subscriber(rospy.get_param("object_detection_topic"), ObjectDetection3DArray, self._obstacle_callback)
        self._prediction_sub = rospy.Subscriber(rospy.get_param("motion_prediction_topic"), StateArrayArray, self._prediction_callback)
        
        # Motion Planner
        self.local_planning_frequency = rospy.get_param("local_planning")["local_planning_frequency"]
        lattice_settings = rospy.get_param("local_planning")["lattice_settings"]
        self._local_coarse_planner = FrenetPlanner(lattice_settings)
        self._local_fine_planner = iLQRPlanner()
        self._local_planner_timer = None
        self._local_planner_pub = rospy.Publisher("planning/local_planner/trajectory", StateArray, queue_size=1)
        self._local_traj = Trajectory()
        self._init_planning_state_prev = None

        self._set_goal_srv = rospy.Service("planning/set_goal", SetGoal, self._set_goal_srv_callback)
        if rospy.get_param("show_hd_map_rviz"):
            self._hd_map_pub = rospy.Publisher("planning/hd_map", Marker, queue_size=1, latch=True)
            hd_map = Marker()
            hd_map.header.frame_id = rospy.get_namespace().replace("/", "") + "/" + self._world_frame
            hd_map.type = Marker.POINTS
            hd_map.action = Marker.ADD
            hd_map.scale.x = 0.02
            hd_map.scale.y = 0.02
            hd_map.lifetime = rospy.Duration(0)
            hd_map.pose.orientation.w = 1.0
            hd_map.color.a = 1.0
            hd_map.color.r = 1.0
            hd_map.color.g = 1.0
            hd_map.color.b = 1.0
            for pt in loadedMap.pointLayer:
                point_msg = Point()
                point_msg.x = pt.x
                point_msg.y = pt.y
                point_msg.z = 0.0
                hd_map.points.append(point_msg)
            self._hd_map_pub.publish(hd_map)

        if DEBUG:
            self._global_planner_path_pub = rospy.Publisher("planning/global_planner/path", Path, queue_size=1, latch=True)
            self._local_planner_path_pub = rospy.Publisher("planning/local_planner/path", Path, queue_size=1)
            self._local_coarse_planner_debug_pub = rospy.Publisher("planning/local_planner/debug", MarkerArray, queue_size=1)

    def _set_goal_srv_callback(self, req):
        if DEBUG_MSGS:
            rospy.loginfo("PlanningManagerMode._set_goal_srv_callback start")
        while self._ego_state is None:
            rospy.sleep(0.1)
        if DEBUG_MSGS:
            rospy.loginfo("PlanningManagerMode._set_goal_srv_callback middle 1")
        start_point = (self._ego_state.x, self._ego_state.y, self._ego_state.heading_angle)
        end_point = (req.x, req.y, req.yaw)
        global_traj = self._global_planner.run_step(start_point, end_point)
        if DEBUG_MSGS:
            rospy.loginfo("PlanningManagerMode._set_goal_srv_callback middle 2")
        self._goal_point = end_point
        if global_traj is None:
            rospy.logerr("Global planner: Failed")
            return SetGoalResponse(False)
        rospy.loginfo("Global planner: Success")
        self._global_planner_pub.publish(traj_to_ros_msg(global_traj, frame_id=self._world_frame))
        self._local_coarse_planner.update_reference_line(global_traj.get_waypoints())
        self._local_planner_timer = rospy.Timer(rospy.Duration(1.0/self.local_planning_frequency), self._local_planning_timer_callback)
        if DEBUG:
            self._global_planner_path_pub.publish(traj_to_ros_msg_path(global_traj, frame_id=self._world_frame))
        if DEBUG_MSGS:
            rospy.loginfo("PlanningManagerMode._set_goal_srv_callback end")
        return SetGoalResponse(True)
    
    def _ego_state_callback(self, state_msg):
        if DEBUG_MSGS:
            rospy.loginfo("PlanningManagerMode._ego_state_callback start")
        self._ego_state = state_msg
        if DEBUG_MSGS:
            rospy.loginfo("PlanningManagerMode._ego_state_callback end")
    
    def _obstacle_callback(self, obstacle_msg):
        if DEBUG_MSGS:
            rospy.loginfo("PlanningManagerMode._obstacle_callback start")
        self._motion_predictor.update_obstacle(obstacle_msg)    
        if DEBUG_MSGS:
            rospy.loginfo("PlanningManagerMode._obstacle_callback end")

    def _prediction_callback(self, prediction_msg):
        if DEBUG_MSGS:
            rospy.loginfo("PlanningManagerMode._prediction_callback start")
        trajectories = []
        for state_array in prediction_msg.trajectories:
            trajectories.append(traj_from_ros_msg(state_array))
        self._motion_predictor.read_prediction(trajectories)
        if DEBUG_MSGS:
            rospy.loginfo("PlanningManagerMode._prediction_callback end")

    def _local_planning_timer_callback(self, event):
        if DEBUG_MSGS:
            rospy.loginfo("PlanningManagerMode._local_planning_timer_callback start")
        if self._ego_state is None:
            if DEBUG_MSGS:
                rospy.loginfo("PlanningManagerMode._local_planning_timer_callback end 1")
            return
        init_planning_state = [self._ego_state.x, self._ego_state.y, self._ego_state.heading_angle, self._ego_state.velocity, self._ego_state.acceleration]
        dist_to_goal = np.linalg.norm(np.array([self._ego_state.x, self._ego_state.y]) - np.array(self._goal_point[:2]))
        if  dist_to_goal < 0.5:
            # TODO: stop the vehicle
            return
        if DEBUG_MSGS:
            rospy.loginfo("PlanningManagerMode._local_planning_timer_callback end 2")
        # if not self._local_traj.is_empty():
        #     init_planning_state[:4]  = self._local_traj.get_closest_point(init_planning_state[0],
        #                                                                   init_planning_state[1],
        #                                                                   init_planning_state[2],
        #                                                                   init_planning_state[3])
        self._local_traj, all_path_vis = self._local_coarse_planner.run_step(init_planning_state, self._init_planning_state_prev, self._motion_predictor)
        self._init_planning_state_prev = init_planning_state
        if self._motion_predictor.check_emergency_stop(self._ego_state.x, self._ego_state.y, self._ego_state.heading_angle, check_solid=False):
            rospy.wait_for_service('control/emergency_stop', timeout=2)
            try:
                emergency_stop = rospy.ServiceProxy('control/emergency_stop', EmergencyStop)
                rospy.logerr("Emergency stop!")
                resp = emergency_stop()
                if DEBUG_MSGS:
                    rospy.loginfo("PlanningManagerMode._local_planning_timer_callback end 3")
                return resp.success
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        if DEBUG:        
            all_path_vis_msg = MarkerArray()
            for i, [path_list, info] in enumerate(all_path_vis):
                marker_msg = Marker()
                marker_msg.header.frame_id = rospy.get_namespace().replace("/", "") + "/" + self._world_frame
                marker_msg.id = i
                marker_msg.type = Marker.LINE_STRIP
                marker_msg.action = Marker.ADD
                marker_msg.pose.orientation.w = 1.0
                marker_msg.scale.x = 0.01
                marker_msg.lifetime = rospy.Duration(10)
                if info == "solid_boundary":
                    marker_msg.color.a = 1.0
                    marker_msg.color.r = 1.0
                    marker_msg.color.g = 1.0
                    marker_msg.color.b = 0.0
                elif info == "obstacle":
                    marker_msg.color.a = 1.0
                    marker_msg.color.r = 0.0
                    marker_msg.color.g = 1.0
                    marker_msg.color.b = 1.0
                elif info == "safe":
                    marker_msg.color.a = 1.0
                    marker_msg.color.r = 0.0
                    marker_msg.color.g = 1.0
                    marker_msg.color.b = 0.0
                else:
                    marker_msg.color.a = 0.5
                    marker_msg.color.r = 1.0
                    marker_msg.color.g = 0.0
                    marker_msg.color.b = 0.0
                for x, y, yaw in path_list:
                    point_msg = Point()
                    point_msg.x = x
                    point_msg.y = y
                    point_msg.z = 0.0
                    marker_msg.points.append(point_msg)
                all_path_vis_msg.markers.append(marker_msg)
            self._local_coarse_planner_debug_pub.publish(all_path_vis_msg)
        if self._local_traj.is_empty():
            rospy.logwarn("Local planner: Failed")
            if DEBUG_MSGS:
                rospy.loginfo("PlanningManagerMode._local_planning_timer_callback end 4")
            return
        # self._local_fine_planner.run_step(self._ego_state, local_traj)
        if DEBUG: 
            self._local_planner_path_pub.publish(traj_to_ros_msg_path(self._local_traj, frame_id=self._world_frame))
        self._local_planner_pub.publish(traj_to_ros_msg(self._local_traj, frame_id=self._world_frame))
        if DEBUG_MSGS:
            rospy.loginfo("PlanningManagerMode._local_planning_timer_callback end 5")

if __name__ == "__main__":
    rospy.init_node("planning_manager_node", anonymous=True)
    planning_manager_node = PlanningManagerNode()
    rospy.spin()
