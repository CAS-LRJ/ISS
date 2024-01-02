#!/usr/bin/env python

import numpy as np

from ISS.algorithms.utils.trajectory import Trajectory

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from iss_manager.msg import State, StateArray

def traj_to_ros_msg(trajectory: Trajectory, frame_id="map"):
    trajectory_msg = StateArray()
    namespaces = rospy.get_namespace().replace("/", "") + "/"
    trajectory_msg.header.frame_id = namespaces + frame_id
    trajectory_msg.header.stamp = rospy.Time.now()
    states = trajectory.get_states_array()
    if states is None:
        return trajectory_msg
    for i in range(states.shape[0]):
        state_msg = State()
        state_msg.header.frame_id = namespaces + frame_id
        state_msg.header.stamp = rospy.Time.now()
        state_msg.x = states[i][0]
        state_msg.y = states[i][1]
        state_msg.heading_angle = states[i][2]
        if states.shape[1] > 3: # consider only the waypoints
            state_msg.velocity = states[i][3]
            state_msg.steering_angle = states[i][4]
            state_msg.acceleration = states[i][5]
            state_msg.jerk = states[i][6]
            state_msg.steering_angle_velocity = states[i][7]
            state_msg.time_from_start = states[i][8]
            trajectory_msg.states.append(state_msg)
    return trajectory_msg

def traj_to_ros_msg_path(trajectory: Trajectory, frame_id="map"):
    path_msg = Path()
    namespaces = rospy.get_namespace().replace("/", "") + "/"
    path_msg.header.frame_id = namespaces + frame_id
    path_msg.header.stamp = rospy.Time.now()
    states = trajectory.get_states_array()
    if states is None:
        return path_msg
    for i in range(states.shape[0]):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = namespaces + frame_id
        pose_msg.pose.position.x = states[i][0]
        pose_msg.pose.position.y = states[i][1]
        pose_msg.pose.position.z = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, states[i][2])
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        path_msg.poses.append(pose_msg)
    return path_msg

def traj_from_ros_msg(trajectory_msg: StateArray):
    trajectory = Trajectory()
    states = []
    for i in range(len(trajectory_msg.states)):
        state = []
        state.append(trajectory_msg.states[i].x)
        state.append(trajectory_msg.states[i].y)
        state.append(trajectory_msg.states[i].heading_angle)
        state.append(trajectory_msg.states[i].velocity)
        state.append(trajectory_msg.states[i].steering_angle)
        state.append(trajectory_msg.states[i].acceleration)
        state.append(trajectory_msg.states[i].jerk)
        state.append(trajectory_msg.states[i].steering_angle_velocity)
        state.append(trajectory_msg.states[i].time_from_start)        
        states.append(state)
    trajectory.update_states_from_list(states)
    return trajectory
