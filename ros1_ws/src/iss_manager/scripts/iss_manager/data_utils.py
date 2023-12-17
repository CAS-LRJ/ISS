#!/usr/bin/env python

import numpy as np

from ISS.algorithms.planning.planning_utils.trajectory import Trajectory

from iss_manager.msg import State, StateArray

def traj_to_ros_msg(trajectory: Trajectory):
    trajectory_msg = StateArray()
    states = trajectory.get_states_array()
    for i in range(states.shape[0]):
        state_msg = State()
        state_msg.x = states[i][0]
        state_msg.y = states[i][1]
        state_msg.heading_angle = states[i][2]
        if states.shape[1] > 3: # consider only the waypoints
            state_msg.velocity = states[i][3]
            state_msg.acceleration = states[i][4]
            state_msg.jerk = states[i][5]
            state_msg.steering_angle = states[i][6]
            state_msg.steering_angle_velocity = states[i][7]
            state_msg.time_from_start = states[i][8]
            trajectory_msg.states.append(state_msg)
    return trajectory_msg

def traj_from_ros_msg(trajectory_msg: StateArray):
    trajectory = Trajectory()
    states = []
    for i in range(len(trajectory_msg.states)):
        state = []
        state.append(trajectory_msg.states[i].x)
        state.append(trajectory_msg.states[i].y)
        state.append(trajectory_msg.states[i].heading_angle)
        state.append(trajectory_msg.states[i].velocity)
        state.append(trajectory_msg.states[i].acceleration)
        state.append(trajectory_msg.states[i].jerk)
        state.append(trajectory_msg.states[i].steering_angle)
        state.append(trajectory_msg.states[i].steering_angle_velocity)
        state.append(trajectory_msg.states[i].time_from_start)        
        states.append(state)
    trajectory.update_states_from_list(states)
    return trajectory
