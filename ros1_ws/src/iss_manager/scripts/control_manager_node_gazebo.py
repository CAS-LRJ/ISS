#!/usr/bin/env python

import rospy
import numpy as np

from ISS.algorithms.control.pid_wpt_tracker import VehiclePIDController
from ISS.algorithms.control.linear_mpc_tracker import VehicleLinearMPCController
from ISS.algorithms.planning.planning_utils.trajectory import Trajectory

from geometry_msgs.msg import Twist
from iss_manager.data_utils import traj_from_ros_msg
from iss_manager.msg import StateArray, State

class WPTTrackerNode:
    def __init__(self) -> None:
        ctrl_freq = rospy.get_param("~control_frequency", 10)
        self._timer = rospy.Timer(rospy.Duration(1 / ctrl_freq), self._timer_callback)
        self._ctrl_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self._ego_state_sub = rospy.Subscriber("ego_state_estimation", State, self._state_callback)
        self._trajectory_sub = rospy.Subscriber("planning/local_planner/trajectory", StateArray, self._trajectory_callback)
        self._ego_state = None
        
        # self._pid_tracker = VehiclePIDController()
        linear_mpc_settings = {
            "acc_table": {0: 0, 0.2: 0.5, 0.4: 0.8, 0.6: 0.9, 0.8: 0.95, 1: 1},
            "nx": 4,
            "nu": 2,
            "N": 10,
            "dt": 0.05,
            "steer_rate_max": np.deg2rad(15.0),
            "speed_max": 50 / 3.6,
            "ego_veh_info": {
                "wheelbase": 2.8,
                "steer_max": np.deg2rad(70.0),
                "acc_max": 8
            },
            "Q": np.diag([2.0, 2.0, 1.0, 4.0]),
            "Qf": np.diag([2.0, 2.0, 1.0, 4.0]),
            "R": np.diag([0.1, 1]),
            "Rd": np.diag([0.1, 1])
        }
        self._pid_tracker = VehiclePIDController()
        self._mpc_tracker = VehicleLinearMPCController(linear_mpc_settings)
        self._trajectory = Trajectory()
        
    def _timer_callback(self, event):
        if self._ego_state is None:
            return
        throttle, steering = self._pid_tracker.run_step(self._ego_state)
        # throttle, steering = self._mpc_tracker.run_step(self._ego_state)
        twist_msg = Twist()
        twist_msg.linear.x = throttle
        twist_msg.angular.z = steering
        self._ctrl_pub.publish(twist_msg)
    
    def _state_callback(self, msg):
        self._ego_state = msg
    
    def _trajectory_callback(self, msg):
        self._trajectory = traj_from_ros_msg(msg)
        # self._mpc_tracker.set_traj(self._trajectory)
        self._pid_tracker.set_traj(self._trajectory.get_states_list())
        

if __name__ == "__main__":
    rospy.init_node("wpt_tracker_node")
    wpt_tracker_node = WPTTrackerNode()
    rospy.spin()