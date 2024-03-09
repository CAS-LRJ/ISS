#!/usr/bin/env python

import rospy
import numpy as np

from ISS.algorithms.control.pid_wpt_tracker import VehiclePIDController
from ISS.algorithms.control.linear_mpc_tracker import VehicleLinearMPCController
from ISS.algorithms.utils.trajectory import Trajectory

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from iss_manager.data_utils import traj_from_ros_msg
from iss_manager.msg import StateArray, State, ControlCommand
from iss_manager.srv import EmergencyStop, EmergencyStopResponse

DEBUG = False
class ControlManagerNode:
    def __init__(self) -> None:
        self._ctrl_freq = rospy.get_param("control")["control_frequency"]
        self._timer = rospy.Timer(rospy.Duration(1 / self._ctrl_freq), self._timer_callback)
        self._ctrl_pub = rospy.Publisher(rospy.get_param("control_command_topic"), ControlCommand, queue_size=1)
        self._ego_state_sub = rospy.Subscriber(rospy.get_param("ego_state_topic"), State, self._state_callback)
        self._trajectory_sub = rospy.Subscriber("planning/local_planner/trajectory", StateArray, self._trajectory_callback)
        self._emergency_stop_srv = rospy.Service("control/emergency_stop", EmergencyStop, self._emergency_stop_callback)
        self._emergency_stop = False
        self._ego_state = None
        
        # linear_mpc_settings = {
        #     "acc_table": {0: 0, 0.2: 0.5, 0.4: 0.8, 0.6: 0.9, 0.8: 0.95, 1: 1},
        #     "nx": 4,
        #     "nu": 2,
        #     "N": 10,
        #     "dt": 0.05,
        #     "steer_rate_max": np.deg2rad(15.0),
        #     "speed_max": 50 / 3.6,
        #     "ego_veh_info": {
        #         "wheelbase": 2.8,
        #         "steer_max": np.deg2rad(70.0),
        #         "acc_max": 8
        #     },
        #     "Q": np.diag([2.0, 2.0, 1.0, 4.0]),
        #     "Qf": np.diag([2.0, 2.0, 1.0, 4.0]),
        #     "R": np.diag([0.1, 1]),
        #     "Rd": np.diag([0.1, 1])
        # }
        # self._mpc_tracker = VehicleLinearMPCController(linear_mpc_settings)

        pid_settings = rospy.get_param("control")["pid_settings"]
        self._pid_tracker = VehiclePIDController(pid_settings["lateral"], pid_settings["longitudinal"], pid_settings["look_ahead"])
        self._trajectory = Trajectory()
        self._ctrl_array = None
        self._ctrl_idx = 0
        self._recorded_states = []
        self._traj_cnt = 0
        self._thro_as_speed = pid_settings["thro_as_speed"]
    
    def _emergency_stop_callback(self, req):
        DURATION_SEC = 1
        self._emergency_stop = True
        rospy.sleep(DURATION_SEC)
        self._emergency_stop = False
        return EmergencyStopResponse(True)
    
    def _timer_callback(self, event):
        if self._ego_state is None:
            return
        ctrl_msg = ControlCommand()
        if self._emergency_stop:
            self._ctrl_pub.publish(ctrl_msg)
            return
        throttle, steering = self._pid_tracker.run_step(self._ego_state, thro_as_speed=self._thro_as_speed)
        if len(self._pid_tracker.traj) != 0:
            self._recorded_states.append([self._ego_state.x, self._ego_state.y, self._ego_state.heading_angle, self._ego_state.velocity])
        ctrl_msg.throttle = throttle
        ctrl_msg.steering = steering
        # print(ctrl_msg.throttle, ctrl_msg.steering)
        self._ctrl_pub.publish(ctrl_msg)
    
    def _state_callback(self, msg):
        if DEBUG:
            rospy.loginfo("ControlManagerNode._state_callback msg.header.seq: " + str(msg.header.seq) + " msg.x: " + str(msg.x) + " msg.y: " + str(msg.y) + " msg.heading_angle: " + str(msg.heading_angle))
        self._ego_state = msg
    
    def _trajectory_callback(self, msg):
        self._trajectory = traj_from_ros_msg(msg)
        states_list = self._trajectory.get_states_list(1 / self._ctrl_freq)
        self._pid_tracker.set_traj(states_list)
        self._traj_cnt += 1       
 
if __name__ == "__main__":
    rospy.init_node("control_manager_node")
    control_manager_node = ControlManagerNode()
    rospy.spin()
