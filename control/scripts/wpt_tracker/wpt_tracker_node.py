#!/usr/bin/env python

from wpt_tracker.pid_wpt_tracker import VehiclePIDController
from iss_msgs.msg import ControlCommand, StateArray, State
from planning_utils.trajectory import Trajectory
import rospy

class WPTTrackerNode:
    def __init__(self) -> None:
        self._pid_tracker = VehiclePIDController()
        ctrl_freq = rospy.get_param("~control_frequency", 10)
        self._timer = rospy.Timer(rospy.Duration(1 / ctrl_freq), self._timer_callback)
        self._ctrl_pub = rospy.Publisher("control/wpt_tracker/control_command", ControlCommand, queue_size=1)
        self._ego_state_sub = rospy.Subscriber("carla_bridge/gt_state", State, self._state_callback)
        self._trajectory_sub = rospy.Subscriber("planning/local_planner/trajectory", StateArray, self._trajectory_callback)
        self._ego_state = None
        
    def _timer_callback(self, event):
        if self._ego_state is None:
            return
        throttle, steering = self._pid_tracker.run_step(self._ego_state)
        self._ctrl_pub.publish(ControlCommand(steering=steering, throttle=throttle))
    
    def _state_callback(self, msg):
        self._ego_state = msg
    
    def _trajectory_callback(self, msg):
        trajectory  = Trajectory()
        trajectory.from_ros_msg(msg)
        self._pid_tracker.set_traj(trajectory.get_states())

if __name__ == "__main__":
    rospy.init_node("wpt_tracker_node")
    wpt_tracker_node = WPTTrackerNode()
    rospy.spin()