from wpt_tracker.pid_wpt_tracker import VehiclePIDController
from iss_msgs.msg import ControlCommand, StateArray, State

import rospy

class WPTTrackerNode:
    def __init__(self) -> None:
        self._pid_tracker = VehiclePIDController()
        ctrl_freq = rospy.get_param("~control_frequency", 10)
        self._timer = rospy.Timer(rospy.Duration(1 / ctrl_freq), self._timer_callback)
        self._ctrl_pub = rospy.Publisher("wpt_tracker/control_command", ControlCommand, queue_size=1)
        self._state_sub = rospy.Subscriber("state_estimator/current_state", State, self._state_callback)
        self._obstacle_sub = rospy.Subscriber("obstacle_detector/obstacle", State, self._obstacle_callback)
        self.current_state = None
        self.last_obstacle = None
        
    def _timer_callback(self, event):
        if self.current_state is None or self.last_obstacle is None:
            return
        steering, throttle = self._pid_tracker.run_step(self.current_state, self.last_obstacle)
        self._ctrl_pub.publish(ControlCommand(steering=steering, throttle=throttle))
    
    def _state_callback(self, msg):
        self.current_state = msg
    
    def _obstacle_callback(self, msg):
        self.last_obstacle = msg


if __name__ == "__main__":
    rospy.init_node("wpt_tracker_node")
    wpt_tracker_node = WPTTrackerNode()
    rospy.spin()