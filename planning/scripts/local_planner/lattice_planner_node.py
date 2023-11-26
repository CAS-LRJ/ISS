from local_planner.lattice_planner import LatticePlanner
import rospy
from iss_msgs.msg import State, StateArray, DetectionArray

class LatticePlannerNode:
    def __init__(self) -> None:
        planning_frequency = rospy.get_param("~planning_frequency", 10)
        self._timer = rospy.Timer(rospy.Duration(1.0/planning_frequency), self._timer_callback)
        self._lattice_planner = LatticePlanner()
        self._state_sub = rospy.Subscriber("carla_bridge/gt_state", State._state_callback)
        self._obstacle_sub = rospy.Subscriber("carla_bridge/obstacles", DetectionArray, self._obstacle_callback)
        self._ego_state = State()
        self._obstacles = {}
        self._trajectory_pub = rospy.Publisher("lattice_planner/trajectory", StateArray, queue_size=1)
    
    def _state_callback(self, state_msg):
        self._ego_state = state_msg
    
    def _obstacle_callback(self, obstacle_msg):
        for detection in obstacle_msg.detections:
            self._obstacles[detection.id] = detection    

    def _timer_callback(self, event):
        trajectory = self._lattice_planner.run_step(self._ego_state, self._obstacles)
        self._trajectory_pub.publish(trajectory)

if __name__ == "__main__":
    rospy.init_node("lattice_planner_node", anonymous=True)
    lattice_planner = LatticePlanner()
    rospy.spin()