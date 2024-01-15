import rospy
import numpy as np
import carla
from geometry_msgs.msg import Twist
from iss_manager.msg import ControlCommand
from iss_manager.srv import SetGoal

class ControllerBridge:
    def __init__(self, vehicle) -> None:
        self._vehicle = vehicle
        self._tele_op_sub = rospy.Subscriber("/cmd_vel", Twist, self._teleop_callback)
        self._control = carla.VehicleControl()
        self._agent_sub = None
    
    def start_iss_agent(self, destination):
        self._agent_sub = rospy.Subscriber(rospy.get_param("control_command_topic"), ControlCommand, self._agent_sub_callback)
        return self._call_set_goal_srv(destination)
    
    def _call_set_goal_srv(self, goal):
        rospy.wait_for_service('planning/set_goal', timeout=2)
        try:
            set_goal = rospy.ServiceProxy('planning/set_goal', SetGoal)
            resp = set_goal(goal.location.x, -goal.location.y, -np.deg2rad(goal.rotation.yaw))
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False
    
    def _agent_sub_callback(self, msg):
        self.set_control(msg.throttle, msg.steering)
        # rospy.loginfo("[simulator] throttle: %.2f,  steering: %.2f" % (msg.throttle, msg.steering))
    
    def _simple_agent_tick(self, event):
        self._control = self._simple_agent.run_step()
    
    def _teleop_callback(self, msg):
        throttle = msg.linear.x
        steering = msg.angular.z
        scale_linear = rospy.get_param("scale_linear", 0.5)
        scale_angular = rospy.get_param("scale_angular", 0.5)
        self.set_control(throttle * scale_linear, steering * scale_angular)
    
    def apply_control(self, ctrl=None):
        if ctrl is not None:
            self._vehicle.apply_control(ctrl)
        else:
            self._vehicle.apply_control(self._control)   
        
    def set_control(self, throttle, steering):
        self._control.steer = min(max(-steering, -1.0), 1.0)
        if throttle < 0:
            self._control.throttle = 0
            self._control.brake = min(-throttle, 1.0)
        else:
            self._control.throttle = min(throttle, 1.0)
            self._control.brake = 0
