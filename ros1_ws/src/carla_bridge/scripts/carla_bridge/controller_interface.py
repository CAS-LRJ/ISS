import rospy
import numpy as np
import carla
from geometry_msgs.msg import Twist
from iss_manager.msg import ControlCommand
from iss_manager.srv import SetGoal

DEBUG_MSGS = False
class ControllerInterface:
    def __init__(self, vehicle) -> None:
        self._vehicle = vehicle
        self._tele_op_sub = rospy.Subscriber("/cmd_vel", Twist, self._teleop_callback)
        self._control = carla.VehicleControl()
        self._agent_sub = None
    
    def start_iss_agent(self, goal):
        self._agent_sub = rospy.Subscriber(rospy.get_param("control_command_topic"), ControlCommand, self._agent_sub_callback)
        rospy.wait_for_service('planning/set_goal', timeout=2)
        try:
            set_goal = rospy.ServiceProxy('planning/set_goal', SetGoal)
            resp = set_goal(goal.location.x, -goal.location.y, -np.deg2rad(goal.rotation.yaw))
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False
    
    def _agent_sub_callback(self, msg):
        if DEBUG_MSGS:
            rospy.loginfo("ControllerInterface._agent_sub_callback")
        self.set_control(msg.throttle, msg.steering)
        # rospy.loginfo("[simulator] throttle: %.2f,  steering: %.2f" % (msg.throttle, msg.steering))
        if DEBUG_MSGS:
            rospy.loginfo("ControllerInterface._agent_sub_callback done")
    
    def _simple_agent_tick(self, event):
        self._control = self._simple_agent.run_step()
    
    def _teleop_callback(self, msg):
        if DEBUG_MSGS:
            rospy.loginfo("ControllerInterface._teleop_callback")
        throttle = msg.linear.x
        steering = msg.angular.z
        scale_linear = rospy.get_param("scale_linear", 0.5)
        scale_angular = rospy.get_param("scale_angular", 0.5)
        self.set_control(throttle * scale_linear, steering * scale_angular)
        if DEBUG_MSGS:
            rospy.loginfo("ControllerInterface._teleop_callback done")
    
    def apply_control(self, ctrl=None):
        if DEBUG_MSGS:
            rospy.loginfo("ControllerInterface.apply_control")
        if ctrl is not None:
            if DEBUG_MSGS:
                rospy.loginfo("ControllerInterface.apply_control ctrl: " + str(ctrl))
            self._vehicle.apply_control(ctrl)
        else:
            if DEBUG_MSGS:
                rospy.loginfo("ControllerInterface.apply_control self._control: " + str(self._control))
            # got stuck at the following line once
            self._vehicle.apply_control(self._control)   
        if DEBUG_MSGS:
            rospy.loginfo("ControllerInterface.apply_control done")
        
    def set_control(self, throttle, steering):
        if DEBUG_MSGS:
            rospy.loginfo("ControllerInterface.set_control")
        self._control.steer = min(max(-steering, -1.0), 1.0)
        if throttle < 0:
            self._control.throttle = 0
            self._control.brake = min(-throttle, 1.0)
        else:
            self._control.throttle = min(throttle, 1.0)
            self._control.brake = 0
        if DEBUG_MSGS:
            rospy.loginfo("ControllerInterface.set_control done")

    def get_control(self):
        return self._control