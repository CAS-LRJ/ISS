import rospy
from iss_manager.msg import State
import numpy as np

from ISS.algorithms.utils.angle import pi_2_pi


class GTStateEstimator:
    def __init__(self, vehicle) -> None:
        self._vehicle = vehicle
        self._state_estimation_pub = rospy.Publisher(rospy.get_param("ego_state_topic"), State, queue_size=1)
        gt_state_estimation_frequency = rospy.get_param('gt_state_estimation_frequency', 10)
        # self._timer = rospy.Timer(rospy.Duration(1 / gt_state_estimation_frequency), self.publish_ego_state)
    
    def publish_ego_state(self, event):
        state = State()
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = rospy.get_param("world_frame")
        state.name = "ego_vehicle"
        carla_transform = self._vehicle.get_transform()
        vehicle_location = carla_transform.location
        vehicle_rotation = carla_transform.rotation
        vehicle_velocity = self._vehicle.get_velocity()
        vehicle_acceleration = self._vehicle.get_acceleration()
        state.x = vehicle_location.x
        state.y = -vehicle_location.y
        state.heading_angle = pi_2_pi(-np.deg2rad(vehicle_rotation.yaw))
        state.velocity = np.hypot(vehicle_velocity.x, vehicle_velocity.y)
        state.acceleration = np.hypot(vehicle_acceleration.x, vehicle_acceleration.y)
        # print("real state: ", state)
        self._state_estimation_pub.publish(state)
    
    def shutdown(self):
        self._timer.shutdown()
        self._state_estimation_pub.unregister()