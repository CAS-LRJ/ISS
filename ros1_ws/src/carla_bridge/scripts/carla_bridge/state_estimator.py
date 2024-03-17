import rospy
from iss_manager.msg import State
import numpy as np

from ISS.algorithms.state_estimation.ekf.ekf import EKF
from ISS.algorithms.utils.angle import pi_2_pi

DEBUG = False

class GTStateEstimator:
    def __init__(self, vehicle) -> None:
        self._vehicle = vehicle
        self._state_estimation_pub = rospy.Publisher(rospy.get_param("ego_state_topic"), State, queue_size=1)
        # gt_state_estimation_frequency = rospy.get_param('gt_state_estimation_frequency', 10)
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
        # print("real angle: ", pi_2_pi(-np.deg2rad(vehicle_rotation.yaw)))
        # print("-------------------")
        self._state_estimation_pub.publish(state)
    
    def run_step(self, input_data, steer):
        self.publish_ego_state(None)
    
class EKFStateEstimator:
    def __init__(self) -> None:
        self._state_estimation_pub = rospy.Publisher(rospy.get_param("ego_state_topic"), State, queue_size=1)
        ekf_setting = {
                        "vehicle_info": {
                            "wheelbase": 2.8498,
                        },
                        "frequency": 20
                    }
        self._ekf = EKF(ekf_setting)
        if DEBUG:
            self.iter = 0
    
    def initialize(self, lat, lon, compass, speed, acc_x, x_std, y_std, compass_std, speed_std, acc_x_std):
        obs_x, obs_y = self._ekf.geo_to_xy(lat, lon)
        obs_y *= -1
        self._ekf.initialize(obs_x, obs_y, pi_2_pi(compass), speed, acc_x, x_std, y_std, compass_std, speed_std, acc_x_std)
        
    def run_step(self, input_data, steer):
        _, gps   = input_data.get('GPS')
        _, imu   = input_data.get('IMU')
        _, ego   = input_data.get('EGO')
        spd      = ego.get('speed')
        
        obs_x, obs_y = self._ekf.geo_to_xy(gps[0], gps[1])
        obs_y *= -1
        obs_yaw = pi_2_pi(np.pi / 2 - imu[-1])
        obs_acc = imu[0]
        
        if self._ekf.is_initialized() is False:
            self._ekf.initialize(obs_x, obs_y, obs_yaw, spd, obs_acc, 1e-1, 1e-1, 1e-3, 1e-5, 1e-5)
        else:
            self._ekf.step(obs_acc, obs_yaw, obs_x, obs_y, spd, steer)
        
        state_list = self._ekf.get_state()
        state = State()
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = rospy.get_param("world_frame")
        state.name = "ego_vehicle"
        state.x = state_list[0]
        state.y = state_list[1]
        state.heading_angle = state_list[2]
        state.velocity = state_list[3]
        state.acceleration = state_list[4]
        if DEBUG:
            self.iter += 1
            rospy.loginfo("EKFStateEstimator.run_step self.iter: " + str(self.iter) + " gps: " + str(gps) + " state.x: " + str(state.x) + " state.y: " + str(state.y) + " state.heading_angle: " + str(state.heading_angle))
        self._state_estimation_pub.publish(state)

    def get_state(self):
        return self._ekf.get_state()