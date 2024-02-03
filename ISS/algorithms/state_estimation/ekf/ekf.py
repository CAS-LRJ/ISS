import numpy as np
from ISS.algorithms.state_estimation.ekf.rotations import Quaternion

class EKF:
    
    EARTH_RADIUS = 6378135 # Aequatorradii
    G = 9.81
    
    def __init__(self, settings) -> None:
        self._dt = 1 / settings['frequency']
        self._vehicle_info = settings['vehicle_info']
        self._state = np.zeros(5)
        self._P = np.zeros((5, 5)) * 1e-7
        self._R = np.eye(5) * 1e-7
        self._Q = np.zeros((5, 5))
        self._H = np.eye(5)
    
    def init(self, lat, long, compass, speed, acc_x, lat_std, long_std, compass_std, speed_std, acc_x_std):
        x, y = self.geo_to_xy(lat, long)
        self._state[0] = x
        self._state[1] = y
        self._state[2] = compass
        self._state[3] = speed
        self._state[4] = acc_x
        self._R = np.diag([lat_std, long_std, compass_std, speed_std, acc_x_std])
    
    def step(self, acc_x, compass, lat, lon, speed, steer):
        self._F = np.eye(5) # TODO: Implement Jacobian
        obs_x, obs_y = self.geo_to_xy(lat, lon)
        self.bicycle_model_step(acc_x, steer)
        self._P = self._F @ self._P @ self._F.T + self._Q
        K = self._P @ self._H.T @ np.linalg.inv(self._H @ self._P @ self._H.T + self._R)
        self._state = self._state + K @ (np.array([obs_x, obs_y, compass, speed, acc_x]) - self._state)
        self._P = (np.eye(5) - K @ self._H) @ self._P
        return self._state
    
    def bicycle_model_step(self, acc_x, steer):
        self._state[0] += self._dt * self._state[3] * np.cos(self._state[2])
        self._state[1] += self._dt * self._state[3] * np.sin(self._state[2])
        self._state[2] += self._dt * self._state[3] * np.tan(self._state[2]) * np.cos(steer) / self._vehicle_info['wheelbase']
        self._state[3] += self._dt * acc_x
        self._state[4] = acc_x
        
    def geo_to_xy(self, lat, lon):
        # https://github.com/carla-simulator/carla/issues/3871
        lat_rad = (np.deg2rad(lat) + np.pi) % (2 * np.pi) - np.pi
        lon_rad = (np.deg2rad(lon) + np.pi) % (2 * np.pi) - np.pi
        x = EKF.EARTH_RADIUS * np.sin(lon_rad) * np.cos(lat_rad) 
        y = EKF.EARTH_RADIUS * np.sin(-lat_rad)
        return x, y