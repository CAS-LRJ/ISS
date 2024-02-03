import numpy as np

class EKF:
    EARTH_RADIUS = 6378135 # Equator radius

    def __init__(self, settings) -> None:
        self._dt = 1 / settings['frequency']
        self._vehicle_info = settings['vehicle_info']
        self._state = np.zeros(5)
        self._P = np.eye(5) * 1e-5  # Adjusted initialization
        self._R = np.eye(5) * 1e-5  # Measurement noise covariance
        self._Q = np.diag([1e-4, 1e-4, 1e-6, 1e-3, 1e-2])  # Process noise covariance
        self._H = np.eye(5)  # Measurement matrix
        self._F = None

    def init(self, lat, long, compass, speed, acc_x, lat_std, long_std, compass_std, speed_std, acc_x_std):
        x, y = self.geo_to_xy(lat, long)
        self._state = np.array([x, y, compass, speed, acc_x])
        self._R = np.diag([lat_std**2, long_std**2, compass_std**2, speed_std**2, acc_x_std**2])

    def step(self, acc_x, compass, lat, lon, speed, steer):
        # Update the state with the bicycle model
        self.bicycle_model_step(acc_x, steer)
        # Calculate the Jacobian of the motion model
        self._F = self.calculate_jacobian(self._state, steer, acc_x)
        # Predict the error covariance
        self._P = self._F @ self._P @ self._F.T + self._Q
        # Calculate the Kalman Gain
        K = self._P @ self._H.T @ np.linalg.inv(self._H @ self._P @ self._H.T + self._R)
        # Update the state with the new measurements
        obs_x, obs_y = self.geo_to_xy(lat, lon)
        z = np.array([obs_x, obs_y, compass, speed, acc_x])
        self._state = self._state + K @ (z - self._H @ self._state)
        # Update the error covariance
        self._P = (np.eye(5) - K @ self._H) @ self._P
        return self._state

    def calculate_jacobian(self, state, steer, acc_x):
        x, y, theta, v, _ = state
        dt = self._dt
        L = self._vehicle_info['wheelbase']
        F = np.array([
            [1, 0, -v*np.sin(theta)*dt, np.cos(theta)*dt, 0],
            [0, 1, v*np.cos(theta)*dt, np.sin(theta)*dt, 0],
            [0, 0, 1, np.tan(steer)/L*dt, 0],
            [0, 0, 0, 1, dt],
            [0, 0, 0, 0, 1]
        ])
        return F

    def bicycle_model_step(self, acc_x, steer):
        x, y, theta, v, _ = self._state
        dt = self._dt
        L = self._vehicle_info['wheelbase']
        self._state[0] += v * np.cos(theta) * dt
        self._state[1] += v * np.sin(theta) * dt
        self._state[2] += v * np.tan(steer) / L * dt
        self._state[3] += acc_x * dt
        # Assuming constant acceleration within this step
        self._state[4] = acc_x
        
    def geo_to_xy(self, lat, lon):
        # https://github.com/carla-simulator/carla/issues/3871
        lat_rad = (np.deg2rad(lat) + np.pi) % (2 * np.pi) - np.pi
        lon_rad = (np.deg2rad(lon) + np.pi) % (2 * np.pi) - np.pi
        x = EKF.EARTH_RADIUS * np.sin(lon_rad) * np.cos(lat_rad) 
        y = EKF.EARTH_RADIUS * np.sin(-lat_rad)
        return x, y