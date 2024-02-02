import numpy as np
from ISS.algorithms.state_estimation.ekf.rotations import Quaternion

class EKF:
    
    EARTH_RADIUS = 6378135 # Aequatorradii
    G = 9.81
    
    def __init__(self, settings) -> None:
        self._var_imu_f = settings['var_imu_f']
        self._var_imu_w = settings['var_imu_w']
        self._var_gnss = settings['var_gnss']
        self._dt = 1 / settings['frequency']
        self._p_est = None
        self._v_est = None
        self._q_est = None
        self._cov = None
    
    def init(self, x, y, z, vx, vy, vz, roll, pitch, yaw):
        self._p_est = np.array([x, y, z])
        self._v_est = np.array([vx, vy, vz])
        self._q_est = Quaternion(euler=[roll, pitch, yaw]).to_numpy()
        self._cov = np.zeros(9)
        
    def update(self):
        pass
    
    def step(self, imu_f, imu_w, gnss):
        pass

    def geo_to_xyz(self, lat, lon, alt):
        # https://github.com/carla-simulator/carla/issues/3871
        lat_rad = (np.deg2rad(lat) + np.pi) % (2 * np.pi) - np.pi
        lon_rad = (np.deg2rad(lon) + np.pi) % (2 * np.pi) - np.pi
        x = EKF.EARTH_RADIUS * np.sin(lon_rad) * np.cos(lat_rad) 
        y = EKF.EARTH_RADIUS * np.sin(-lat_rad)
        z = alt
        return x, y, z