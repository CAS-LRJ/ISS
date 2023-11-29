import numpy as np
import math
from motion_predictor.collision_check import find_vertices, check_collision_polygons
from iss_msgs.msg import ObjectDetection3DArray, ObjectDetection3D

def kinematic_bicycle_model(bicycle_model_state, acc, steer, L):
    d_bicycle_model_state = np.zeros(4)
    d_bicycle_model_state[0] = bicycle_model_state[3] * math.cos(bicycle_model_state[2])
    d_bicycle_model_state[1] = bicycle_model_state[3] * math.sin(bicycle_model_state[2])
    d_bicycle_model_state[2] = bicycle_model_state[3] * math.tan(steer) / L
    d_bicycle_model_state[3] = acc
    return d_bicycle_model_state

def bicycle_model_step(bicycle_model_state, acc, steer, L, dt):
    # Runge-Kutta 4th Order
    k1 = kinematic_bicycle_model(bicycle_model_state, acc, steer, L)
    k2 = kinematic_bicycle_model(bicycle_model_state + k1 * 0.5 * dt, acc, steer, L)
    k3 = kinematic_bicycle_model(bicycle_model_state + k2 * 0.5 * dt, acc, steer, L)
    k4 = kinematic_bicycle_model(bicycle_model_state + k3 * dt, acc, steer, L)
    bicycle_model_state = bicycle_model_state + (k1 + 2. * k2 + 2. * k3 + k4) * dt / 6.
    return bicycle_model_state

class ConstVelPredictor:
    def __init__(self, predictor_settings) -> None:
        self._dt = predictor_settings['dt']
        self._horizon = predictor_settings['MAX_T']
        self._ego_veh_info = predictor_settings['ego_veh_info']
        self._obstacle_detections = None

    def update(self, obstacle_detections):
        self._obstacle_detections = obstacle_detections

    def collision_check(self, path):
        if self._obstacle_detections is None:
            return False
        # all_pred_trajs = []
        # for obstacle in self._obstacle_detections:
        #     L = obstacle.bbox.size.x
        #     bicycle_model_state = np.zeros(4)
        #     bicycle_model_state[0] = obstacle.state.x
        #     bicycle_model_state[1] = obstacle.state.y
        #     bicycle_model_state[2] = obstacle.state.heading_angle
        #     bicycle_model_state[3] = obstacle.state.velocity
        #     acc = 0.0
        #     steer = 0.0
        #     all_pred_trajs.append(self._predict(bicycle_model_state, acc, steer, L))
        for obstacle in self._obstacle_detections:
            for wpt in path:
                obs_length = obstacle.bbox.size.x
                obs_width = obstacle.bbox.size.y
                obs_heading = obstacle.state.heading_angle
                obs_center = np.array([obstacle.state.x, obstacle.state.y])
                ego_length = self._ego_veh_info['length']
                ego_width = self._ego_veh_info['width']
                ego_heading = self._ego_veh_info['heading_angle']
                ego_center = np.array([wpt[0], wpt.y[1]])
                obs_Poly = find_vertices(obs_center, obs_heading, obs_length, obs_width)
                ego_Poly = find_vertices(ego_center, ego_heading, ego_length, ego_width)
                if check_collision_polygons(obs_Poly, ego_Poly):
                    return True
            
        

    def _predict(self, bicycle_model_state, acc, steer, L):
        pred_traj = []
        for t in np.arange(0, self._horizon, self._dt):
            bicycle_model_state = bicycle_model_step(bicycle_model_state, acc, steer, L, self._dt)
            pred_traj.append(bicycle_model_state)
        return pred_traj