import numpy as np
import math
from ISS.algorithms.planning.motion_predictor.collision_check import find_vertices, check_collision_polygons
from scipy.spatial import KDTree

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

def get_circle_centers(x, y, heading_angle, length, width, num_circles=3):
    # represent the vehicle circles, the state is the center of the bicycle model
    centers = []
    for j in range(num_circles):
        center = [0, 0]
        center[0] = x + length / (2 * num_circles) * (2 * j - num_circles - 1) * math.cos(heading_angle)
        center[1] = y + length / (2 * num_circles) * (2 * j - num_circles - 1) * math.sin(heading_angle)
        centers.append(center)
    r = 0.5 * math.sqrt((length / num_circles) ** 2 + (width / 2) ** 2)
    return centers, r
    
class ConstVelPredictor:
    def __init__(self, predictor_settings) -> None:
        self._dt = predictor_settings['dt']
        self._horizon = predictor_settings['MAX_T']
        self._ego_veh_info = predictor_settings['ego_veh_info']
        self._obstacles = None
        self._map = None
    
    def update_map(self, lane_map):
        self._map = lane_map

    def update_obstacle(self, obstacle_detections):
        obstacle_list = []
        for obstacle in obstacle_detections.detections:
            obstacle_centers, r = get_circle_centers(obstacle.state.x, obstacle.state.y, obstacle.state.heading_angle, obstacle.bbox.size.x, obstacle.bbox.size.y)
            obstacle_list.extend(obstacle_centers)
        if len(obstacle_list) > 0:
            self._obstacles = KDTree(np.array(obstacle_list))
            
    def collision_check(self, path):
        if self._obstacles == None:
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
        
        ego_length = self._ego_veh_info['length']
        ego_width = self._ego_veh_info['width']
        for wpt in path:
            ego_heading = wpt[2]
            ego_center = [wpt[0], wpt[1]]
            ego_circle_centers, ego_radius = get_circle_centers(ego_center[0], ego_center[1], ego_heading, ego_length, ego_width)
            for ego_circle_center in ego_circle_centers:
                ego_circle_center_array = np.array(ego_circle_center)
                self._map.check_collision(ego_circle_center_array, ego_radius)
                dist, ind = self._obstacles.query(ego_circle_center_array)
                if dist < 3 * ego_radius:
                    return True
        return False
            
        

    def _predict(self, bicycle_model_state, acc, steer, L):
        pred_traj = []
        for t in np.arange(0, self._horizon, self._dt):
            bicycle_model_state = bicycle_model_step(bicycle_model_state, acc, steer, L, self._dt)
            pred_traj.append(bicycle_model_state)
        return pred_traj
    
