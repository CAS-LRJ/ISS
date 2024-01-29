import numpy as np
import math
from scipy.spatial import KDTree
from ISS.algorithms.utils.cubic_spline import Spline2D
import time
import pickle

def get_circle_centers(x, y, heading_angle, length, width, num_circles=3):
    spacing = length / num_circles
    centers = [(x + spacing * (j - (num_circles + 1) / 2) * math.cos(heading_angle),
                y + spacing * (j - (num_circles + 1) / 2) * math.sin(heading_angle))
               for j in range(1, num_circles + 1)]
    r = 0.5 * math.sqrt((length / num_circles) ** 2 + width ** 2)
    return centers, r
    
class ConstVelPredictor:
    def __init__(self, predictor_settings, lanemap_collision_checker, vehicle_info) -> None:
        self._predictor_settings = predictor_settings
        self._ego_veh_info = vehicle_info
        self._lanemap_collision_checker = lanemap_collision_checker
        self._obstacle_detections = None        
        self._loaded_trajecotries = None        
        self._spatial_temporal_obstacles = None
        self._obstacles_info_list = None

    def update_obstacle(self, obstacle_detections):
        self._obstacle_detections = obstacle_detections
    
    def read_prediction(self, trajectories):
        self._loaded_trajecotries = trajectories
    
    def save_obstacle(self):
        with open("/home/shaohang/work_space/autonomous_vehicle/ISS/obstacle.pkl", "wb") as f:
            pickle.dump(self._spatial_temporal_obstacles.data, f)
        with open("/home/shaohang/work_space/autonomous_vehicle/ISS/obstacle_info.pkl", "wb") as f:
            pickle.dump(self._obstacles_info_list, f)
    
    def update_prediction(self, dt, horizon):
        spatial_temporal_obstacles_list = []
        self._obstacles_info_list = []
        if self._predictor_settings["method"] == "constant_velocity_predictor":
            if self._obstacle_detections is None:
                return
            obstacle_infos = [[obstacle.state.x, obstacle.state.y, obstacle.state.heading_angle, obstacle.bbox.size.x, obstacle.bbox.size.y, obstacle.state.velocity]\
                                for obstacle in self._obstacle_detections.detections]
            for idx in range(horizon):
                for _, obstacle in enumerate(obstacle_infos):
                    obstacle_centers, r = get_circle_centers(obstacle[0], obstacle[1], obstacle[2], obstacle[3], obstacle[4])
                    for obstacle_center in obstacle_centers:
                        spatial_temporal_obstacles_list.append(obstacle_center)
                        self._obstacles_info_list.append((idx, r))
                    obstacle[0] += obstacle[5] * math.cos(obstacle[2]) * dt
                    obstacle[1] += obstacle[5] * math.sin(obstacle[2]) * dt
        elif self._predictor_settings["method"] == "loaded_waypoint_predictor":
            if self._loaded_trajecotries is None:
                return
            for traj in self._loaded_trajecotries:
                state_array = traj.get_states_array()
                x = state_array[:, 0].tolist()
                y = state_array[:, 1].tolist()
                csp = Spline2D(x, y)
                s = np.arange(0, csp.s[-1], (csp.s[-1] / horizon)) 
                for idx, ss in enumerate(s):
                    ox, oy = csp.calc_position(ss)
                    oyaw = csp.calc_yaw(ss)
                    obstacle_centers, r = get_circle_centers(ox, oy, oyaw, self._ego_veh_info['length'], self._ego_veh_info['width'])
                    for obstacle_center in obstacle_centers:
                        spatial_temporal_obstacles_list.append(obstacle_center)
                        self._obstacles_info_list.append((idx, r))
        if len(spatial_temporal_obstacles_list) > 0:
            self._spatial_temporal_obstacles = KDTree(np.array(spatial_temporal_obstacles_list), copy_data=True)
        else:
            self._spatial_temporal_obstacles = None
    
    def collision_check(self, path, ego_length=None, ego_width=None):
        if self._lanemap_collision_checker.check_path(path): #TODO: not accurate
            return True, 0
        if self._spatial_temporal_obstacles is None:
            return False, 0
        if ego_length is None:
            ego_length = self._ego_veh_info['length']
        if ego_width is None:
            ego_width = self._ego_veh_info['width']
        for i, wpt in enumerate(path):
            ego_heading = wpt[2]
            ego_center = [wpt[0], wpt[1]]
            ego_circle_centers, ego_radius = get_circle_centers(ego_center[0], ego_center[1], ego_heading, ego_length, ego_width)
            for ego_circle_center in ego_circle_centers:
                ego_circle_center_array = np.array(ego_circle_center)
                possible_obstacles = self._spatial_temporal_obstacles.query_ball_point(ego_circle_center_array, 6 * ego_radius)
                for idx in possible_obstacles:
                    obs_info = self._obstacles_info_list[idx]
                    if obs_info[0] in [(i + j) for j in range(-1, 2)]:                    
                        dist = np.linalg.norm(ego_circle_center_array - self._spatial_temporal_obstacles.data[idx])
                        if dist < (ego_radius + obs_info[1]):
                            return True, 1
        return False, 0
    
    def get_front_obstacle(self, csp, s_ego, LOOK_AHEAD_DISTANCE):
        s_obstacle = s_ego
        obstacle_detections_kdtree = KDTree(np.array([[obstacle.state.x, obstacle.state.y] for obstacle in self._obstacle_detections.detections]), copy_data=True)
        while s_obstacle < (s_ego + LOOK_AHEAD_DISTANCE):
            s_obstacle += self._ego_veh_info["length"] / 2
            x, y = csp.calc_position(s_obstacle)
            d, idx = obstacle_detections_kdtree.query([x, y])
            if d < self._ego_veh_info["width"] / 2:
                return s_obstacle
        return None
    
    def check_emergency_stop(self, x, y, heading_angle):
        LEN_INF_FACTOR = 1.1
        WID_INF_FACTOR = 1.05
        res, _ = self.collision_check([[x, y, heading_angle]], LEN_INF_FACTOR * self._ego_veh_info['length'], WID_INF_FACTOR * self._ego_veh_info['width'])
        return res
        