import numpy as np
from math import cos, sin
from ISS.algorithms.utils.vehicleutils.vehicleutils import check_collision_polygons

class Trajectory(object):

    def __init__(self, waypoints, speed=None) -> None:
        # list of (x, y, yaw)
        self.waypoints = waypoints
        # list of speed
        self.speed = speed

    def downsample(self, precision=0.1):
        new_waypoints = []
        new_speed = []
        prev_point = None
        for ind in range(len(self.waypoints)):
            point = self.waypoints[ind]
            if self.speed != None:
                speed = self.speed[ind]
            if prev_point == None or np.linalg.norm([prev_point[0] - point[0], prev_point[1] - point[1]]) > precision:
                prev_point = point
                new_waypoints.append(point)
                if self.speed != None:
                    new_speed.append(speed)
        
        self.waypoints = new_waypoints
        if self.speed != None:
            self.speed = new_speed
    
    def cubic_resample(self, precision=0.1):
        pass

class AllPredictionOutput(object):
    
    def __init__(self):
        self.predicted_trajectories = {}
    
    def add_prediction(self, obs_id, prediction):
        self.predicted_trajectories[obs_id] = prediction
    
    def check_point(self, point, ego_veh_info):
        for obs_id in self.predicted_trajectories:
            if self.predicted_trajectories[obs_id].check_point(point, ego_veh_info) == False:
                return False
        return True
    
    def check_path(self, path, ego_veh_info):
        for obs_id in self.predicted_trajectories:
            if self.predicted_trajectories[obs_id].check_path(path, ego_veh_info) == False:
                return False
        return True

class TrajectoryPredictionOutput(object):

    def __init__(self, init_state, veh_info) -> None:
        self.states_in_world_frame = [init_state]
        self.states_in_ego_frame = None
        self.veh_info = veh_info
        
    def get_last_state(self):
        return self.states_in_world_frame[-1]
    
    def get_veh_info(self):
        return self.veh_info
    
    def add_state(self, state):
        self.states_in_world_frame.append(state)
    
    def transform_state_to_ego_frame(self, ego_state):
        x_ego, y_ego, yaw_ego = ego_state[:3]
        self.states_in_ego_frame = []
        for npc_state in self.states_in_world_frame:
            x_npc, y_npc, yaw_npc = npc_state[:3]
            relative_x = (x_npc - x_ego) * cos(yaw_ego) + (y_npc - y_ego) * sin(yaw_ego)
            relative_y = -(x_npc - x_ego) * sin(yaw_ego) + (y_npc - y_ego) * cos(yaw_ego)
            self.states_in_ego_frame.append(np.array([relative_x, relative_y, yaw_npc - yaw_ego]))   
    
    def check_point(self, point, ego_veh_info):
        # if self.states_in_ego_frame == None:
        #     raise Exception('Please transform states to ego frame first!')
        obs_vertices = self.get_vertices(self.states_in_world_frame[0], self.veh_info)
        ego_vertices = self.get_vertices(point, ego_veh_info)
        if check_collision_polygons(obs_vertices, ego_vertices):
            return False
        return True
    
    def check_path(self, path, ego_veh_info):
        # if self.states_in_ego_frame == None:
        #     raise Exception('Please transform states to ego frame first!')
        for i in range(min(len(path), len(self.states_in_world_frame))):
            obs_vertices = self.get_vertices(self.states_in_world_frame[i], self.veh_info)
            ego_state = path[i]
            ego_vertices = self.get_vertices(ego_state, ego_veh_info)
            if check_collision_polygons(obs_vertices, ego_vertices):
                return False
        return True
    
    def get_vertices(self, state_rear, veh_info):
        wheelbase = veh_info['wheelbase']
        overhang_rear = veh_info['overhang_rear']
        overhang_front = veh_info['overhang_front']
        length_difference = (wheelbase + overhang_rear + overhang_front) / 2 - overhang_rear
        center = state_rear[:2] + length_difference * np.array([cos(state_rear[2]), sin(state_rear[2])])
        heading = state_rear[2]
        vertices = self.find_vertices(center, heading, veh_info['length'], veh_info['width'])
        return vertices
    
    def find_vertices(self, center, heading, length, width):        
        half_veh_len = length / 2
        half_veh_wid = width / 2
        R = np.array([[cos(heading), -sin(heading)],
                        [sin(heading), cos(heading)]])
        return [center + R @ np.array([-half_veh_len, -half_veh_wid]),
                center + R @ np.array([+half_veh_len, -half_veh_wid]),
                center + R @ np.array([+half_veh_len, +half_veh_wid]),
                center + R @ np.array([-half_veh_len, +half_veh_wid])]
    