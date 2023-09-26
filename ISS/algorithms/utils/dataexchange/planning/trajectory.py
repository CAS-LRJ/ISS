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
    
    def check_point(self, point, half=False):
        return False
    
    def check_path(self, path, ego_veh_info):
        for obs_id in self.predicted_trajectories:
            if self.predicted_trajectories[obs_id].check_path(path, ego_veh_info):
                return True
        return False

class TrajectoryPredictionOutput(object):

    def __init__(self, veh_info) -> None:
        self.states = []
        self.veh_info = veh_info
    
    def get_last_state(self):
        return self.states[-1]
    
    def get_veh_info(self):
        return self.veh_info
    
    def add_state(self, state):
        self.states.append(state)
    
    def check_point(self, point, half=False):
        return False
    
    def check_path(self, path, ego_veh_info):
        # check if the predicted trajectory collides with the path
        for i in range(min(len(path), len(self.states))):
            obs_state = self.states[i]
            obs_vertices = self.get_vertices(obs_state, self.veh_info)
            ego_state = path[i]
            ego_vertices = self.get_vertices(ego_state, ego_veh_info)
            if check_collision_polygons(obs_vertices, ego_vertices):
                return True
        return False
    
    def get_vertices(self, state_rear, veh_info):
        wheelbase = veh_info['wheelbase']
        overhang_rear = veh_info['overhang_rear']
        overhang_front = veh_info['overhang_front']
        length_difference = (wheelbase + overhang_rear + overhang_front) / 2 - overhang_rear
        center = state_rear[:2] + length_difference * np.array([cos(state_rear[2]), sin(state_rear[2])])
        heading = state_rear[2]
        vertices = self.find_vertices(center, heading, veh_info['length'], veh_info['width'])
        return vertices
    
    def find_vertices(center, heading, length, width):        
        half_veh_len = length / 2
        half_veh_wid = width / 2
        R = np.array([[cos(heading), -sin(heading)],
                        [sin(heading), cos(heading)]])
        return [center + R @ np.array([-half_veh_len, -half_veh_wid]),
                center + R @ np.array([+half_veh_len, -half_veh_wid]),
                center + R @ np.array([+half_veh_len, +half_veh_wid]),
                center + R @ np.array([-half_veh_len, +half_veh_wid])]
    
    