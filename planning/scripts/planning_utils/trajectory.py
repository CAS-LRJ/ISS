import numpy as np

class Trajectory:
    def __init__(self, bbox_size=None) -> None:
        self.waypoints = None # list of [x, y, heading_angle]
        self.states = None # list of [x, y, heading_angle, velocity]

    def update_waypoints(self, waypoints):
        # list of (x, y, yaw)
        self.waypoints = waypoints

    def downsample(self, precision=0.1):
        new_waypoints = []
        prev_point = None
        for ind in range(len(self.waypoints)):
            point = self.waypoints[ind]
            if prev_point == None or np.linalg.norm([prev_point[0] - point[0], prev_point[1] - point[1]]) > precision:
                prev_point = point
                new_waypoints.append(point)
        self.waypoints = new_waypoints

    def get_waypoints(self):
        return self.waypoints
    
    def collision_check_state(self, state, target_bbox_size):
        return
    
    def collision_check_trajectory(self, trajectory, target_bbox_size):
        return
    
        