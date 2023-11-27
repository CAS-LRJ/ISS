import numpy as np

class Trajectory:
    def __init__(self, waypoints) -> None:
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