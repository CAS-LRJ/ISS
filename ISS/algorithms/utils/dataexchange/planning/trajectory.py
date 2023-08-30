import numpy as np

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