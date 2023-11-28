import numpy as np
from iss_msgs.msg import StateArray, State

class Trajectory:
    def __init__(self, bbox_size=None) -> None:
        # list of [x, y, heading_angle, velocity, acceleration, jerk, steering_angle, steer_velocity]
        self._states = None

    def update_waypoints(self, waypoints, downsample_precision=0.1):
        # waypoints: list of (x, y, yaw)
        new_waypoints = []
        prev_point = None
        for ind in range(len(waypoints)):
            point = waypoints[ind]
            if prev_point == None or np.linalg.norm([prev_point[0] - point[0], prev_point[1] - point[1]]) > downsample_precision:
                prev_point = point
                new_waypoints.append(point)
        self._states = np.zeros([len(new_waypoints), 8])
        for i in range(len(new_waypoints)):
            self._states[i][0] = new_waypoints[i][0]
            self._states[i][1] = new_waypoints[i][1]
            self._states[i][2] = new_waypoints[i][2]

    def get_waypoints(self):
        return self._states[:, :3]
    
    def collision_check_state(self, state, target_bbox_size):
        return
    
    def collision_check_trajectory(self, trajectory, target_bbox_size):
        return
    
    def to_ros_msg(self):
        trajectory_msg = StateArray()
        for i in range(self._states.shape[0]):
            state_msg = State()
            state_msg.x = self._states[i][0]
            state_msg.y = self._states[i][1]
            state_msg.heading_angle = self._states[i][2]
            state_msg.velocity = self._states[i][3]
            state_msg.acceleration = self._states[i][4]
            state_msg.jerk = self._states[i][5]
            state_msg.steering_angle = self._states[i][6]
            state_msg.steering_angle_velocity = self._states[i][7]
            trajectory_msg.states.append(state_msg)
        return trajectory_msg
    