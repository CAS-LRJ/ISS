import numpy as np
from typing import List, Tuple
from scipy.interpolate import interp1d


class Trajectory:
    def __init__(self, bbox_size=None) -> None:
        # np.array([[x, y, heading_angle, velocity, steering_angle, acceleration,
        #            jerk, steering_angle_velocity, time_from_start]])
        self._states = None

    def update_waypoints(self, waypoints: List[Tuple[float, float, float]], downsample_precision: float = 0.1) -> None:
        # waypoints: list of (x, y, yaw)
        new_waypoints = []
        prev_point = None
        for point in waypoints:
            if prev_point is None or np.linalg.norm(np.array(prev_point[:2]) - np.array(point[:2])) > downsample_precision:
                prev_point = point
                new_waypoints.append(point)
        self._states = np.array(new_waypoints)

    def get_waypoints(self):
        return self._states[:, :3].tolist()

    def update_states_from_array(self, states_array):
        if states_array is None:
            return
        self._states = np.zeros((states_array.shape[0], 9))
        self._states[:, :states_array.shape[1]] = states_array

    def update_states_from_list(self, states_list):
        if len(states_list) != 0:
            self._states = np.array(states_list)

    def get_states_list(self):
        return self._states.tolist()

    def get_states_array(self):
        return self._states.copy()

    # interpolate the states to get the reference trajectory, the first state is the closest planned state to the current state
    def get_ref_trajectory(self, ego_state, N, dt, dim=4):
        if self._states is None:
            return None
        cur_state = np.array([ego_state.x, ego_state.y, ego_state.heading_angle, ego_state.velocity, ego_state.steering_angle, 
                             ego_state.acceleration, ego_state.jerk, ego_state.steering_angle_velocity])

        distances = np.linalg.norm(self._states[:, :2] - cur_state[:2], axis=1)
        closest_index = np.argmin(distances)
        planned_time_points = np.arange(
            0, self._states.shape[0] * self._time_step, self._time_step)
        ref_trajectory = np.zeros((N, self._states.shape[1]))
        time_steps = np.arange(
            planned_time_points[closest_index], planned_time_points[closest_index] + N*dt, dt)
        try:
            for i in range(dim):
                # Create an interpolator for each state element
                state_interpolator = interp1d(
                    planned_time_points[closest_index:], self._states[closest_index:, i], fill_value="extrapolate", bounds_error=False)
                # Interpolate and fill the ref_trajectory
                ref_trajectory[:, i] = state_interpolator(
                    time_steps[:ref_trajectory.shape[0]])
        except:
            print("------------------")
            print(planned_time_points[closest_index:].shape)
            print(self._states[closest_index:, i].shape)
            print(planned_time_points)
            print(self._states)
            raise ValueError("Error in trajectory interpolation")
        return ref_trajectory[:, :dim].T

    def get_time_step(self):
        return self._time_step

    def is_empty(self):
        return self._states is None

    def collision_check_state(self, state, target_bbox_size):
        return

    def collision_check_trajectory(self, trajectory, target_bbox_size):
        return
