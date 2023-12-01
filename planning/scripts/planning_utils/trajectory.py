import numpy as np
from scipy.interpolate import interp1d
from iss_msgs.msg import StateArray, State


class Trajectory:
    def __init__(self, bbox_size=None) -> None:
        # np.array([[x, y, heading_angle, velocity, acceleration, jerk, steering_angle, steer_velocity]])
        self._states = None
        self._time_step = None

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

    def update_states(self, states_list, time_step):
        if len(states_list) != 0:
            self._states = np.array(states_list)
        self._time_step = time_step

    def get_waypoints(self):
        return self._states[:, :3].tolist()

    def get_states(self):
        return self._states.tolist()

    # interpolate the states to get the reference trajectory, the first state is the closest planned state to the current state
    def get_ref_trajectory(self, ego_state, N, dt, dim=4):
        if self._states is None:
            return None
        cur_state = np.array([ego_state.x, ego_state.y, ego_state.heading_angle, ego_state.velocity,
                             ego_state.acceleration, ego_state.jerk, ego_state.steering_angle, ego_state.steering_angle_velocity])

        distances = np.linalg.norm(self._states[:, :2] - cur_state[:2], axis=1)
        closest_index = np.argmin(distances)
        planned_time_points = np.arange(0, self._states.shape[0] * self._time_step, self._time_step)
        ref_trajectory = np.zeros((N, self._states.shape[1]))
        time_steps = np.arange(planned_time_points[closest_index], planned_time_points[closest_index] + N*dt, dt)
        for i in range(dim):
            # Create an interpolator for each state element
            state_interpolator = interp1d(planned_time_points[closest_index:], self._states[closest_index:, i], fill_value="extrapolate", bounds_error=False)
            # Interpolate and fill the ref_trajectory
            ref_trajectory[:, i] = state_interpolator(time_steps[:ref_trajectory.shape[0]])
        # print(len(self._states))
        # print(self._states[:3, :dim])
        # print(self._states[-3:, :dim])
        # print(ref_trajectory[:3, :dim])
        # print(ref_trajectory[-3:, :dim])
        # print("---")
        return ref_trajectory[:, :dim].T
            
    def get_time_step(self):
        return self._time_step

    def is_empty(self):
        return self._states is None

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
        if self._time_step == None:
            self._time_step = 0
        trajectory_msg.duration = self._time_step * self._states.shape[0]
        return trajectory_msg

    def from_ros_msg(self, trajectory_msg):
        self._states = np.zeros([len(trajectory_msg.states), 8])
        for i in range(len(trajectory_msg.states)):
            self._states[i][0] = trajectory_msg.states[i].x
            self._states[i][1] = trajectory_msg.states[i].y
            self._states[i][2] = trajectory_msg.states[i].heading_angle
            self._states[i][3] = trajectory_msg.states[i].velocity
            self._states[i][4] = trajectory_msg.states[i].acceleration
            self._states[i][5] = trajectory_msg.states[i].jerk
            self._states[i][6] = trajectory_msg.states[i].steering_angle
            self._states[i][7] = trajectory_msg.states[i].steering_angle_velocity
        self._time_step = trajectory_msg.duration / len(trajectory_msg.states)


if __name__ == "__main__":
    a = np.array([[1, 2, 3], [4, 5, 6]])
    print(a.tolist())
