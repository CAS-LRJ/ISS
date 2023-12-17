from iss_msgs.msg import StateArray, State

def to_ros_msg(self):
    trajectory_msg = StateArray()
    for i in range(_states.shape[0]):
        state_msg = State()
        state_msg.x = _states[i][0]
        state_msg.y = _states[i][1]
        state_msg.heading_angle = _states[i][2]
        state_msg.velocity = _states[i][3]
        state_msg.acceleration = _states[i][4]
        state_msg.jerk = _states[i][5]
        state_msg.steering_angle = _states[i][6]
        state_msg.steering_angle_velocity = _states[i][7]
        trajectory_msg.states.append(state_msg)
    if _time_step == None:
        _time_step = 0
    trajectory_msg.duration = _time_step * _states.shape[0]
    return trajectory_msg

def from_ros_msg(self, trajectory_msg):
    _states = np.zeros([len(trajectory_msg.states), 8])
    for i in range(len(trajectory_msg.states)):
        _states[i][0] = trajectory_msg.states[i].x
        _states[i][1] = trajectory_msg.states[i].y
        _states[i][2] = trajectory_msg.states[i].heading_angle
        _states[i][3] = trajectory_msg.states[i].velocity
        _states[i][4] = trajectory_msg.states[i].acceleration
        _states[i][5] = trajectory_msg.states[i].jerk
        _states[i][6] = trajectory_msg.states[i].steering_angle
        _states[i][7] = trajectory_msg.states[i].steering_angle_velocity
    _time_step = trajectory_msg.duration / len(trajectory_msg.states)