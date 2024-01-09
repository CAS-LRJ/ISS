## This file creates state object corresponding to ros message ros1_ws/iss_manager/msg/State.msg
class State(object):
    ## To-DO Docs Here!
    available_keys = ['name', 'x', 'y', 'heading_angle', 'velocity', 'acceleration', 
                      'jerk', 'steering_angle', 'steering_angle_velocity', 'time_from_start']

    def __init__(self) -> None:
        self.name = None
        self.x = 0.0
        self.y = 0.0
        self.heading_angle = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.jerk = 0.0
        self.steering_angle = 0.0
        self.steering_angle_velocity = 0.0
        self.time_from_start = 0.0

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            if key in State.available_keys:
                setattr(self, key, value)