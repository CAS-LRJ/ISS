class VehicleControl(object):

    def __init__(self, steer = 0., throttle=0., brake=1., hand_brake=True, manual_gear_shift=False) -> None:
        self.steer = steer
        self.throttle = throttle
        self.brake = brake
        self.hand_brake = hand_brake
        self.manual_gear_shift = manual_gear_shift