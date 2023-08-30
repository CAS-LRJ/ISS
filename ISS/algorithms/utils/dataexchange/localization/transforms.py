class VehicleTransform(object):

    def __init__(self, location = (None, None, None), velocity = (None, None, None, None), acceleration = (None, None, None, None), rotatation = (None, None, None)) -> None:
        self.x, self.y, self.z = location
        self.velocity_x, self.velocity_y, self.velocity_z, self.velocity = velocity
        self.acceleration_x, self.acceleration_y, self.acceleration_z, self.acceleration = acceleration
        self.yaw, self.pitch, self.roll = rotatation