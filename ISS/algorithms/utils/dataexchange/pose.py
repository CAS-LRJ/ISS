from ISS.algorithms.utils.dataexchange import Point, Quaternion

class Pose(object):
    available_keys = ['center', 'quaternion']

    def __init__(self) -> None:
        self.center = Point()
        self.quaternion = Quaternion()
    
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            if key in Pose.available_keys:
                setattr(self, key, value)