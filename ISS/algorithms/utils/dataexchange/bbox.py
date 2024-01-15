from ISS.algorithms.utils.dataexchange import Point
import numpy as np

## This file creates state object corresponding to ros message ros1_ws/iss_manager/msg/ObjectDetection3D.msg
class Bbox(object):
    ## To-DO Docs Here!
    available_keys = ['center', 'size']

    def __init__(self) -> None:
        self.center = Point()
        self.size = np.zeros(3)

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            if key in Bbox.available_keys:
                setattr(self, key, value)