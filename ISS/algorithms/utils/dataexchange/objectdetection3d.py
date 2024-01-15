from ISS.algorithms.utils.dataexchange import Bbox, State

## This file creates state object corresponding to ros message ros1_ws/iss_manager/msg/ObjectDetection3D.msg
class ObjectDetection3D(object):
    ## To-DO Docs Here!
    available_keys = ['id', 'score', 'state', 'bbox']

    def __init__(self) -> None:
        self.id = 0
        self.score = 0.0
        self.state = State()
        self.bbox = Bbox()

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            if key in ObjectDetection3D.available_keys:
                setattr(self, key, value)