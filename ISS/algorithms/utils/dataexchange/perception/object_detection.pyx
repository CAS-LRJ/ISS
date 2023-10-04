import numpy as np
cimport numpy as np

# for single image/pcd file
class ObjectDetectionOutput(object):

    def __init__(self):
        self._label = None
        self._score = None
        self._bbox = None
        self._mask = None
        self._keypoints = None
        self._keypoints_score = None
        self._trans = None
        self._loc = None
        self._vel = None

    def debug_print(self):
        print("Label: {}".format(self._label))
        print("Score: {}".format(self._score))
        print("Bbox: {}".format(self._bbox))

    def visualize(self):
        pass
    
    def get_state_bicycle_model(self):
        x = self._loc.x
        y = self._loc.y
        yaw = np.deg2rad(self._trans.rotation.yaw)
        vel = np.sqrt(self._vel.x**2 + self._vel.y**2)
        return np.array([x, y, yaw, vel])
    
    def get_veh_info(self):
        veh_info = {}
        veh_info['length'] = self._bbox.extent.x * 2
        veh_info['width'] = self._bbox.extent.y * 2
        veh_info['wheelbase'] = 2.8
        veh_info['overhang_rear'] = 0.978
        veh_info['overhang_front'] = 0.874 # Tesla Model 3
        return veh_info

        