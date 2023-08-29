import numpy as np
cimport numpy as np

# for single image/pcd file
class ObjectDetectionOutput(object):

    def __init__(self, gt=None):
        self._label = None
        self._score = None
        self._bbox = None
        self._mask = None
        self._keypoints = None
        self._keypoints_score = None

    def debug_print(self):
        print("Label: {}".format(self._label))
        print("Score: {}".format(self._score))
        print("Bbox: {}".format(self._bbox))

    def visualize(self):
        pass