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