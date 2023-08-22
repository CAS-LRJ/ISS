from ISS.algorithms.perception.detection_3d.base import Detection3DBase

import carla
import os
import numpy as np

class Detection3Dgt(Detection3DBase):
    def _preprocess(self, detection_3d_input):
        self.input = detection_3d_input

    def detect(self, detection_3d_input):
        