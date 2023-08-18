import numpy as np
import torch

from ISS.algorithms.utils.dataexchange.sensor.camera import CameraOutput

class Detection3DBase(object):

    def __init__(self, model=None):
        self.model = model

    def _preprocess(self, detection_3d_input):
        pass

    def detect(self, detection_3d_input):
        pass