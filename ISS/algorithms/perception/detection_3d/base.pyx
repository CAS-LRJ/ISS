import numpy as np
# import torch
from multiprocessing.connection import Listener, Client 

from ISS.algorithms.utils.dataexchange.sensor.camera import CameraOutput

class Detection3DBase(object):

    def __init__(self, model=None):
        self.model = model
        self.port = 6002
        self.to_port = 6003

    def _preprocess(self, detection_3d_input):
        pass

    def detect(self, detection_3d_input):
        pass