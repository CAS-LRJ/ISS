import numpy as np
cimport numpy as np

class RadarOutput(object):

    def __init__(self, points=None):
        self.points = points
        