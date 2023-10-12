import numpy as np
cimport numpy as np

class GNSSOutput(object):

    def __init__(self, altitude=None, latitude=None, longitude=None):
        self.altitude = altitude
        self.latitude = latitude
        self.longitude = longitude

    