import numpy as np
from enum import Enum
from libc.stdlib cimport *

# define sensor source type
class SensorSource(Enum):
    DEFAULT = 0
    CARLA = 1
    BEAMNG = 2
    DATASET = 3
    REALVEHICLE = 4
    OTHERSOURCE = 5

# define sensor type
class SensorType(Enum):
    DEFAULT = 0
    CAMERA = 1
    RGBCAMERA = 2
    DEPTHCAMERA = 3
    SEMANTICCAMERA = 4
    LIDAR = 5
    SEMANTICLIDAR = 6
    RADAR = 7
    IMU = 8
    GNSS = 9
    VEHICLE = 10
    INFRASTRUCTURE = 11
    OTHERTYPE = 12

class Sensor:
        
    def __init__(self, frame, timestamp, ssource, stype):
        self.frame = frame
        self.timestamp = timestamp
        self.ssource = ssource
        self.stype = stype

    def set_frame(self, frame):
        self.frame = frame
    def set_timestamp(self, timestamp):
        self.timestamp = timestamp
    def set_ssource(self, ssource):
        self.ssource = ssource
    def set_stype(self, stype):
        self.stype = stype

    def get_frame(self):
        return self.frame
    def get_timestamp(self):
        return self.timestamp
    def get_ssource(self):
        if self.ssource == SensorSource.DEFAULT:
            return "DEFAULT"
        elif self.ssource == SensorSource.CARLA:
            return "CARLA"
        elif self.ssource == SensorSource.BEAMNG:
            return "BEAMNG"
        elif self.ssource == SensorSource.DATASET:
            return "DATASET"
        elif self.ssource == SensorSource.REALVEHICLE:
            return "REALVEHICLE"
        elif self.ssource == SensorSource.OTHERSOURCE:
            return "OTHERSOURCE"
        else:
            return "UNKNOWN"
        # return self.ssource
    def get_stype(self):
        if self.sstype == SensorType.DEFAULT:
            return "DEFAULT"
        elif self.sstype == SensorType.CAMERA:
            return "CAMERA"
        elif self.sstype == SensorType.RGBCAMERA:
            return "RGBCAMERA"
        elif self.sstype == SensorType.DEPTHCAMERA:
            return "DEPTHCAMERA"
        elif self.sstype == SensorType.SEMANTICCAMERA:
            return "SEMANTICCAMERA"
        elif self.sstype == SensorType.LIDAR:
            return "LIDAR"
        elif self.sstype == SensorType.SEMANTICLIDAR:
            return "SEMANTICLIDAR"
        elif self.sstype == SensorType.RADAR:
            return "RADAR"
        elif self.sstype == SensorType.IMU:
            return "IMU"
        elif self.sstype == SensorType.VEHICLE:
            return "VEHICLE"
        elif self.sstype == SensorType.INFRASTRUCTURE:
            return "INFRASTRUCTURE"
        elif self.sstype == SensorType.OTHERTYPE:
            return "OTHERTYPE"
        else:
            return "UNKNOWN"
        # return self.stype