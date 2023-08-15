import numpy as np
from libc.stdlib cimport *

# define sensor source type
ctypedef enum sensor_source:
    CARLA,
    BEAMNG,
    DATASET,
    REALVEHICLE,
    OTHERSOURCE

# define sensor type
ctypedef enum sensor_type:
    CAMERA,
    LIDAR,
    RADAR,
    IMU,
    VEHICLE,
    INFRASTRUCTURE,
    OTHERTYPE

cdef class Sensor:
    
    cdef:
        int frame
        double timestamp
        sensor_source ssource
        sensor_type stype
        
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