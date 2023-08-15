import numpy as np
from libc.stdlib cimport *

cdef class Sensor(Object):
    
    cdef:
        int frame
        double timestamp
    def __init__(self):
        self.frame = 