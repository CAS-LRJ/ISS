import numpy as np
cimport numpy as np
import carla
import json

cdef bint sig_interrupt = False

def signal_handler(signal, frame):
    global sig_interrupt
    sig_interrupt = True
