from ISS.algorithms.utils.mathutils.angle cimport calculate_rot_angle
from libc cimport math
import numpy as np
cimport numpy as np

def localization(veh, np.ndarray[np.float64_t, ndim = 1] offset) -> np.ndarray:
    cdef np.ndarray[np.float64_t, ndim = 2] center = np.array([[veh.state['pos'][0], veh.state['pos'][1]]])
    cdef double rot = calculate_rot_angle(np.asarray(veh.state['dir']))
    cdef np.ndarray[np.float64_t, ndim = 2] rot_matrix = np.array([[math.cos(rot), math.sin(rot)], [-math.sin(rot), math.cos(rot)]])
    center = np.dot(center, rot_matrix.T)
    rot_matrix = np.array([[math.cos(-rot), math.sin(-rot)], [-math.sin(-rot), math.cos(-rot)]])
    center += offset
    center = np.dot(center, rot_matrix.T).reshape(2)
    return center