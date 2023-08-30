import numpy as np
cimport numpy as np
from libc cimport math
cimport cython


cpdef double pi_2_pi(double angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

cpdef double zero_2_2pi(double angle):
    return angle % (math.pi * 2)

cpdef double calculate_rot_angle(np.float64_t[::1] dir):
    dir = np.array(dir) / np.linalg.norm(dir)
    cdef double rot_angle
    if dir[0]>=0 and dir[1]>=0:
        rot_angle = math.asin(dir[1])
    elif dir[0]>=0 and dir[1]<0:
        rot_angle = -math.asin(abs(dir[1]))
    elif dir[0]<0 and dir[1]>=0:
        rot_angle = math.pi - math.asin(abs(dir[1]))        
    else:
        rot_angle = math.pi + math.asin(abs(dir[1]))
    if rot_angle < 0:
        rot_angle += math.pi * 2
    return rot_angle