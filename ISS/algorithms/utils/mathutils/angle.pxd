import numpy as np
cimport numpy as np

## Normalize an angle to [-pi, pi] degrees
cdef double pi_2_pi(double angle)

## Normalize an angle to [0, 2pi] degrees
cdef double zero_2_2pi(double angle)

## Get Rotation Angle of a direction vector
cdef double calculate_rot_angle(np.float64_t[::1] dir)