import numpy as np
cimport numpy as np

## Collision Check
cdef int collision_check(np.ndarray[np.float64_t, ndim=1] center, np.ndarray[np.float64_t, ndim=2] points, double yaw, double length=*, double width=*)

## Bicycle Model
cdef tuple bicycle_model_move(double x, double y, double yaw, double distance, double steer, double L)