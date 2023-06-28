cimport ISS.algorithms.planning.dubins.core as core
import numpy as np
cimport numpy as np

cdef class _DubinsPath:
    cdef core.DubinsPath *ppth
    @staticmethod
    cdef _DubinsPath shortest_path_(np.float64_t[::1] q0, np.float64_t[::1] q1, double rho)

cdef _DubinsPath shortest_path_(np.float64_t[::1] q0, np.float64_t[::1] q1, double rho)