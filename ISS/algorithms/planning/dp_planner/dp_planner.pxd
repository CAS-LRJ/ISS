import numpy as np
cimport numpy as np

cdef np.float64_t[:,::1] dp_planning_(const np.float64_t[::1] &start,
                const np.float64_t[::1] &goal, 
                const np.float64_t[::1] &bounding_x, 
                const np.float64_t[::1] &bounding_y, 
                list obstacles, 
                list soft_obstacles, 
                dict config_dict                
            )