import numpy as np
cimport numpy as np
from libc cimport math
cimport cython
from ISS.algorithms.utils.mathutils.angle cimport pi_2_pi

@cython.boundscheck(False)
@cython.wraparound(False)
cdef np.float64_t[:,::1] vehicle_coord_world(np.ndarray[np.float64_t, ndim=1] center,                 
                np.ndarray[np.float64_t, ndim=2] points, 
                double yaw):
    cdef np.ndarray[np.float64_t, ndim=2] rot_matrix = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    cdef np.ndarray[np.float64_t, ndim=2] points_ = np.dot(points, rot_matrix) + center
    return points_

@cython.boundscheck(False)
@cython.wraparound(False)
cdef np.float64_t[:,::1] world_coord_vehicle(np.ndarray[np.float64_t, ndim=1] center,                 
                np.ndarray[np.float64_t, ndim=2] points, 
                double yaw):
    cdef np.ndarray[np.float64_t, ndim=2] rot_matrix = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    cdef np.ndarray[np.float64_t, ndim=2] points_ = np.dot(points - center, rot_matrix.T)
    return points_

@cython.boundscheck(False)
@cython.wraparound(False)
cdef int collision_check(np.ndarray[np.float64_t, ndim=1] center,                 
                np.ndarray[np.float64_t, ndim=2] points, 
                double yaw, 
                double length=4.4, 
                double width=2.2):        
    # cdef np.ndarray[np.float64_t, ndim=2] rot_matrix = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    # cdef np.ndarray[np.float64_t, ndim=2] points_ = np.dot(points - center, rot_matrix.T)
    cdef np.ndarray[np.float64_t, ndim=2] points_ = np.asarray(world_coord_vehicle(center, points, yaw))
    cdef np.ndarray[np.uint8_t, ndim=1] check_x = np.logical_and(np.greater_equal(points_[:,0], -length/2.), np.less_equal(points_[:,0], length/2.))
    cdef np.ndarray[np.uint8_t, ndim=1] check_y = np.logical_and(np.greater_equal(points_[:,1], -width/2.), np.less_equal(points_[:,1], width/2.))    
    return sum(np.logical_and(check_x, check_y))


## Bicycle Model
cdef tuple bicycle_model_move(double x, 
    double y, 
    double yaw, 
    double distance, 
    double steer, 
    double L):
    x += distance * math.cos(yaw)
    y += distance * math.sin(yaw)
    yaw += pi_2_pi(distance * math.tan(steer) / L)  # distance/2        

    return (x, y, yaw)