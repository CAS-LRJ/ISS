import numpy as np
cimport numpy as np
from libc cimport math
cimport cython
from libcpp cimport bool
from libcpp.deque cimport deque
from scipy.spatial import cKDTree

cdef extern from 'pointcloud.hpp':    
    cdef cppclass PointCloud:
        double x
        double y
        double cost
        double stepsize
        PointCloud() except +
        PointCloud(double x, double y, double z, double stepsize) except +         
        bint operator<(const PointCloud& pc)
    
    # double pc_stepsize "PointCloud.stepsize"

cdef struct Region:
    double center_x
    double center_y
    double radius

@cython.boundscheck(False)
@cython.wraparound(False)
cdef np.float64_t[:,::1] handle_clouds_data(np.float64_t[:,::1] clouds,
                np.float64_t[::1] ego_pos, 
                double heights_lower,
                double heights_upper,
                dict config_dict):
    cdef dict mapping_config_dict = config_dict['Mapping']
    cdef double detect_radius = mapping_config_dict['FLM_DetectRadius']
    cdef np.ndarray[np.float64_t, ndim=2] clouds_ = np.asarray(clouds)
    cdef np.ndarray[np.uint8_t, ndim=1] clouds_valid        
    clouds_valid = np.greater(np.linalg.norm(clouds_, axis=1), 0.)        
    clouds_ = clouds_[clouds_valid]
    # print(clouds_.shape[0])
    cdef np.ndarray[np.uint8_t, ndim=1] clouds_heightvalid = np.equal(clouds_[:, 2].clip(heights_lower, heights_upper), clouds_[:, 2])
    # clouds_ = clouds_[clouds_valid]        
    cdef np.ndarray[np.uint8_t, ndim=1] clouds_egovalid = np.greater(np.linalg.norm(clouds_ - ego_pos, axis=1), 3.)    
    # clouds_ = clouds_[clouds_valid]    
    cdef np.ndarray[np.uint8_t, ndim=1] clouds_xvalid = np.equal(clouds_[:, 0].clip(ego_pos[0] - detect_radius, ego_pos[0] + detect_radius - 0.01), clouds_[:, 0])
    # clouds_ = clouds_[clouds_xvalid]
    cdef np.ndarray[np.uint8_t, ndim=1] clouds_yvalid = np.equal(clouds_[:, 1].clip(ego_pos[1] - detect_radius, ego_pos[1] + detect_radius - 0.01), clouds_[:, 1])
    clouds_ = clouds_[clouds_xvalid & clouds_yvalid & clouds_egovalid & clouds_heightvalid]
    # clouds_ = np.asarray(sorted(clouds_, key=lambda x: (math.ceil(x[0] / stepsize), math.ceil(x[1] / stepsize), x[2])), dtype=np.float64)    
    clouds_ = clouds_[clouds_[:,2].argsort()]
    # print(clouds_.shape[0])
    return clouds_

## 下面这段代码是无效的，因为其不能很好的处理局部浮空点
# cdef np.float64_t[:,::1] generate_occupancy_grid_fast(const np.float64_t[:,::1] &clouds,
#                 const np.float64_t[::1] &ego_pos,
#                 const double &detect_radius,                
#                 const double &stepsize,
#                 const double &tolerance,
#                 const double &vehicle_height):
#     cdef int pc_len = clouds.shape[0]
#     cdef int i
#     cdef double leftup_x = ego_pos[0] - detect_radius
#     cdef double leftup_y = ego_pos[1] - detect_radius
#     cdef int map_size = int(math.ceil(detect_radius / stepsize) * 2)
#     ## Assert the Z coords greater than zero
#     cdef np.float64_t[:,::1] lowest = -np.ones((map_size, map_size))
#     cdef np.float64_t[:,::1] second_lowest = -np.ones((map_size, map_size))    

#     cdef int coord_x
#     cdef int coord_y
#     cdef list obstacles = []
#     for i in range(pc_len):                
#         coord_x = int(math.floor((clouds[i, 0] - leftup_x) / stepsize))
#         coord_y = int(math.floor((clouds[i, 1] - leftup_y) / stepsize))
#         # if (clouds[i, 1] <= 113) and (clouds[i, 1] >= 110) and (clouds[i, 0] <= -714) and (clouds[i, 0] >= -718):
#         #     print(clouds[i, 0], clouds[i, 1], clouds[i, 2])
#         #     print(coord_x, coord_y)
#         if second_lowest[coord_x, coord_y] <= 0.:
#             if lowest[coord_x, coord_y] <= 0.:
#                 lowest[coord_x, coord_y] = clouds[i, 2]
#             else:
#                 if clouds[i, 2] >= lowest[coord_x, coord_y] + tolerance:
#                     second_lowest[coord_x, coord_y] = clouds[i, 2]
#                     if clouds[i, 2] <= lowest[coord_x, coord_y] + tolerance + vehicle_height:                        
#                         # if (clouds[i, 1] <= 113) and (clouds[i, 1] >= 110) and (clouds[i, 0] <= -714) and (clouds[i, 0] >= -718):
#                         #     print('Found', lowest[coord_x, coord_y], second_lowest[coord_x, coord_y], coord_x, coord_y)
#                         obstacles.append(((coord_x + 0.5) * stepsize + leftup_x, (coord_y + 0.5) * stepsize + leftup_y))
#     return np.asarray(obstacles)


# cdef np.float64_t[:,::1] generate_occupancy_grid_fast(const np.float64_t[:,::1] &clouds,
#                 const np.float64_t[::1] &ego_pos,
#                 const double &detect_radius,                
#                 const double &stepsize,
#                 const double &tolerance,
#                 const double &vehicle_height,
#                 list pre_resolution = [12.8, 3.2, 0.8]):
#     cdef int pc_len = clouds.shape[0]
#     cdef int i
#     cdef double leftup_x = ego_pos[0] - detect_radius
#     cdef double leftup_y = ego_pos[1] - detect_radius
#     cdef int map_size = int(math.ceil(detect_radius / stepsize) * 2)
#     ## Assert the Z coords greater than zero
#     cdef np.float64_t[:,::1] lowest = -np.ones((map_size, map_size))
#     cdef np.float64_t[:,::1] second_lowest = -np.ones((map_size, map_size))    

#     cdef int coord_x
#     cdef int coord_y
#     cdef list obstacles = []
#     for i in range(pc_len):                
#         coord_x = int(math.floor((clouds[i, 0] - leftup_x) / stepsize))
#         coord_y = int(math.floor((clouds[i, 1] - leftup_y) / stepsize))
#         # if (clouds[i, 1] <= 113) and (clouds[i, 1] >= 110) and (clouds[i, 0] <= -714) and (clouds[i, 0] >= -718):
#         #     print(clouds[i, 0], clouds[i, 1], clouds[i, 2])
#         #     print(coord_x, coord_y)
#         if second_lowest[coord_x, coord_y] <= 0.:
#             if lowest[coord_x, coord_y] <= 0.:
#                 lowest[coord_x, coord_y] = clouds[i, 2]
#             else:
#                 if clouds[i, 2] >= lowest[coord_x, coord_y] + tolerance:
#                     second_lowest[coord_x, coord_y] = clouds[i, 2]
#                     if clouds[i, 2] <= lowest[coord_x, coord_y] + tolerance + vehicle_height:                        
#                         # if (clouds[i, 1] <= 113) and (clouds[i, 1] >= 110) and (clouds[i, 0] <= -714) and (clouds[i, 0] >= -718):
#                         #     print('Found', lowest[coord_x, coord_y], second_lowest[coord_x, coord_y], coord_x, coord_y)
#                         obstacles.append(((coord_x + 0.5) * stepsize + leftup_x, (coord_y + 0.5) * stepsize + leftup_y))
#     return np.asarray(obstacles)

@cython.boundscheck(False)
@cython.wraparound(False)
cdef np.float64_t[:,::1] generate_occupancy_grid_fast(const np.float64_t[:,::1] &clouds,
                const np.float64_t[::1] &ego_pos,
                dict config_dict,
                # const double &detect_radius,                
                # const double &stepsize,
                # const double &tolerance,
                # const double &vehicle_height
                ):
    cdef dict mapping_config_dict = config_dict['Mapping']
    cdef double detect_radius = mapping_config_dict['FLM_DetectRadius']
    cdef double stepsize = mapping_config_dict['FLM_Stepsize']
    cdef double tolerance = mapping_config_dict['FLM_Tolerance']
    cdef double vehicle_height = mapping_config_dict['FLM_VehicleHeight']
    point_kdtree = cKDTree(clouds[:, :2])
    cdef deque[Region] region_list    

    cdef double leftup_x = ego_pos[0] - detect_radius
    cdef double leftup_y = ego_pos[1] - detect_radius

    cdef Region init_region
    cdef Region current_region
    cdef list obstacles = []
    cdef list result    
    cdef double lowest
    cdef double temp
    cdef double half_r
    cdef int i, x_grid, y_grid    
    for x_grid in range(10):
        for y_grid in range(10):
            region_list.clear()
            init_region = Region(leftup_x + (x_grid + 0.5) * detect_radius / 10. * 2, leftup_y + (y_grid + 0.5) * detect_radius / 10. * 2, detect_radius / 10.)
            region_list.push_back(init_region)

            while not region_list.empty():
                current_region = region_list.front()
                region_list.pop_front()
                result = point_kdtree.query_ball_point((current_region.center_x, current_region.center_y), current_region.radius, p=float('inf'), return_sorted=True)            
                # if current_region.radius <= stepsize:
                #     obstacles.append((current_region.center_x, current_region.center_y))
                # else:                    
                if len(result) > 1:
                    lowest = clouds[result[0], 2]
                    for i in range(1, len(result)):
                        temp = clouds[result[i], 2]
                        if temp >= lowest + tolerance:                        
                            if temp <= lowest + tolerance + vehicle_height:         
                                if current_region.radius <= stepsize:
                                    obstacles.append((current_region.center_x, current_region.center_y))
                                else:
                                    ## Split
                                    half_r = current_region.radius / 2.
                                    region_list.push_back(Region(current_region.center_x - half_r, current_region.center_y - half_r, half_r))
                                    region_list.push_back(Region(current_region.center_x - half_r, current_region.center_y + half_r, half_r))
                                    region_list.push_back(Region(current_region.center_x + half_r, current_region.center_y - half_r, half_r))
                                    region_list.push_back(Region(current_region.center_x + half_r, current_region.center_y + half_r, half_r))                            
                            break                            
    return np.asarray(obstacles)


# cdef void generate_occupancy_grid_fast(np.float64_t[:,::1] clouds,
#                 np.float64_t[::1] center,
#                 double detect_radius,
#                 const double &stepsize,
#                 const double &tolerance,
#                 const double &vehicle_height):
#     cdef np.ndarray[np.float64_t, ndim=2] clouds_ = np.asarray(clouds)
#     cdef np.ndarray[np.float64_t, ndim=1] center_ = np.asarray(center)
#     cdef np.ndarray[np.uint8_t, ndim=1] clouds_xvalid = clouds_[:, 0] < center[0]
#     cdef np.ndarray[np.uint8_t, ndim=1] clouds_yvalid = clouds_[:, 1] < center[1]
    
#     cdef np.ndarray[np.float64_t, ndim=2] clouds_0, clouds_1, clouds_2, clouds_3
#     clouds_0 = clouds_[clouds_xvalid & clouds_yvalid]
#     clouds_1 = clouds_[clouds_xvalid & np.logical_not(clouds_yvalid)]
#     clouds_2 = clouds_[np.logical_not(clouds_xvalid) & clouds_yvalid]
#     clouds_3 = clouds_[np.logical_not(clouds_xvalid) & np.logical_not(clouds_yvalid)]
    
#     cdef bool find_obstacles
#     cdef int clouds_len
#     cdef double ground_height
#     cdef int i

#     clouds_len = clouds_0.shape[0]    
#     if clouds_len > 1:
#         find_obstacles = False
#         ground_height = clouds_0[0, 2]
#         for i in range(1, clouds_len):
#             if clouds_0[i][2] >= ground_height + tolerance:                        
#                 if clouds_0[i][2] <= ground_height + tolerance + vehicle_height:
#                     find_obstacles = True
#                 break
#     # print(find_obstacles)
#     if find_obstacles:
#         if detect_radius <= stepsize:
#             pass
#         else:
#             generate_occupancy_grid_fast(clouds_0, center_ + np.array([-detect_radius, -detect_radius, 0], dtype = np.float64), detect_radius / 2., stepsize, tolerance, vehicle_height)
    
#     clouds_len = clouds_1.shape[0]    
#     if clouds_len > 1:
#         find_obstacles = False
#         ground_height = clouds_1[0, 2]        
#         for i in range(1, clouds_len):
#             if clouds_1[i][2] >= ground_height + tolerance:                        
#                 if clouds_1[i][2] <= ground_height + tolerance + vehicle_height:
#                     find_obstacles = True
#                 break
#     if find_obstacles:
#         if detect_radius <= stepsize:
#             pass
#         else:
#             generate_occupancy_grid_fast(clouds_1, center_ + np.array([-detect_radius, detect_radius, 0], dtype = np.float64), detect_radius / 2., stepsize, tolerance, vehicle_height)

#     clouds_len = clouds_2.shape[0]    
#     if clouds_len > 1:
#         find_obstacles = False
#         ground_height = clouds_2[0, 2]
#         for i in range(1, clouds_len):
#             if clouds_2[i][2] >= ground_height + tolerance:                        
#                 if clouds_2[i][2] <= ground_height + tolerance + vehicle_height:
#                     find_obstacles = True
#                 break
#     if find_obstacles:
#         if detect_radius <= stepsize:
#             pass
#         else:
#             generate_occupancy_grid_fast(clouds_2, center_ + np.array([detect_radius, -detect_radius, 0], dtype = np.float64), detect_radius / 2., stepsize, tolerance, vehicle_height)
    
#     clouds_len = clouds_3.shape[0]    
#     if clouds_len > 1:
#         find_obstacles = False
#         ground_height = clouds_3[0, 2]
#         for i in range(1, clouds_len):
#             if clouds_3[i][2] >= ground_height + tolerance:                        
#                 if clouds_3[i][2] <= ground_height + tolerance + vehicle_height:
#                     find_obstacles = True
#                 break
#     if find_obstacles:
#         if detect_radius <= stepsize:
#             pass
#         else:
#             generate_occupancy_grid_fast(clouds_3, center_ + np.array([detect_radius, detect_radius, 0], dtype = np.float64), detect_radius / 2., stepsize, tolerance, vehicle_height)

def fast_obstacle_detection(np.ndarray[np.float64_t, ndim=2] clouds,
                np.ndarray[np.float64_t, ndim=1] ego_pos, 
                double heights_lower,
                double heights_upper,
                dict config_dict,
                # double detect_radius,
                # double stepsize,
                # double tolerance,
                # double vehicle_height
                ) -> np.ndarray:
    cdef np.float64_t[:,::1] clouds_cleaned    
    clouds_cleaned = handle_clouds_data(clouds, ego_pos, heights_lower, heights_upper, config_dict)
    return np.asarray(generate_occupancy_grid_fast(clouds_cleaned, ego_pos, config_dict))
    # return np.asarray(clouds_cleaned)
    # return np.asarray(generate_occupancy_grid_fast(clouds_cleaned, ego_pos, detect_radius, stepsize, tolerance, vehicle_height))

def happy():    
    cdef PointCloud pc1 = PointCloud(2.3, 4.0, 5.0, 0.2)
    cdef PointCloud pc2 = PointCloud(2.2, 3.0, 4.0, 0.2)    
    # PointCloud.stepsize = 0.2
    # print(pc1.stepsize, pc2.stepsize)
    print(math.floor(pc1.x / pc1.stepsize), math.floor(pc2.x / pc2.stepsize))
    print(pc1.x, pc2.x)
    print(pc1.stepsize, pc2.stepsize)
    print(pc1 < pc2)