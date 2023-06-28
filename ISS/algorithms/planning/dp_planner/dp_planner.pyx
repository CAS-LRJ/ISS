import numpy as np
cimport numpy as np
from libc cimport math
from libcpp.deque cimport deque
from libcpp.vector cimport vector
from libcpp.queue cimport priority_queue
cimport cython

# cdef struct MapCoords:
#     int x
#     int y     
#     double cost

# cdef struct MapCoordsCompare:
#     bool operator()(MapCoords &coords1, MapCoords &coords2):
#         return coords1.cost < coords2.cost

cdef extern from 'mapcoords.hpp':
    cdef cppclass MapCoords:
        int x
        int y
        double cost
        MapCoords() except +
        MapCoords(int x, int y, double cost) except + 
        bint operator<(const MapCoords& coords)

cdef list movement = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)]

@cython.boundscheck(False)
@cython.wraparound(False)
cdef MapCoords pos_to_map_coords(const np.float64_t[::1] &pos, 
                const np.float64_t[::1] &bounding_x, 
                const np.float64_t[::1] &bounding_y,  
                const double &stepsize
            ):
    return MapCoords(int(math.floor(pos[0] - bounding_x[0]) / stepsize), int(math.floor(pos[1] - bounding_y[0]) / stepsize), 0.)

@cython.boundscheck(False)
@cython.wraparound(False)
cdef void coords_expand(priority_queue[MapCoords] &q,
                np.float64_t[:,::1] &dp_result,
                const MapCoords &coords,
                const np.uint8_t[:,::1] &obstacle_map,
                const np.uint8_t[:,::1] &soft_obstacle_map,
                const int &size_x,
                const int &size_y,
                const double &stepsize,                
                const double &soft_penalty,                
            ):        
    ## Move Expansion
    cdef MapCoords new_coord_
    cdef (int, int, double) move_
    for move_ in movement:
        new_coord_ = MapCoords(coords.x + move_[0], coords.y + move_[1], coords.cost + move_[2] * stepsize)
        if (new_coord_.x < size_x) and (new_coord_.x >= 0) and (new_coord_.y < size_y) and (new_coord_.y >=0):
            if obstacle_map[new_coord_.x, new_coord_.y] and dp_result[new_coord_.x, new_coord_.y] < 0.:
                if soft_obstacle_map[new_coord_.x, new_coord_.y]:
                    q.push(new_coord_)
                else:
                    new_coord_.cost += soft_penalty
                    q.push(new_coord_)
                # dp_result[new_coord_.x, new_coord_.y] = new_coord_.cost

cdef inline insert_grid(deque[int] &grid_x,
                deque[int] &grid_y,
                deque[int] &grid_r,
                deque[bint] &has_obstacles,
                deque[bint] &has_soft_obstacles,
                const int &x,
                const int &y,
                const int &r,
                const bint &obstacles_detected,
                const bint &soft_obstacles_detected):
    grid_x.push_back(x)
    grid_y.push_back(y)
    grid_r.push_back(r)
    has_obstacles.push_back(obstacles_detected)
    has_soft_obstacles.push_back(soft_obstacles_detected)


@cython.boundscheck(False)
@cython.wraparound(False)
cdef void generate_obstacle_map(np.uint8_t[:,::1] &obstacle_map,
                np.uint8_t[:,::1] &soft_obstacle_map,
                const np.float64_t[::1] &bounding_x, 
                const np.float64_t[::1] &bounding_y,
                const int size_x,
                const int size_y,
                const double &stepsize,
                list obstacles,
                list soft_obstacles,
                const double &obstacle_detect,
                const int &search_stepsize):
    
    cdef int x_
    cdef int y_
    cdef int current_x
    cdef int current_y
    cdef int current_r
    cdef bint current_has_obstacles
    cdef bint current_has_soft_obstacles
    cdef bint obstacles_detected
    cdef bint soft_obstacles_detected
    cdef list res
    cdef double x_pos
    cdef double y_pos
    cdef deque[int] grid_x
    cdef deque[int] grid_y
    cdef deque[int] grid_r
    cdef deque[bint] has_obstacles
    cdef deque[bint] has_soft_obstacles

    for x_ in range(0, size_x, search_stepsize):
        for y_ in range(0, size_y, search_stepsize):                        
            grid_x.clear()
            grid_y.clear()
            grid_r.clear()
            has_obstacles.clear()
            has_soft_obstacles.clear()

            grid_x.push_back(x_)
            grid_y.push_back(y_)
            grid_r.push_back(search_stepsize // 2)
            has_obstacles.push_back(True)
            has_soft_obstacles.push_back(True)

            while (not grid_x.empty()):
                current_x = grid_x.front()
                current_y = grid_y.front()
                current_r = grid_r.front()
                current_has_obstacles = has_obstacles.front()
                current_has_soft_obstacles = has_soft_obstacles.front()

                grid_x.pop_front()
                grid_y.pop_front()
                grid_r.pop_front()
                has_obstacles.pop_front()
                has_soft_obstacles.pop_front()

                if current_r >= 1:                    
                    x_pos = (current_x + current_r) * stepsize + bounding_x[0]
                    y_pos = (current_y + current_r) * stepsize + bounding_y[0]
                    obstacles_detected = False
                    if current_has_obstacles:                        
                        for kd_tree_, _ in obstacles:
                            res = kd_tree_.query_ball_point((x_pos, y_pos), current_r * stepsize + obstacle_detect, p=float('inf'), workers = -1)
                            if len(res) > 0:
                                obstacles_detected = True
                                break
                    
                    soft_obstacles_detected = False
                    if current_has_soft_obstacles:                        
                        for kd_tree_, _ in soft_obstacles:
                            res = kd_tree_.query_ball_point((x_pos, y_pos), current_r * stepsize + obstacle_detect, p=float('inf'), workers = -1)
                            if len(res) > 0:
                                soft_obstacles_detected = True
                                break
                    
                    if obstacles_detected or soft_obstacles_detected:
                        insert_grid(grid_x, grid_y, grid_r, has_obstacles, has_soft_obstacles, current_x, current_y, current_r // 2, obstacles_detected, soft_obstacles_detected)
                        insert_grid(grid_x, grid_y, grid_r, has_obstacles, has_soft_obstacles, current_x + current_r, current_y, current_r // 2, obstacles_detected, soft_obstacles_detected)
                        insert_grid(grid_x, grid_y, grid_r, has_obstacles, has_soft_obstacles, current_x, current_y + current_r, current_r // 2, obstacles_detected, soft_obstacles_detected)
                        insert_grid(grid_x, grid_y, grid_r, has_obstacles, has_soft_obstacles, current_x + current_r, current_y + current_r, current_r // 2, obstacles_detected, soft_obstacles_detected)
                else:                    
                    x_pos = (current_x + current_r) * stepsize + bounding_x[0]
                    y_pos = (current_y + current_r) * stepsize + bounding_y[0]                    
                    if current_has_obstacles:
                        obstacle_map[current_x, current_y] = False
                    if current_has_soft_obstacles:
                        soft_obstacle_map[current_x, current_y] = False

    
@cython.boundscheck(False)
@cython.wraparound(False)
cdef np.float64_t[:,::1] dp_planning_(const np.float64_t[::1] &start, 
                const np.float64_t[::1] &goal, 
                const np.float64_t[::1] &bounding_x, 
                const np.float64_t[::1] &bounding_y, 
                list obstacles, 
                list soft_obstacles, 
                dict config_dict
                # const double &soft_penalty,
                # const double &stepsize,
                # const double &obstacle_detect,
                # const int &search_stepsize
            ):
    cdef dict planning_config = config_dict['Planning']
    cdef double soft_penalty = planning_config['DP_SoftPenalty']
    cdef double stepsize = planning_config['DP_Stepsize']
    cdef double obstacle_detect = planning_config['DP_ObstacleDetect']
    cdef int search_stepsize = planning_config['DP_ObstacleSearchStepsize']
    cdef int size_x = int(math.ceil((bounding_x[1] - bounding_x[0]) / stepsize))
    cdef int size_y = int(math.ceil((bounding_y[1] - bounding_y[0]) / stepsize))        
    cdef np.float64_t[:,::1] dp_result = -np.ones((size_x, size_y))    

    cdef np.uint8_t[:,::1] obstacle_map = np.ones((size_x, size_y), dtype = bool)
    cdef np.uint8_t[:,::1] soft_obstacle_map = np.ones((size_x, size_y), dtype = bool)
    generate_obstacle_map(obstacle_map, soft_obstacle_map, bounding_x, bounding_y, size_x, size_y, stepsize, obstacles, soft_obstacles, obstacle_detect, search_stepsize)

    cdef MapCoords goal_xy = pos_to_map_coords(goal, bounding_x, bounding_y, stepsize)
    # dp_result[goal_xy.x, goal_xy.y] = 0.

    cdef priority_queue[MapCoords] coords_deque
    coords_deque.push(goal_xy)
    cdef MapCoords current_coord
    cdef int i = 0    
    while (not coords_deque.empty()):        
        i += 1
        current_coord = coords_deque.top()
        coords_deque.pop()
        if dp_result[current_coord.x, current_coord.y] < 0.:
            dp_result[current_coord.x, current_coord.y] = current_coord.cost
            coords_expand(coords_deque, dp_result, current_coord, obstacle_map, soft_obstacle_map, size_x, size_y, stepsize, soft_penalty)        
    # print(i)
    return dp_result

@cython.boundscheck(False)
@cython.wraparound(False)
def dp_planning(np.ndarray[np.float64_t, ndim=1] start, 
                np.ndarray[np.float64_t, ndim=1] goal, 
                np.ndarray[np.float64_t, ndim=1] bounding_x, 
                np.ndarray[np.float64_t, ndim=1] bounding_y, 
                list obstacles, 
                list soft_obstacles, 
                dict config_dict
                # double soft_penalty = 1000.,
                # double stepsize = 0.2,
                # double obstacle_detect = 1.1,
                # int search_stepsize = 64
            ):
    # return np.asarray(dp_planning_(start, goal, bounding_x, bounding_y, obstacles, soft_obstacles, soft_penalty, stepsize, obstacle_detect, search_stepsize))
    return np.asarray(dp_planning_(start, goal, bounding_x, bounding_y, obstacles, soft_obstacles, config_dict))


def small_test():
    cdef priority_queue[MapCoords] q
    cdef MapCoords coord1 = MapCoords(0, 0, 100.)
    cdef MapCoords coord2 = MapCoords(1, 1, 50.)
    cdef MapCoords coord3 = MapCoords(1, 1, 150.)
    cdef MapCoords current_coord
    q.push(coord1)
    q.push(coord1)
    q.push(coord2)
    q.push(coord3)
    while not q.empty():
        current_coord = q.top()
        q.pop()
        print(current_coord.x, current_coord.y, current_coord.cost)