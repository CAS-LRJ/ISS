from ISS.algorithms.planning.dubins cimport dubins
from ISS.algorithms.planning.dp_planner.dp_planner cimport dp_planning_
from libc cimport math
from libcpp cimport bool
from libcpp.queue cimport priority_queue
from ISS.algorithms.utils.mathutils.angle cimport pi_2_pi, zero_2_2pi
from ISS.algorithms.utils.vehicleutils.vehicleutils cimport collision_check, bicycle_model_move
import numpy as np
cimport numpy as np


cdef extern from 'record.hpp':
    cdef cppclass Record:
        int x
        int y
        int yaw
        double x_
        double y_
        double yaw_
        double cost
        double sum_cost
        Record() except +
        Record(int x, int y, int yaw, double x_, double y_, double yaw_, double cost, double sum_cost) except + 
        bint operator<(const Record& coords)

cdef Record pos_to_record(const np.float64_t[::1] &pos, 
                double cost,
                double sum_cost,
                const np.float64_t[::1] &bounding_x, 
                const np.float64_t[::1] &bounding_y,  
                const double &XYGridSize,
                const double &YawGridSize
            ):
    cdef int x = int(math.floor((pos[0] - bounding_x[0]) / XYGridSize))
    cdef int y = int(math.floor((pos[1] - bounding_y[0]) / XYGridSize))
    cdef int yaw = int(math.floor(zero_2_2pi(pos[2]) / YawGridSize))
    return Record(x, y, yaw, pos[0], pos[1], pos[2], cost, sum_cost)

## Hybrid A star Entrance
cdef bool hybrid_a_star_planner_(np.float64_t[::1] start, 
                np.float64_t[::1] goal, 
                const np.float64_t[::1] &bounding_x, 
                const np.float64_t[::1] &bounding_y, 
                list obstacles, 
                list soft_obstacles,
                dict config_dict):
    cdef dict planning_config = config_dict['Planning']
    cdef int NSteer = planning_config['HA_NSteer']
    cdef double SoftCollisionCost = planning_config['HA_SoftCollisionCost']
    cdef double SteerChangeCostCoeff = planning_config['HA_SteerChangeCostCoeff']
    cdef double SteerCostCoeff = planning_config['HA_SteerCostCoeff']
    cdef double HeuristicCostCoeff = planning_config['HA_HeuristicCostCoeff']
    cdef double Stepsize = planning_config['HA_Stepsize']
    cdef double YawGridSize = planning_config['HA_YawGridSize']
    cdef double XYGridSize = planning_config['HA_XYGridSize']
    cdef double RouteInterval = planning_config['HA_RouteInterval']

    cdef dict vehicle_config = config_dict['Vehicle']
    cdef double Max_Steer = vehicle_config['MaxSteer']
    cdef double TurningRadius = vehicle_config['TurningRadius']
    cdef double VehicleLength = vehicle_config['VehicleLength']    
    cdef double VehicleWidth = vehicle_config['VehicleWidth']    
    cdef double VehicleRadius = math.sqrt((VehicleWidth / 2.) ** 2 + (VehicleLength / 2.) ** 2)

    cdef double DP_Stepsize = planning_config['DP_Stepsize']    

    cdef int X_SIZE = int(math.ceil((bounding_x[1] - bounding_x[0]) / XYGridSize))
    cdef int Y_SIZE = int(math.ceil((bounding_y[1] - bounding_y[0]) / XYGridSize))
    cdef int YAW_SIZE = int(math.ceil(math.pi * 2 / YawGridSize))        

    # print(np.asarray(goal))

    cdef int Goal_X = int(math.floor((goal[0] - bounding_x[0]) / XYGridSize))
    cdef int Goal_Y = int(math.floor((goal[1] - bounding_y[0]) / XYGridSize))
    cdef int Goal_Yaw = int(math.floor(zero_2_2pi(goal[2]) / YawGridSize)) 

    # print(Goal_X, Goal_Y, Goal_Yaw)   

    cdef np.uint8_t[:,:,::1] grid_travelled = np.zeros((X_SIZE, Y_SIZE, YAW_SIZE), dtype=np.uint8)

    ## Call DP Planner Get Heuristic
    cdef np.float64_t[:,::1] DP_Heuristic = dp_planning_(start, goal, bounding_x, bounding_y, obstacles, soft_obstacles, config_dict)    
    ## Generate First Node
    cdef priority_queue[Record] record_q
    record_q.push(pos_to_record(start, 0, 0, bounding_x, bounding_y, XYGridSize, YawGridSize))
    

    ## Hybrid A Star
    cdef Record current_record, new_record
    cdef int current_x, current_y, current_yaw, dp_grid_x, dp_grid_y
    cdef double steer_, new_cost, current_cost, heuristic_cost, x_, y_, yaw_, current_x_, current_y_, current_yaw_
    cdef bool hard_collision, soft_collision, found_path
    cdef np.ndarray[np.float64_t, ndim=1] next_point
    cdef np.ndarray[np.float64_t, ndim=2] potential_points
    cdef dubins._DubinsPath dubins_path
    found_path = False
    while not (record_q.empty() or found_path):
        current_record = record_q.top()
        current_x = current_record.x
        current_y = current_record.y
        current_yaw = current_record.yaw  
        current_x_ = current_record.x_
        current_y_ = current_record.y_
        current_yaw_ = current_record.yaw_        
        current_cost = current_record.cost
        record_q.pop()        
        # print(current_x, current_y, current_yaw, current_x_, current_y_, current_yaw_, current_cost)
        if not grid_travelled[current_x, current_y, current_yaw]:
            grid_travelled[current_x, current_y, current_yaw] = True            
            ## Node Expansion
            for steer_ in np.linspace(-Max_Steer, Max_Steer, NSteer):
                x_, y_, yaw_ = bicycle_model_move(current_x_, current_y_, current_yaw_, Stepsize, steer_, TurningRadius)          
                # print(x_, y_ , yaw_)      
                if (x_ >= bounding_x[0]) and (x_ <= bounding_x[1]) and (y_ >= bounding_y[0]) and (y_ <= bounding_y[1]):
                    next_point = np.array((x_, y_))
                    ## Obstacle Detection 
                    hard_collision = False
                    for kdtree_, kdtree_points_ in obstacles:                    
                        potential_index = kdtree_.query_ball_point(next_point, VehicleRadius, p=2, return_sorted=True)
                        potential_points = kdtree_points_[potential_index]
                        # print(collision_check(next_point, potential_points, yaw_, length=VehicleLength, width=VehicleWidth))
                        if collision_check(next_point, potential_points, yaw_, length=VehicleLength, width=VehicleWidth):
                            hard_collision = True
                            break
                    
                    if not hard_collision:
                        soft_collision = False
                        for kdtree_, kdtree_points_ in soft_obstacles:
                            potential_index = kdtree_.query_ball_point(next_point, VehicleRadius, p=2, return_sorted=True)
                            potential_points = kdtree_points_[potential_index]
                            if collision_check(next_point, potential_points, yaw_, length=VehicleLength, width=VehicleWidth):
                                soft_collision = True
                                break
                        
                        ## Calculate Cost
                        next_point = np.array((x_, y_, yaw_))
                        new_cost = current_cost + Stepsize + SteerCostCoeff * steer_
                        if soft_collision:
                            new_cost += SoftCollisionCost                    
                        dp_grid_x = int(math.floor((x_ - bounding_x[0]) / DP_Stepsize))
                        dp_grid_y = int(math.floor((y_ - bounding_y[0]) / DP_Stepsize))
                        dubins_path = dubins.shortest_path_(next_point, goal, TurningRadius)
                        heuristic_cost = DP_Heuristic[dp_grid_x, dp_grid_y] + dubins_path.path_length()                        
                        new_record = pos_to_record(next_point, new_cost, new_cost + HeuristicCostCoeff * heuristic_cost, bounding_x, bounding_y, XYGridSize, YawGridSize)
                        record_q.push(new_record)
                        if (new_record.x == Goal_X) and (new_record.y == Goal_Y) and (new_record.yaw == Goal_Yaw):
                            found_path = True
            pass
        pass    
    return found_path

## Python API entrance
def hybrid_a_star_planner(np.float64_t[::1] start, 
                np.float64_t[::1] goal, 
                np.float64_t[::1] bounding_x, 
                np.float64_t[::1] bounding_y, 
                list obstacles, 
                list soft_obstacles,
                dict config_dict):        
    # print(np.asarray(goal))
    return hybrid_a_star_planner_(start, goal, bounding_x, bounding_y, obstacles, soft_obstacles, config_dict)