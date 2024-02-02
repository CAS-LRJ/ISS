import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, tan, pi
import math
import casadi as ca
import copy
from matplotlib.patches import Circle

from ISS.algorithms.planning.local_planner.mpcc.draw import draw_vehicle
from ISS.algorithms.planning.local_planner.mpcc.utils import Vehicle, ReferencePath
from ISS.algorithms.planning.local_planner.mpcc.contouring_control import ContouringController
from ISS.algorithms.planning.local_planner.mpcc.road_geometry import CenterLine

def f_dyn(q, u, params):
    WB = params['WB']
    x, y, theta, v = q
    a, phi = u
    q_dot = np.zeros(q.shape)
    q_dot[0] = v * cos(theta)
    q_dot[1] = v * sin(theta)
    q_dot[2] = v * tan(phi) / WB
    q_dot[3] = a    
    return q_dot

def update(q, u, params):
    q_ = q + f_dyn(q, u, params)*params["dt"]
    return q_

if __name__ == "__main__":
     # Line + Arc + Line + Arc + ...
    # lane_width = 3.5
    # line_config = [
    #     ['line', 30],
    #     ['arc', 10, -pi/2],
    #     ['line', 10],
    #     ['arc', 5, pi],
    #     ['line', 36],
    #     # ['arc', 12, -pi],
    #     # ['line', 50]
    # ]
    # start_pose = [0., 0., 0.]
    # center_line = CenterLine(line_config, start_pose, lane_width=lane_width, resolution=2.5)
    # waypoints = center_line.get_2Dpoints()
    import pickle
    waypoints = pickle.load(open('/home/shaohang/work_space/autonomous_vehicle/ISS/points.pkl', 'rb'))

    
    
    reference_path = ReferencePath(waypoints)
    
    dt = 0.1
    horizon = 50
    params = {"L": 0.35, "W": 0.22, "WB": 0.28,
              "dt": dt,
              "horizon": horizon}
                
    cost_weight = np.array([1, 1, 1, 1, 1]) # [longitudinal error, contouring error, speed, acceleration, stering angle]
    initial_state = np.array([9.76, 1.16, 3.14, 0]) # [x, y, theta, v]

    targets = [0.5] # desired speed
    
    vehicle = Vehicle(cost_weight, 
                      targets, 
                      dt,
                      reference_path,
                      wheel_base=0.28) 
    # planner = ContouringController(vehicle)
    
    T_sim = 15.
    time = 0.
    
    state = copy.deepcopy(initial_state)
    
    x =  [state[0]]
    y = [state[1]]
    theta = [state[2]]
    v = [state[3]]
    
    ss = np.arange(reference_path.s_min, reference_path.s_max, 0.1)
    draw_points = []
    for s in ss:
        draw_points.append(list(reference_path.get_point(s)))
    draw_points = np.array(draw_points)
    plt.plot(draw_points[:,0], draw_points[:,1], 'k--')
    
    # while time <= T_sim:
        
    #     # k = math.floor(time/dt)
    #     k = math.ceil(time/dt)
        
    #     states, inputs, _ = planner.solve(reference_path, state)     
    #     u_con = inputs[:,0]    
                
    #     x.append(state[0])
    #     y.append(state[1])
    #     theta.append(state[2])
    #     v.append(state[3])

    #     plt.cla()
    #     plt.gcf().canvas.mpl_connect('key_release_event',
    #                 lambda event: [exit(0) if event.key == 'escape' else None])
    #     # plt.plot(x, y, "ob", label="trajectory")
    #     # plt.plot(states[0,:], states[1,:], '-o', color='darkorange')
        
        
    #     # center_line.plot_center_line()     
    #     draw_vehicle(pose=state[:3], params=params, alpha=0.8)
                
    #     plt.axis("equal")
    #     plt.grid(True)
   
    #     plt.pause(0.1)

    #     state = update(state, u_con, params)
    
    #     time += dt
        
    plt.show()
    