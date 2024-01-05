import numpy as np
import time
import matplotlib.pyplot as plt
from math import cos, sin, tan, pi

from ISS.algorithms.planning.local_planner.mpcc.draw import draw_vehicle
from ISS.algorithms.planning.local_planner.mpcc.utils import Vehicle, ReferencePath
from ISS.algorithms.planning.local_planner.mpcc.contouring_control import ContouringController
from ISS.algorithms.planning.local_planner.mpcc.road_geometry import CenterLine

class MPCCPlanner:
    def __init__(self, mpcc_setting) -> None:
        pass
    
    def run_step(self, init_state, waypoints):
        reference_path = ReferencePath(waypoints)
        
        dt = 0.1
        horizon = 50
        params = {"L": 4., "W": 2., "WB": 3.,
                "dt": dt,
                "horizon": horizon}
                    
        cost_weight = np.array([1., 2., 1., 1., 1.]) # [longitudinal error, contouring error, speed, acceleration, stering angle]
        # initial_state = np.array([0., -1., 0., 5.]) # [x, y, theta, v]

        targets = [5.] # desired speed
        
        vehicle = Vehicle(cost_weight, 
                        targets, 
                        dt,
                        reference_path) 
        planner = ContouringController(vehicle)
        states, inputs, _ = planner.solve(reference_path, initial_state) 
        
        return states
    
if __name__ == "__main__":
    lane_width = 3.5
    line_config = [
        ['line', 30],
        ['arc', 10, -pi/2],
        ['line', 10],
        ['arc', 5, pi],
        ['line', 36],
        # ['arc', 12, -pi],
        # ['line', 50]
    ]
    start_pose = [0., 0., 0.]
    center_line = CenterLine(line_config, start_pose, lane_width=lane_width, resolution=2.5)
    waypoints = center_line.get_2Dpoints()
    initial_state = np.array([0., -1., 0., 5.])
    mpcc_wrapper = MPCCPlanner(None)
    
    time_start = time.time()
    for i in range(100):
        states = mpcc_wrapper.run_step(initial_state, waypoints)
    print("Time elapsed: ", time.time() - time_start)