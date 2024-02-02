import numpy as np
import time
import matplotlib.pyplot as plt
from math import cos, sin, tan, pi

from ISS.algorithms.planning.local_planner.mpcc.utils import Vehicle, ReferencePath
from ISS.algorithms.planning.local_planner.mpcc.contouring_control import ContouringController
from ISS.algorithms.planning.local_planner.mpcc.road_geometry import CenterLine

class MPCCPlanner:
    def __init__(self, mpcc_setting) -> None:
        pass
    
    def update_reference_line(self, waypoints):
        self._reference_path = ReferencePath(np.array(waypoints))
        cost_weight = np.array([1., 2., 1., 1., 1.]) # [longitudinal error, contouring error, speed, acceleration, stering angle]
        targets = [5.] # desired speed
        dt = 0.1
        self._vehicle = Vehicle(cost_weight, 
                                targets, 
                                dt,
                                self._reference_path,
                                wheel_base=0.28) 
        self._planner = ContouringController(self._vehicle)
    
    def run_step(self, init_state):
        states, inputs, _ = self._planner.solve(self._reference_path, init_state) 
        return states