from ISS.algorithms.utils.lanelet2_utils import get_solid_checker
from ISS.algorithms.planning.global_planner.lanelet2_planner import Lanelet2Planner
import lanelet2
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d

import numpy as np
import dubins
import matplotlib.pyplot as plt
import time

# test_map = '/home/shaohang/work_space/autonomous_vehicle/ISS/ros1_ws/src/iss_manager/maps/simple_road_v1.osm'
test_map = "/home/shaohang/work_space/autonomous_vehicle/ISS/ros1_ws/src/iss_manager/maps/Town06_hy.osm"
projector = UtmProjector(lanelet2.io.Origin(0., 0.))
loadedMap, load_errors = lanelet2.io.loadRobust(test_map, projector) 

traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                lanelet2.traffic_rules.Participants.Vehicle)
solid_checker, _ = get_solid_checker(loadedMap, 4.6, 2)
lanelet2_settings = dict()
lanelet2_settings['TURNING_RADIUS'] = 5
lanelet2_settings["GOAL_TORELANCE"] = 1
global_planner = Lanelet2Planner(loadedMap, traffic_rules, solid_checker, lanelet2_settings)


start = (584.831238, 23.577756, 3.134272156385384)
# goal = (284.831238, 15.577756, 3.134272156385384)
goal = (200.566803, -45.258755, 0)

start_time = time.time()
traj = global_planner.run_step(start, goal)
end_time = time.time()
print("total time: ", end_time - start_time)
points = np.array(traj.get_waypoints())
plt.plot(points[:, 0], points[:, 1])
plt.plot(start[0], start[1], 'ro')
plt.axes().set_xlim(-200, 700)
plt.axes().set_ylim(-100, 80)
plt.axes().set_aspect('equal')
plt.show()