from ISS.algorithms.utils.lanelet2_utils import get_solid_checker
from ISS.algorithms.planning.global_planner.lanelet2_planner import Lanelet2Planner
import lanelet2
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d

import numpy as np
import dubins
import matplotlib.pyplot as plt
import time
import pickle

import warnings
warnings.filterwarnings("ignore")

# test_map = '/home/shaohang/work_space/autonomous_vehicle/ISS/ros1_ws/src/iss_manager/maps/simple_road_v1.osm'
test_map = "/home/shaohang/work_space/autonomous_vehicle/ISS/ros1_ws/src/iss_manager/maps/Town06_hy.osm"
projector = UtmProjector(lanelet2.io.Origin(0., 0.))
loadedMap, load_errors = lanelet2.io.loadRobust(test_map, projector) 

traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                lanelet2.traffic_rules.Participants.Vehicle)
solid_checker, _ = get_solid_checker(loadedMap, 4.6, 2)
lanelet2_settings = dict()
lanelet2_settings['TURNING_RADIUS'] = 5
lanelet2_settings["POSITION_TOLERANCE"] = 1
lanelet2_settings["YAW_TOLERANCE"] = 0.1
global_planner = Lanelet2Planner(loadedMap, traffic_rules, solid_checker, lanelet2_settings)

spawn_points = pickle.load(open('spawn_points.pkl', 'rb'))

start = spawn_points[1]
start = (start[0], -start[1], start[2])
idx_list = [17]
for i in range(5, 30):
    print("-------------------")
    goal = spawn_points[i]
    goal = (goal[0], -goal[1], goal[2])
    # print("start: ", start)
    # print("goal: ", goal)
    start_time = time.time()
    traj = global_planner.run_step(start, goal)
    end_time = time.time()
    print("total time: ", end_time - start_time)
    if traj is None:
        print("Failed with idx: ", i)
    else:
        pass
        # print("Success with idx: ", i)
        # points = np.array(traj.get_waypoints())
        # plt.plot(points[:, 0], points[:, 1])
        # plt.plot(start[0], start[1], 'ro')
        # plt.axes().set_xlim(-200, 700)
        # plt.axes().set_ylim(-100, 80)
        # plt.axes().set_aspect('equal')
        # plt.show()



# start = (584.831238, 23.577756, 3.134272156385384)
# # goal = (284.831238, 15.577756, 3.134272156385384)
# goal = (122.56680297851562, -45.25875473022461, 0.0009416936964334374)
# start_time = time.time()
# traj = global_planner.run_step(start, goal)
# end_time = time.time()
# print("total time: ", end_time - start_time)
# points = np.array(traj.get_waypoints())
# plt.plot(points[:, 0], points[:, 1])
# plt.plot(start[0], start[1], 'ro')
# plt.axes().set_xlim(-200, 700)
# plt.axes().set_ylim(-100, 80)
# plt.axes().set_aspect('equal')
# plt.show()

# save spawn points into a file
# import carla
# import pickle
# import numpy as np

# client = carla.Client('localhost', 2000)
# client.set_timeout(10.0)
# world = client.load_world('Town06')
# map = world.get_map()
# spawn_points = map.get_spawn_points()
# spawn_points_list = []
# for point in spawn_points:
#     yaw = point.rotation.yaw
#     yaw = np.deg2rad(-yaw)
#     spawn_points_list.append((point.location.x, point.location.y, yaw))

# with open('spawn_points.pkl', 'wb') as f:
#     pickle.dump(spawn_points_list, f)
    