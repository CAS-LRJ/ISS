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
test_map = "/home/shaohang/work_space/autonomous_vehicle/ISS/ros1_ws/src/iss_manager/maps/road_2024.osm"
projector = UtmProjector(lanelet2.io.Origin(0., 0.))
loadedMap, load_errors = lanelet2.io.loadRobust(test_map, projector) 

traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                lanelet2.traffic_rules.Participants.Vehicle)
solid_checker, _ = get_solid_checker(loadedMap, 4.6, 2)
lanelet2_settings = dict()
lanelet2_settings['TURNING_RADIUS'] = 0.3
lanelet2_settings["POSITION_TOLERANCE"] = 0.2
lanelet2_settings["YAW_TOLERANCE"] = 0.1
global_planner = Lanelet2Planner(loadedMap, traffic_rules, solid_checker, lanelet2_settings)

# spawn_points = pickle.load(open('spawn_points.pkl', 'rb'))

start = (9.75, 1.16, 3.14)
# goal = (6.21, 1.13, 3.14)
# goal = (6.82, -5.25, 3.14)
# goal = (5.95, -4.47, -1.57)
goal = (6.2, -4.7, 3.14)
# goal = (3.84, 1.3, 3.14)
trajectory = global_planner.run_step(start, goal)
points = np.array(trajectory.get_waypoints())
import pickle
with open('points.pkl', 'wb') as f:
    pickle.dump(points, f)
# shortest_path = global_planner.run_step(start, goal)
# points = []
# for lanelet in shortest_path:
#     centers = list(lanelet.centerline)
#     for point in centers:
#         points.append((point.x, point.y))
# points = np.array(points)
plt.plot(points[:, 0], points[:, 1])
plt.plot(start[0], start[1], 'ro')
plt.axes().set_aspect('equal')


points = []
# read txt file of tuples
def read_txt(file_path):
    with open(file_path, 'r') as file:
        content = file.read()
    content = content.split('\n')
    points = []
    for tup_str in content:
        # tup_str = tup_str[1:-1]
        tup = tup_str.split(',')
        tup = [float(x) for x in tup]
        points.append(tup)
    points = np.array(points)
    return points
file_path = '/home/shaohang/Downloads/adjusted_coordinates.txt'
points = read_txt(file_path)
new_points = []
for point in points:
    if (point[0] > 0) and (point[1] > 400):
        continue
    if (point[0] < -500) and (point[1] < -200):
        continue
    new_points.append(point)
new_points = np.array(new_points) * 0.01 
for new_point in new_points:
    if new_point[0] == -4:
        print(new_point)
plt.scatter(new_points[:, 0], new_points[:, 1], s=0.1)
plt.show()

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
    