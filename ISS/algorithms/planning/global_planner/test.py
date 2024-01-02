from ISS.algorithms.utils.lanelet2_utils import get_solid_checker
from ISS.algorithms.planning.global_planner.lanelet2_planner import Lanelet2Planner
import lanelet2
from lanelet2.projection import UtmProjector

import numpy as np
import dubins
import matplotlib.pyplot as plt

def main():
    # test_map = '/home/shaohang/work_space/autonomous_vehicle/ISS/ros1_ws/src/iss_manager/maps/simple_road_v1.osm'
    test_map = "/home/shaohang/work_space/autonomous_vehicle/ISS/ros1_ws/src/iss_manager/maps/Town06_hy.osm"
    projector = UtmProjector(lanelet2.io.Origin(0., 0.))
    loadedMap, load_errors = lanelet2.io.loadRobust(test_map, projector) 

    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
    solid_checker, _ = get_solid_checker(loadedMap, 4.6, 2)
    lanelet2_settings = dict()
    lanelet2_settings['TURNING_RADIUS'] = 5
    lanelet2_settings["GOAL_TORELANCE"] = 5
    global_planner = Lanelet2Planner(loadedMap, traffic_rules, solid_checker, lanelet2_settings)


    start = (584.8312377929688, 13.57775592803955, 3.134272156385384)
    goal = (399.70245361328125, 20.892736434936523, -3.1379505144830007)


    # start = (584.8312377929688, 13.57775592803955, 3.134272156385384)
    # goal = (399.70245361328125, 20.892736434936523, -3.1379505144830007)


    traj = global_planner.run_step(start, goal)
    points = np.array(traj.get_waypoints())
    plt.plot(points[:, 0], points[:, 1])
    plt.show()

if __name__ == "__main__":
    main()  
    # start = np.array([0, 0, 3.13 / 2])
    # goal = np.array([2, 10, 3.13 / 2])
    # dubins_path = dubins.shortest_path(start, goal, 5)  
    # dubins_points, dubins_distances = dubins_path.sample_many(0.1)
    # dubins_points_list = []
    # for t in dubins_points:
    #     dubins_points_list.append([t[0], t[1], t[2]])
    # dubins_points_list = np.array(dubins_points_list)
    # plt.plot(dubins_points_list[:, 0], dubins_points_list[:, 1])
    # plt.plot(start[0], start[1], 'ro')
    # plt.plot(goal[0], goal[1], 'ro')
    # plt.show()