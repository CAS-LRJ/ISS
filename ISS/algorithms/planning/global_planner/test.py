from ISS.algorithms.planning.planning_utils.lanelet2_utils import get_solid_checker
from ISS.algorithms.planning.global_planner.lanelet2_planner import Lanelet2Planner
import lanelet2
from lanelet2.projection import UtmProjector

import numpy as np
import dubins
import matplotlib.pyplot as plt

def main():
    test_map = '/home/shaohang/work_space/autonomous_vehicle/ISS/ISS/algorithms/planning/global_planner/test.osm'
    projector = UtmProjector(lanelet2.io.Origin(0., 0.))
    loadedMap, load_errors = lanelet2.io.loadRobust(test_map, projector) 

    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
    solid_checker = get_solid_checker(loadedMap)
    lanelet2_settings = dict()
    lanelet2_settings['TURNING_RADIUS'] = 0.3
    lanelet2_settings["GOAL_TORELANCE"] = 0.1
    global_planner = Lanelet2Planner(loadedMap, traffic_rules, solid_checker, lanelet2_settings)


    start = np.array([1, 0.2, np.pi])
    goal = np.array([1, -0.2, 0])

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