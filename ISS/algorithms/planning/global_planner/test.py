from ISS.algorithms.planning.global_planner.lanelet2_planner import Lanelet2Planner
import lanelet2
from lanelet2.projection import UtmProjector
import numpy as np
from ISS.algorithms.planning.planning_utils.lanelet2_utils import get_solid_checker
import dubins
import matplotlib.pyplot as plt

def main():
    lanelet2_town06 = '/home/shaohang/work_space/autonomous_vehicle/iss_ws/src/ISS/planning/maps/Town06_hy.osm'
    projector = UtmProjector(lanelet2.io.Origin(0., 0.))
    loadedMap, load_errors = lanelet2.io.loadRobust(lanelet2_town06, projector) 

    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
    solid_checker = get_solid_checker(loadedMap)
    lanelet2_settings = dict()
    lanelet2_settings['TURNING_RADIUS'] = 5
    lanelet2_settings["GOAL_TORELANCE"] = 2
    global_planner = Lanelet2Planner(loadedMap, traffic_rules, solid_checker, lanelet2_settings)

    start = np.array([584.8312377929688, 13.57775592803955, 3.1342721563853835])
    goal = np.array([586.9081420898438, 24.062835693359375, 3.1342721563853835])
    start = np.array([0, 0, 0])
    goal = np.array([10, 10, 0])
    dubins_path = dubins.shortest_path(start, goal, 5)
    # trajectory = global_planner.run_step(start, goal)
    # print(trajectory.get_waypoints())

if __name__ == "__main__":
    # main()  
    start = np.array([0, 0, 3.13 / 2])
    goal = np.array([2, 10, 3.13 / 2])
    dubins_path = dubins.shortest_path(start, goal, 5)  
    dubins_points, dubins_distances = dubins_path.sample_many(0.1)
    dubins_points_list = []
    for t in dubins_points:
        dubins_points_list.append([t[0], t[1], t[2]])
    dubins_points_list = np.array(dubins_points_list)
    plt.plot(dubins_points_list[:, 0], dubins_points_list[:, 1])
    plt.plot(start[0], start[1], 'ro')
    plt.plot(goal[0], goal[1], 'ro')
    plt.show()