import time
import numpy as np
import numpy.linalg as LA
from queue import PriorityQueue
import dubins

import lanelet2
from lanelet2.core import BasicPoint2d

class PlanningNode:
    def __init__(self, points=[], current_lanelet_id=-1, current_index=0, edge_cost=0, heu_cost=0) -> None:
        self.points = points
        self.current_lanelet_id = current_lanelet_id
        self.current_index = current_index
        self.edge_cost = edge_cost
        self.heu_cost = heu_cost
    
    def __gt__(self, other):
        return self.edge_cost + self.heu_cost > other.edge_cost + other.heu_cost

class Lanelet2Planner(object):
    
    def __init__(self, lanelet_map, traffic_rules, collision_detector, lanelet2_settings) -> None:
        self.lanelet_map = lanelet_map
        self.traffic_rules = traffic_rules
        self.collision_detector = collision_detector
        self.route_graph = lanelet2.routing.RoutingGraph(self.lanelet_map, self.traffic_rules)
        self.route = None
        self.goal_pos = None
        self.turning_radius = lanelet2_settings["TURNING_RADIUS"]
        self.goal_torelance = lanelet2_settings["GOAL_TORELANCE"]
    
    def run_step(self, start_pos, goal_pos):
        self.goal_pos = goal_pos
        self.route = self.route_graph.getRoute(start_pos, goal_pos)
        fromLanelet = lanelet2.geometry.findNearest(
            self.lanelet_map.laneletLayer, BasicPoint2d(start_pos[0], start_pos[1]), 1)[0][1]
        toLanelet = lanelet2.geometry.findNearest(
            self.lanelet_map.laneletLayer, BasicPoint2d(goal_pos[0], goal_pos[1]), 1)[0][1]
        assert self.route is not None, "No route found"
        assert fromLanelet is not None, "No fromLanelet found"
        assert toLanelet is not None, "No toLanelet found"
        
        min_dis = None
        for ind, point in enumerate(fromLanelet_centerline[:-1]):
            dis = np.linalg.norm(
                [start_pos[0] - point.x, start_pos[1] - point.y])
            if min_dis == None or dis < min_dis:
                start_ind = ind
                min_dis = dis