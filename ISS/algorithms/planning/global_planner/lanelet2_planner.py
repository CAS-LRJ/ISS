import numpy as np
import numpy.linalg as LA
from queue import PriorityQueue
import dubins
import cvxpy as cp
import math
import time

import lanelet2
from lanelet2.core import BasicPoint2d

from ISS.algorithms.utils.angle import calculate_rot_angle, pi_2_pi
from ISS.algorithms.utils.trajectory import Trajectory


def smooth(trajectory: Trajectory):
    points = trajectory.get_states_array()[:, :2]
    points_opt = cp.Variable(points.shape)
    cost = 0
    for i in range(1, points.shape[0] - 1):
        cost += cp.norm(points_opt[i + 1] + points_opt[i - 1] - 2 * points_opt[i]) ** 2
    for i in range(0, points.shape[0]):
        cost += cp.norm(points_opt[i] - points[i]) ** 2
    constraints = []
    for i in range(points.shape[0]):
        constraints += [points_opt[i, 0] <= points[i, 0] + 0.1]
        constraints += [points_opt[i, 0] >= points[i, 0] - 0.1]
        constraints += [points_opt[i, 1] <= points[i, 1] + 0.1]
        constraints += [points_opt[i, 1] >= points[i, 1] - 0.1]
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(verbose=False, solver=cp.ECOS)
    if prob.status == cp.OPTIMAL:
        trajectory.update_positions(points_opt.value)
    else:
        print("Error: Cannot smooth the trajectory")

class PlanningNode:
    def __init__(self, points=[], current_lanelet_id=-1, current_index=0, lane_change_num=0, distance=0., heuristic=0):
        self.points = points
        self.current_lanelet_id = current_lanelet_id
        self.current_index = current_index
        self.lane_change_num = lane_change_num
        self.distance = distance
        self.heuristic = heuristic

    def __gt__(self, other):
        if (self.distance + self.heuristic) > (other.distance + other.heuristic):
            return True
        elif (self.distance + self.heuristic) < (other.distance + other.heuristic):
            return False

        if self.lane_change_num > other.lane_change_num:
            return True
        elif self.lane_change_num < other.lane_change_num:
            return False

        return False


class Lanelet2Planner(object):

    def __init__(self, lanelet_map, traffic_rules, collision_detector, lanelet2_settings) -> None:
        self.lanelet_map = lanelet_map
        self.traffic_rules = traffic_rules
        self.collision_detector = collision_detector
        self.route_graph = lanelet2.routing.RoutingGraph(self.lanelet_map, self.traffic_rules)
        self.route = None
        self.goal_pos = None
        self.closed_set = set()
        for key, value in lanelet2_settings.items():
            setattr(self, key, value)
        
    def is_close(self, current_point, target_point):
        pos_diff = LA.norm([current_point[0] - target_point[0], current_point[1] - target_point[1]])
        yaw_diff = pi_2_pi(np.abs(current_point[2] - target_point[2]))
        return (pos_diff < self.POSITION_TOLERANCE) and (yaw_diff < self.YAW_TOLERANCE)
    
    def is_dubins_valid(self, start_point, target_point, dubins_points):
        if dubins_points is None:
            return False
        sta_x, sta_y, _ = start_point
        tar_x, tar_y, _ = target_point
        direction = math.atan2(tar_y - sta_y, tar_x - sta_x)
        for dp in dubins_points:
            yaw = dp[2]
            angle_diff = pi_2_pi(np.abs(direction - yaw))
            if abs(angle_diff) > math.pi / 3:
                return False
        return True

    def expand(self, node):
        current_lanelet = self.lanelet_map.laneletLayer[node.current_lanelet_id]
        results = []
        current_lanelet_centerline = list(current_lanelet.centerline)
        current_lanelet_length = len(current_lanelet_centerline)
        # Lane Change
        lanechange_lanelets = []
        left_lanelet = self.route.leftRelation(current_lanelet)
        if left_lanelet is not None:
            lanechange_lanelets.append(left_lanelet.lanelet)
        right_lanelet = self.route.rightRelation(current_lanelet)
        if right_lanelet is not None:
            lanechange_lanelets.append(right_lanelet.lanelet)
        for target_lanelet in lanechange_lanelets:
            if target_lanelet.id in self.closed_set:
                continue
            passed_points = []
            new_node = None
            target_lanelet_centerline = list(target_lanelet.centerline)
            target_lanelet_length = len(target_lanelet_centerline)
            cur_ind = node.current_index
            target_ind = node.current_index # TODO: Change this to 0
            while (cur_ind < current_lanelet_length - 1) and (target_ind < target_lanelet_length - 1):
                cur_point = current_lanelet_centerline[cur_ind]
                cur_nextpoint = current_lanelet_centerline[cur_ind+1]
                cur_rot = calculate_rot_angle(np.array([cur_nextpoint.x - cur_point.x, (cur_nextpoint.y - cur_point.y)]))
                tar_point = target_lanelet_centerline[target_ind]
                tar_nextpoint = target_lanelet_centerline[target_ind+1]
                tar_rot = calculate_rot_angle(np.array([tar_nextpoint.x - tar_point.x, (tar_nextpoint.y - tar_point.y)]))

                dubins_path = dubins.shortest_path((cur_point.x, cur_point.y, cur_rot), (tar_point.x, tar_point.y, tar_rot), self.TURNING_RADIUS)
                dubins_points, dubins_dis = dubins_path.sample_many(0.5)
                if self.is_dubins_valid([cur_point.x, cur_point.y, cur_rot], [tar_point.x, tar_point.y, tar_rot], dubins_points):
                    new_node = PlanningNode(node.points + dubins_points, target_lanelet.id,
                                            target_ind, node.lane_change_num + 1, dubins_dis[-1] + node.distance, heuristic=LA.norm([self.goal_pos[0] - tar_point.x, self.goal_pos[1] - tar_point.y]))
                    break
                target_ind += 1
            if new_node:
                results.append(new_node)
        # Forwards Connection
        cur_remain_points = []
        for cur_ind in range(node.current_index, current_lanelet_length-1):
            cur_point = current_lanelet_centerline[cur_ind]
            cur_nextpoint = current_lanelet_centerline[cur_ind+1]
            cur_rot = calculate_rot_angle(
                np.array([cur_nextpoint.x - cur_point.x, (cur_nextpoint.y - cur_point.y)]))
            cur_remain_points.append((cur_point.x, cur_point.y, cur_rot))
        forward_lanelets = self.route.followingRelations(current_lanelet)
        if forward_lanelets is not None:
            for forward_ in forward_lanelets:
                forward_lanelet = forward_.lanelet
                forward_centerline = list(forward_lanelet.centerline)
                if len(forward_centerline) > 1:
                    first_point = forward_centerline[0]
                dist = LA.norm([cur_remain_points[0][0] - first_point.x, cur_remain_points[0][1] - first_point.y])
                new_node = PlanningNode(node.points + cur_remain_points,
                                       forward_lanelet.id, 0, node.lane_change_num, node.distance + dist, heuristic=LA.norm([self.goal_pos[0] - first_point.x, self.goal_pos[1] - first_point.y]))
                results.append(new_node)
        return results

    def run_step(self, start_pos, goal_pos):
        self.goal_pos = goal_pos
        fromLanelet = lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, BasicPoint2d(start_pos[0], start_pos[1]), 1)[0][1]
        toLanelet = lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, BasicPoint2d(goal_pos[0], goal_pos[1]), 1)[0][1]
        self.route = self.route_graph.getRoute(fromLanelet, toLanelet)
        assert fromLanelet != None and toLanelet != None
        assert self.route != None, "Global Planner: Cannot find a route!"

        # Assume the planner start from the closest lanelet!
        # Warning: Sometime the start lanelet is the second closest one! To-DO: Fix this
        fromLanelet_centerline = list(fromLanelet.centerline)
        start_ind = 0
        min_dis = None
        for ind, point in enumerate(fromLanelet_centerline[:-1]):
            dis = LA.norm(
                [start_pos[0] - point.x, start_pos[1] - point.y])
            if min_dis == None or dis < min_dis:
                start_ind = ind
                min_dis = dis
        closest_node = None
        for ind in range(start_ind, len(fromLanelet_centerline)-1):
            point = fromLanelet_centerline[ind]
            rot = calculate_rot_angle(np.array([fromLanelet_centerline[ind + 1].x - point.x, (fromLanelet_centerline[ind + 1].y - point.y)]))
            if self.is_close([point.x, point.y, rot], start_pos):
                closest_node = PlanningNode([start_pos], fromLanelet.id, ind, 0, 0)
                break
            dubins_path = dubins.shortest_path(start_pos, (point.x, point.y, rot), self.TURNING_RADIUS)
            dubins_points, dubins_dis = dubins_path.sample_many(0.5)
            target_point = [point.x, point.y, rot]
            if self.is_dubins_valid(start_pos, target_point, dubins_points):
                closest_node = PlanningNode([start_pos] + dubins_points, fromLanelet.id, ind, 0, dubins_dis[-1])
                break
        assert closest_node is not None, "Failed to find a valid closest node"

        open_set = PriorityQueue()
        open_set.put(closest_node)
        self.closed_set.add(closest_node.current_lanelet_id)
        while not open_set.empty():
            current_node = open_set.get()
            if current_node.current_lanelet_id == toLanelet.id:
                toLanelet_centerline = list(toLanelet.centerline)
                start_ind = current_node.current_index
                for ind in range(start_ind, len(toLanelet_centerline)-1):
                    point = toLanelet_centerline[ind]
                    rot = calculate_rot_angle(np.array([toLanelet_centerline[ind+1].x - point.x, (toLanelet_centerline[ind+1].y - point.y)]))
                    get_valid_path = False
                    dubins_points = []
                    if self.is_close([point.x, point.y, rot], goal_pos):
                        get_valid_path = True
                    else: 
                        dubins_path = dubins.shortest_path((point.x, point.y, rot), goal_pos, self.TURNING_RADIUS)
                        dubins_points, dubins_dis = dubins_path.sample_many(0.5)
                        if self.is_dubins_valid([point.x, point.y, rot], goal_pos, dubins_points):
                            get_valid_path = True
                    if get_valid_path:
                        point_list = current_node.points
                        for traj_ind in range(current_node.current_index, ind):
                            point = toLanelet_centerline[traj_ind]
                            rot = calculate_rot_angle(np.array([toLanelet_centerline[traj_ind+1].x - point.x, (toLanelet_centerline[traj_ind+1].y - point.y)]))
                            point_list.append([point.x, point.y, rot])
                        point_list += dubins_points
                        point_list.append(goal_pos)
                        traj = Trajectory()
                        traj.update_waypoints(point_list)
                        return traj
                    
            results = self.expand(current_node)
            for result_node in results:
                self.closed_set.add(result_node.current_lanelet_id)
                open_set.put(result_node)

        return None

