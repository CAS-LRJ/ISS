import numpy as np
from queue import PriorityQueue
import lanelet2
import dubins
import cvxpy as cp
import time

from lanelet2.core import (AllWayStop, AttributeMap, BasicPoint2d,
                           BoundingBox2d, Lanelet, LaneletMap,
                           LaneletWithStopLine, LineString3d, Point2d, Point3d,
                           RightOfWay, TrafficLight, getId)
from lanelet2.projection import (UtmProjector, MercatorProjector,
                                 LocalCartesianProjector, GeocentricProjector)

from ISS.algorithms.planning.planning_utils.angle import calculate_rot_angle
from ISS.algorithms.planning.planning_utils.trajectory import Trajectory

# To-DO: Use mapping objects instead of fixed lanelet2

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
    prob.solve(verbose=False)
    if prob.status == cp.OPTIMAL:
        trajectory.update_positions(points_opt.value)
    else:
        print("Error: Cannot smooth the trajectory")

class PlanningNode:
    def __init__(self, points=[], current_lanelet_id=-1, current_index=0, lane_change_num=0, distance=0., solid_conflicts=0, conflicts=set()):
        self.points = points
        self.current_lanelet_id = current_lanelet_id
        self.current_index = current_index
        self.lane_change_num = lane_change_num
        self.distance = distance

    def __gt__(self, other):
        if self.distance > other.distance:
            return True
        elif self.distance < other.distance:
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
        self.route_graph = lanelet2.routing.RoutingGraph(
            self.lanelet_map, self.traffic_rules)
        self.route = None
        self.goal_pos = None
        self.turning_radius = lanelet2_settings["TURNING_RADIUS"]
        self.goal_torelance = lanelet2_settings["GOAL_TORELANCE"]

    def is_goal_reached(self, ego_state):
        return np.linalg.norm([ego_state.x - self.goal_pos[0], ego_state.y - self.goal_pos[1]]) < self.goal_torelance

    def explore(self, node):
        current_lanelet = self.lanelet_map.laneletLayer[node.current_lanelet_id]
        lanechange_lanelets = []
        left_lanelet = self.route.leftRelation(current_lanelet)
        if left_lanelet is not None:
            lanechange_lanelets.append(left_lanelet.lanelet)

        right_lanelet = self.route.rightRelation(current_lanelet)
        if right_lanelet is not None:
            lanechange_lanelets.append(right_lanelet.lanelet)

        # Pair Algorithm
        results = []
        current_lanelet_centerline = list(current_lanelet.centerline)
        current_lanelet_length = len(current_lanelet_centerline)
        for target_lanelet in lanechange_lanelets:
            if target_lanelet.id in self.reach_map:
                # To-DO: Here we should judge whether it can access closer waypoint of the road
                continue
            passed_points = []
            found_path = False
            minNode = None
            target_lanelet_centerline = list(target_lanelet.centerline)
            target_lanelet_length = len(target_lanelet_centerline)
            cur_ind = node.current_index
            target_ind = node.current_index
            while (cur_ind < current_lanelet_length - 1) and (target_ind < target_lanelet_length - 1):
                cur_point = current_lanelet_centerline[cur_ind]
                cur_nextpoint = current_lanelet_centerline[cur_ind+1]
                cur_rot = calculate_rot_angle(
                    np.array([cur_nextpoint.x - cur_point.x, (cur_nextpoint.y - cur_point.y)]))
                tar_point = target_lanelet_centerline[target_ind]
                tar_nextpoint = target_lanelet_centerline[target_ind+1]
                tar_rot = calculate_rot_angle(
                    np.array([tar_nextpoint.x - tar_point.x, (tar_nextpoint.y - tar_point.y)]))

                dubins_path = dubins.shortest_path(
                    (cur_point.x, cur_point.y, cur_rot), (tar_point.x, tar_point.y, tar_rot), self.turning_radius)
                dubins_points, dubins_dis = dubins_path.sample_many(0.1)
                if len(dubins_points) > 0 and np.linalg.norm([cur_point.x - tar_point.x, cur_point.y - tar_point.y]) * 1.5 > dubins_dis[-1]:
                    check_result = self.collision_detector.check_path(
                        dubins_points)
                    if check_result:
                        found_path = True
                        newNode = PlanningNode(node.points + passed_points + list(dubins_points), target_lanelet.id,
                                               target_ind, node.lane_change_num + 1, dubins_dis[-1] + node.distance)
                        if minNode == None or newNode < minNode:
                            minNode = newNode
                        # Early Stop
                        break
                    passed_points.append((cur_point.x, cur_point.y, cur_rot))
                    cur_ind += 1
                else:
                    target_ind += 1
            if found_path:
                results.append(minNode)

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
                    second_point = forward_centerline[1]
                    rot_ = calculate_rot_angle(
                        np.array([second_point.x - first_point.x, (second_point.y - first_point.y)]))
                newNode = PlanningNode(node.points + cur_remain_points,
                                       forward_lanelet.id, 0, node.lane_change_num, node.distance)
                results.append(newNode)
        return results

    def run_step(self, start_pos, goal_pos):
        self.goal_pos = goal_pos
        self.reach_map = dict()
        fromLanelet = lanelet2.geometry.findNearest(
            self.lanelet_map.laneletLayer, BasicPoint2d(start_pos[0], start_pos[1]), 1)[0][1]
        toLanelet = lanelet2.geometry.findNearest(
            self.lanelet_map.laneletLayer, BasicPoint2d(goal_pos[0], goal_pos[1]), 1)[0][1]

        self.route = self.route_graph.getRoute(fromLanelet, toLanelet)
        assert fromLanelet != None and toLanelet != None
        assert self.route != None

        # Assume the planner start from the closest lanelet!
        # Warning: Sometime the start lanelet is the second closest one! To-DO: Fix this
        fromLanelet_centerline = list(fromLanelet.centerline)
        start_ind = 0
        min_dis = None
        for ind, point in enumerate(fromLanelet_centerline[:-1]):
            dis = np.linalg.norm(
                [start_pos[0] - point.x, start_pos[1] - point.y])
            if min_dis == None or dis < min_dis:
                start_ind = ind
                min_dis = dis
        cloesetNode = None
        for ind in range(start_ind, len(fromLanelet_centerline)-1):
            point = fromLanelet_centerline[ind]
            rot = calculate_rot_angle(np.array(
                [fromLanelet_centerline[ind + 1].x - point.x, (fromLanelet_centerline[ind + 1].y - point.y)]))
            if (np.linalg.norm([start_pos[0] - point.x, start_pos[1] - point.y]) < 0.5 and np.abs(rot - start_pos[2]) < 0.1) \
                or (np.linalg.norm([start_pos[0] - point.x, start_pos[1] - point.y]) < 0.1):
                cloesetNode = PlanningNode([start_pos], fromLanelet.id, ind, 0, 0)
                break
            dubins_path = dubins.shortest_path(
                start_pos, (point.x, point.y, rot), self.turning_radius)
            dubins_points, dubins_dis = dubins_path.sample_many(0.1)
            if len(dubins_points) > 0 and np.linalg.norm([start_pos[0] - point.x, start_pos[1] - point.y]) * 1.5 > dubins_dis[-1]:
                # check_result = check_path(solid_points, dubins_points, solid_kdtree, vehicle_length, vehicle_width)
                check_result = self.collision_detector.check_path(
                    dubins_points)
                if check_result:
                    newNode = PlanningNode(
                        [start_pos] + list(dubins_points), fromLanelet.id, ind, 0, dubins_dis[-1])
                    if cloesetNode == None or cloesetNode > newNode:
                        cloesetNode = newNode
        assert cloesetNode != None

        pqueue = PriorityQueue()
        pqueue.put(cloesetNode)
        self.reach_map[cloesetNode.current_lanelet_id] = cloesetNode.current_index
        while not pqueue.empty():
            currentNode = pqueue.get()
            
            # print(currentNode.current_lanelet_id)
            if currentNode.current_lanelet_id == toLanelet.id:
                toLanelet_centerline = list(toLanelet.centerline)
                start_ind = currentNode.current_index
                min_dis = None
                # for ind in range(start_ind, len(toLanelet_centerline)-1):
                #     point = toLanelet_centerline[ind]
                #     dis = np.linalg.norm([goal_pos[0] - point.x, goal_pos[1] - point.y])
                #     if min_dis == None or dis < min_dis:
                #         start_ind = ind
                #         min_dis = dis
                for ind in range(start_ind, len(toLanelet_centerline)-1):
                    point = toLanelet_centerline[ind]
                    rot = calculate_rot_angle(np.array(
                        [toLanelet_centerline[ind+1].x - point.x, (toLanelet_centerline[ind+1].y - point.y)]))
                    if np.linalg.norm([goal_pos[0] - point.x, goal_pos[1] - point.y]) < 0.5 and np.abs(rot - goal_pos[2]) < 0.1:
                        point_list = currentNode.points
                        for traj_ind in range(currentNode.current_index, ind):
                            point = toLanelet_centerline[traj_ind]
                            rot = calculate_rot_angle(np.array(
                                [toLanelet_centerline[traj_ind+1].x - point.x, (toLanelet_centerline[traj_ind+1].y - point.y)]))
                            point_list.append([point.x, point.y, rot])
                        point_list.append(goal_pos)
                        traj = Trajectory()
                        traj.update_waypoints(point_list)
                        if self.turning_radius < 2: # small robot
                            smooth(traj)
                        return traj
                    dubins_path = dubins.shortest_path(
                        (point.x, point.y, rot), goal_pos, self.turning_radius)
                    dubins_points, dubins_dis = dubins_path.sample_many(0.1)
                    if len(dubins_points) > 0 and np.linalg.norm([goal_pos[0] - point.x, goal_pos[1] - point.y]) * 1.5 > dubins_dis[-1]:
                        check_result = self.collision_detector.check_path(
                            dubins_points)
                        if check_result:
                            point_list = currentNode.points
                            for traj_ind in range(currentNode.current_index, ind):
                                point = toLanelet_centerline[traj_ind]
                                rot = calculate_rot_angle(np.array(
                                    [toLanelet_centerline[traj_ind+1].x - point.x, (toLanelet_centerline[traj_ind+1].y - point.y)]))
                                point_list.append([point.x, point.y, rot])
                            goal_pos_list = [goal_pos[0], goal_pos[1], goal_pos[2]] 
                            point_list = point_list + \
                                list(dubins_points) + [goal_pos_list]
                            # Global path does not need any speed information
                            traj = Trajectory()
                            traj.update_waypoints(point_list)   
                            if self.turning_radius < 2: # small robot
                                smooth(traj)
                            return traj

            results = self.explore(currentNode)
            for result_node in results:
                self.reach_map[result_node.current_lanelet_id] = result_node.current_index
                pqueue.put(result_node)

        return None

