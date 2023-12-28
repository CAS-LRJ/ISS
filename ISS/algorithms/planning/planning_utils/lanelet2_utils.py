import numpy as np
import math
from scipy.spatial import KDTree
import matplotlib.pyplot as plt


def get_solid_checker(loadedMap, vehicle_length, vehicle_width):
    # Get Solid Points...
    inset = set()
    solid_points = []
    solid_id = []
    for lanelet in loadedMap.laneletLayer:
        if 'subtype' not in lanelet.attributes or lanelet.attributes['subtype'] != 'road':
            continue
        left_lane = lanelet.leftBound
        right_lane = lanelet.rightBound
        if "subtype" in left_lane.attributes and left_lane.attributes['subtype'] == 'solid' and left_lane.id not in inset:
            inset.add(left_lane.id)
            for point in left_lane:
                solid_id.append(left_lane.id)
                print(solid_points)
                solid_points.append((point.x, point.y))

        if "subtype" in right_lane.attributes and right_lane.attributes['subtype'] == 'solid' and right_lane.id not in inset:
            inset.add(right_lane.id)
            for point in right_lane:
                solid_id.append(right_lane.id)
                solid_points.append((point.x, point.y))
    
    # plt.scatter([point[0] for point in solid_points], [point[1] for point in solid_points])
    # plt.show()
    solid_checker = CollisionChecker(solid_points, vehicle_length, vehicle_width)
    return solid_checker


def vehicle_coord_world(center, points, yaw):
    rot_matrix = np.array(
        [[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    points_ = np.dot(points, rot_matrix) + center
    return points_


def world_coord_vehicle(center, points, yaw):
    rot_matrix = np.array(
        [[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    points_ = np.dot(points - center, rot_matrix.T)
    return points_


def collision_check(center, points, yaw, length=4.4, width=2.2):
    points_ = np.asarray(world_coord_vehicle(center, points, yaw))
    check_x = np.logical_and(np.greater_equal(
        points_[:, 0], -length / 2.), np.less_equal(points_[:, 0], length / 2.))
    check_y = np.logical_and(np.greater_equal(
        points_[:, 1], -width / 2.), np.less_equal(points_[:, 1], width / 2.))
    return np.sum(np.logical_and(check_x, check_y))


def collision_check_fronthalf(center, points, yaw, length=4.4, width=2.2):
    points_ = np.asarray(world_coord_vehicle(center, points, yaw))
    check_x = np.logical_and(np.greater_equal(
        points_[:, 0], 0.), np.less_equal(points_[:, 0], length / 2.))
    check_y = np.logical_and(np.greater_equal(
        points_[:, 1], -width / 2.), np.less_equal(points_[:, 1], width / 2.))
    return np.sum(np.logical_and(check_x, check_y))


def collision_check_index(center, points, yaw, length=4.4, width=2.2):
    points_ = np.asarray(world_coord_vehicle(center, points, yaw))
    check_x = np.logical_and(np.greater_equal(
        points_[:, 0], -length/2.), np.less_equal(points_[:, 0], length/2.))
    check_y = np.logical_and(np.greater_equal(
        points_[:, 1], -width/2.), np.less_equal(points_[:, 1], width/2.))
    return np.logical_and(check_x, check_y)

# 2D Collision Checker


class CollisionChecker(object):

    def __init__(self, points, vehicle_length, vehicle_width) -> None:
        self.points = np.array([(point[0], point[1]) for point in points])
        self.kdtree = KDTree(self.points)
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width

    def check_point(self, point, half=False):
        point = np.array(point)
        potential_index = self.kdtree.query_ball_point(
            point[:2], self.vehicle_length)
        potential_points = self.points[potential_index]
        if half:
            return collision_check_fronthalf(point[:2], potential_points, point[2], self.vehicle_length, self.vehicle_width) == 0
        return collision_check(point[:2], potential_points, point[2], self.vehicle_length, self.vehicle_width) == 0

    def check_point_index(self, point):
        point = np.array(point)
        potential_index = self.kdtree.query_ball_point(
            point[:2], self.vehicle_length)
        potential_points = self.points[potential_index]
        potential_index = np.array(potential_index)
        return potential_index[collision_check_index(point[:2], potential_points, point[2], self.vehicle_length, self.vehicle_width)]

    def check_path(self, path, half=False):
        path = np.array(path)
        path_xy = path[:, :2]
        potential_indices = self.kdtree.query_ball_point(
            path_xy, self.vehicle_length)
        for i, index in enumerate(potential_indices):
            potential_points = self.points[index]
            if half:
                if collision_check_fronthalf(path[i][:2], potential_points, path[i][2], self.vehicle_length, self.vehicle_width) > 0:
                    return True
            else:
                if collision_check(path[i][:2], potential_points, path[i][2], self.vehicle_length, self.vehicle_width) > 0:
                    return True
        return False
