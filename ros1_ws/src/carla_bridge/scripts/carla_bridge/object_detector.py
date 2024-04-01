import rospy
import numpy as np
import carla

import tf.transformations
from iss_manager.msg import ObjectDetection3DArray, ObjectDetection3D, StateArrayArray, StateArray, State

from ISS.algorithms.utils.angle import pi_2_pi
from ISS.algorithms.utils.cubic_spline import Spline2D


DURATION = 0.05

color_map = { 
    'red': carla.Color(255, 0, 0),
    'green': carla.Color(0, 255, 0),
    'blue': carla.Color(0, 0, 255),
    'yellow': carla.Color(255, 255, 0),
}

class GTObjectDetector:
    def __init__(self, vehicle_id, world) -> None:
        self._vehicle_id = vehicle_id
        self._world = world
        self._object_detection_pub = rospy.Publisher(rospy.get_param("object_detection_topic"), ObjectDetection3DArray, queue_size=1)
        self._MAX_DISTANCE = 100
        self._timer = rospy.Timer(rospy.Duration(1 / rospy.get_param('object_detection_frequency')), self.publish_object_detection)
        
    def publish_object_detection(self, event):
        ego_location = self._world.get_actor(self._vehicle_id).get_location()
        all_detections = ObjectDetection3DArray()
        for actor in self._world.get_actors().filter('vehicle.*'):
            if self._vehicle_id != None and self._vehicle_id == actor.id:
                continue
            if actor.get_location().distance(ego_location) > self._MAX_DISTANCE:
                continue
            detection = ObjectDetection3D()
            detection.header.stamp = rospy.Time.now()
            detection.header.frame_id = rospy.get_param("world_frame")
            detection.id = actor.id
            detection.score = 1.0
            detection.state.x = actor.get_location().x
            detection.state.y = -actor.get_location().y
            detection.state.heading_angle = pi_2_pi(-np.deg2rad(actor.get_transform().rotation.yaw))
            detection.state.velocity = np.hypot(actor.get_velocity().x, actor.get_velocity().y)
            detection.state.acceleration = np.hypot(actor.get_acceleration().x, actor.get_acceleration().y)
            detection.bbox.size.x = actor.bounding_box.extent.x * 2 * 0.7
            detection.bbox.size.y = actor.bounding_box.extent.y * 2 * 0.7
            detection.bbox.size.z = actor.bounding_box.extent.z * 2 * 0.7
            detection.bbox.center.position.x = actor.bounding_box.location.x
            detection.bbox.center.position.y = -actor.bounding_box.location.y
            detection.bbox.center.position.z = actor.bounding_box.location.z
            roll = np.deg2rad(actor.get_transform().rotation.roll)
            pitch = np.deg2rad(actor.get_transform().rotation.pitch)
            yaw = -np.deg2rad(actor.get_transform().rotation.yaw)
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            detection.bbox.center.orientation.x = quaternion[0]
            detection.bbox.center.orientation.y = quaternion[1]
            detection.bbox.center.orientation.z = quaternion[2]
            detection.bbox.center.orientation.w = quaternion[3]
            all_detections.detections.append(detection)
        self._object_detection_pub.publish(all_detections)
            
    def shutdown(self):
        self._timer.shutdown()
        self._object_detection_pub.unregister()


class LAVObjectDetector:
    def __init__(self, vehicle_id, world) -> None:
        self._world = world
        self._vehicle_id = vehicle_id
        self._object_detection_pub = rospy.Publisher(rospy.get_param("object_detection_topic"), ObjectDetection3DArray, queue_size=1)
        self._prediction_pub = rospy.Publisher(rospy.get_param("motion_prediction_topic"), StateArrayArray, queue_size=1)
        self._MAX_DISTANCE = 20
        self._METER_PER_PIXEL = 1 / 4
        self._THRESHOLD_PREDICTION = 0.2
        self._NUM_POINTS = 50
    
    def publish_prediction(self, other_cast_locs, other_cast_cmds, ego_transform_matrix):
        all_trajs = StateArrayArray()
        for trajs, cmds in zip(other_cast_locs, other_cast_cmds):
            for traj, score in zip(trajs, cmds):
                if score < self._THRESHOLD_PREDICTION:
                    continue
                state_array = StateArray()
                for idx, loc in enumerate(traj):
                    loc_rotated = rotate(loc, -np.pi/2)
                    loc_vec = np.array([-loc_rotated[0], -loc_rotated[1], 0.5, 1])
                    loc_transformed = ego_transform_matrix @ loc_vec
                    self._world.debug.draw_string(carla.Location(x=loc_transformed[0], y=loc_transformed[1], z=loc_transformed[2]), str(idx), life_time=DURATION, color=color_map['red'])
                    state_array.states.append(State(x=loc_transformed[0], y=-loc_transformed[1]))
                all_trajs.trajectories.append(state_array)
        self._prediction_pub.publish(all_trajs)
        
    def publish_object_detection(self, det, ego_transform_matrix):
        all_detections = ObjectDetection3DArray()
        for det_x, det_y, det_ww, det_hh, cos, sin in det[1]:
            x = (det_x - 160) * self._METER_PER_PIXEL
            y = (det_y - 280) * self._METER_PER_PIXEL
            loc_rotated = rotate((x, y), -np.pi/2)
            rot_cos = sin
            rot_sin = -cos
            rot_ang = np.arctan2(rot_sin, rot_cos)
            loc_vec = np.array([-loc_rotated[0], -loc_rotated[1], 0, 1])
            loc_transformed = ego_transform_matrix @ loc_vec
            ww = det_ww * self._METER_PER_PIXEL
            hh = det_hh * self._METER_PER_PIXEL
            
            p1 = tuple((loc_transformed.tolist()[:2] + [-ww,-hh]@np.array([[-rot_sin,rot_cos],[-rot_cos,-rot_sin]])))
            p2 = tuple((loc_transformed.tolist()[:2] + [-ww, hh]@np.array([[-rot_sin,rot_cos],[-rot_cos,-rot_sin]])))
            p3 = tuple((loc_transformed.tolist()[:2] + [ ww, hh]@np.array([[-rot_sin,rot_cos],[-rot_cos,-rot_sin]])))
            p4 = tuple((loc_transformed.tolist()[:2] + [ ww,-hh]@np.array([[-rot_sin,rot_cos],[-rot_cos,-rot_sin]])))
            base_corners = [p1, p2, p3, p4]
            base_locations = [carla.Location(x=float(corner[0]), y=float(corner[1]), z=0) for corner in base_corners]
            for i in range(4):
                self._world.debug.draw_string(base_locations[i], str(i), life_time=DURATION, color=color_map['green'])
            
            detection = ObjectDetection3D()
            detection.header.stamp = rospy.Time.now()
            detection.header.frame_id = rospy.get_param("world_frame")
            detection.id = -1
            detection.score = 1.0
            detection.state.x = loc_transformed[0]
            detection.state.y = -loc_transformed[1]
            detection.state.heading_angle = pi_2_pi(-rot_ang)
            detection.state.velocity = 0
            detection.state.acceleration = 0
            detection.bbox.size.x = hh
            detection.bbox.size.y = ww
            detection.bbox.size.z = 1.5
            detection.bbox.center.position.x = detection.state.x 
            detection.bbox.center.position.y = detection.state.y
            detection.bbox.center.position.z = 0
            roll = 0
            pitch = 0
            yaw = detection.state.heading_angle
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            detection.bbox.center.orientation.x = quaternion[0]
            detection.bbox.center.orientation.y = quaternion[1]
            detection.bbox.center.orientation.z = quaternion[2]
            detection.bbox.center.orientation.w = quaternion[3]
            all_detections.detections.append(detection)
        self._object_detection_pub.publish(all_detections)
            
    def shutdown(self):
        self._object_detection_pub.unregister()
        

def rotate(loc, yaw):
    loc_array = np.array(loc)
    rot_mat = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    loc_rotated = rot_mat @ loc_array
    return loc_rotated