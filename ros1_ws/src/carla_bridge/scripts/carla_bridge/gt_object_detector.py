import rospy
import numpy as np

import tf.transformations
from iss_manager.msg import ObjectDetection3DArray, ObjectDetection3D

from ISS.algorithms.planning.planning_utils.angle import pi_2_pi


class GTObjectDetector:
    def __init__(self, vehicle_id, world) -> None:
        self._vehicle_id = vehicle_id
        self._world = world
        gt_object_detection_frequency = rospy.get_param('~gt_object_detection_frequency', 10)
        self._timer = rospy.Timer(rospy.Duration(1 / gt_object_detection_frequency), self._timer_callback)
        self._object_detection_pub = rospy.Publisher("carla_bridge/gt_object_detection", ObjectDetection3DArray, queue_size=1)
        self._MAX_DISTANCE = 20
        
    def _timer_callback(self, event):
        ego_location = self._world.get_actor(self._vehicle_id).get_location()
        all_detections = ObjectDetection3DArray()
        for actor in self._world.get_actors().filter('vehicle.*'):
            if self._vehicle_id != None and self._vehicle_id == actor.id:
                continue
            if actor.get_location().distance(ego_location) > self._MAX_DISTANCE:
                continue
            detection = ObjectDetection3D()
            detection.header.stamp = rospy.Time.now()
            detection.header.frame_id = "map"
            detection.id = actor.id
            detection.score = 1.0
            detection.state.x = actor.get_location().x
            detection.state.y = -actor.get_location().y
            detection.state.heading_angle = pi_2_pi(-np.deg2rad(actor.get_transform().rotation.yaw))
            detection.state.velocity = np.hypot(actor.get_velocity().x, actor.get_velocity().y)
            detection.state.acceleration = np.hypot(actor.get_acceleration().x, actor.get_acceleration().y)
            detection.bbox.size.x = actor.bounding_box.extent.x * 2
            detection.bbox.size.y = actor.bounding_box.extent.y * 2
            detection.bbox.size.z = actor.bounding_box.extent.z * 2
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
            