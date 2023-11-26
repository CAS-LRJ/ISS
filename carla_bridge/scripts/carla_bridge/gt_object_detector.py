import rospy
import numpy as np

from iss_msgs.msg import DetectionArray, Detection

class GTObjectDetector:
    def __init__(self, vehicle_id, world) -> None:
        self._vehicle_id = vehicle_id
        self._world = world
        gt_object_detection_frequency = rospy.get_param('~gt_object_detection_frequency', 10)
        self._timer = rospy.Timer(rospy.Duration(1 / gt_object_detection_frequency), self._timer_callback)
        self._object_detection_pub = rospy.Publisher("carla_bridge/gt_object_detection", DetectionArray, queue_size=1)
        self._MAX_DISTANCE = 100
        
    def _timer_callback(self, event):
        ego_location = self._world.get_actor(self._vehicle_id).get_location()
        all_detections = DetectionArray()
        for actor in self._world.get_actors().filter('vehicle.*'):
            if self.vehicle_id != None and self._vehicle_id == actor.id:
                continue
            if actor.get_location().distance(ego_location) > self.MAX_DISTANCE:
                continue
            detection = Detection()
            detection.header.stamp = rospy.Time.now()
            detection.header.frame_id = "map"
            detection.id = 0
            detection.score = 1.0
            detection.state.x = actor.get_location().x
            detection.state.y = -actor.get_location().y
            detection.state.heading_angle = -np.deg2rad(actor.get_transform().rotation.yaw)
            detection.state.velocity = np.hypot(actor.get_velocity().x, actor.get_velocity().y)
            detection.state.acceleration = np.hypot(actor.get_acceleration().x, actor.get_acceleration().y)
            # detection.bbox.size = 
            
            
            