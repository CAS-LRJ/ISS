#!/usr/bin/env python

import rospy

from iss_manager.msg import ObjectTracker

class TrackingManagerNode:
    def __init__(self) -> None:
        self._object_tracking_pub = rospy.Publisher(rospy.get_param("object_tracking_topic"), ObjectTracker, queue_size = 1)
    
if __name__ == "__main__":
    rospy.init_node("tracking_manager_node", anonymous=True)
    tracking_manager_node = TrackingManagerNode()
    rospy.spin()
