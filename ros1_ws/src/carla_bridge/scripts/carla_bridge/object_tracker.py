import rospy

import message_filters
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix
from geometry_msgs.msg import TwistStamped

from iss_manager.msg import ObjectTracking

class ObjectTracker:
    def __init__(self) -> None:
        # subscribe to the data sources: point clouds, inertial movement unit, gps position, gps velocity
        self._point_cloud_sub = message_filters.Subscriber(rospy.get_param("tracker_lidar_pointcloud_topic"), PointCloud2)
        self._imu_sub = message_filters.Subscriber(rospy.get_param("tracker_imu_topic"), Imu)
        self._gps_position_sub = message_filters.Subscriber(rospy.get_param("tracker_gps_position_topic"), NavSatFix)
        self._gps_velocity_sub = message_filters.Subscriber(rospy.get_param("tracker_gps_velocity_topic"), TwistStamped)
        # group all data sources based on their timestamps
        data_source = message_filters.ApproximateTimeSynchronizer(([self._point_cloud_sub, self._imu_sub, self._gps_position_sub, self._gps_velocity_sub ]), 1, rospy.get_param("tracker_data_source_granularity"))
        # register function to call on receiving grouped data
        data_source.registerCallback(self._tracking_callback)

        # declaring the published topic for the tracked objects
        self._object_tracking_pub = rospy.Publisher(rospy.get_param("object_tracking_topic"), ObjectTracking, queue_size = 1)


    def _tracking_callback(self, lidarPointCloudMsg, imuMsg, gpsPositionMsg, gpsVelocityMsg):
            