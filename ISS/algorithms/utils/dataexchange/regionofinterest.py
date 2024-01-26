import numpy as np
import rospy
from sensor_msgs.msg import RegionOfInterest

class ISSRegionOfInterest(object):

    def __init__(self) -> None:
        self.x_offset = 0
        self.y_offset = 0
        self.height = 0
        self.width = 0
        self.do_rectify = False

    def __init__(self, region_of_interest: RegionOfInterest):
        self.x_offset = region_of_interest.x_offset
        self.y_offset = region_of_interest.y_offset
        self.height = region_of_interest.height
        self.width = region_of_interest.width
        self.do_rectify = region_of_interest.do_rectify