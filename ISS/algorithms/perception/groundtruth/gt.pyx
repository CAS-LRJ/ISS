# import cv2 as cv
# import numpy as np
import os
import sys

import carla

from ISS.algorithms.sensors.carla_camera import CameraBase, RgbCamera, DepthCamera, SemanticSegmentationCamera
from ISS.algorithms.sensors.carla_infrastucture import Infrastructure
from ISS.algorithms.sensors.carla_lidar import Lidar
from ISS.algorithms.sensors.carla_radar import Radar
from ISS.algorithms.sensors.carla_vehicle import Vehicle, OtherVehicle

from ISS.algorithms.utils.dataexchange.sensor.camera import CameraOutput, Camera3DOutput
from ISS.algorithms.utils.dataexchange.sensor.lidar import LiDAROutput, LiDARSegOutput
from ISS.algorithms.utils.dataexchange.sensor.radar import RadarOutput

def test():
    a = CameraBase()