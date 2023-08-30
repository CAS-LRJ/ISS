from ISS.algorithms.perception.detection_3d.base import Detection3DBase
from ISS.algorithms.utils.dataexchange.perception.object_detection import ObjectDetectionOutput
from ISS.algorithms.utils.sensorutils.transform import carla_bbox_to_bbox, carla_location_to_location, carla_transform_to_transform

import carla
import os
import numpy as np
from typing import List

class Detection3Dgt(Detection3DBase):
    def _preprocess(self, detection_3d_input):
        pass

    def detect(self, carla_world):
        # directly return all actors in the world
        res = []
        for actor in carla_world.get_actors():
            output = ObjectDetectionOutput()
            output._score = 1
            # "static", "vehicle", "walker", "traffic"
            if actor.type_id.startswith("vehicle"):
                output._label = "vehicle"
                print("VEHICLE: " + actor.type_id)
                output._bbox = carla_bbox_to_bbox(actor.bounding_box)
            elif actor.type_id.startswith("walker"):
                output._label = "walker"
                print("WALKER:" + actor.type_id)
                output._bbox = carla_bbox_to_bbox(actor.bounding_box)
            #BUG: cannot invoke get_light_boxes() for traffic light
            # elif actor.type_id.startswith("traffic.traffic_light"):
            #     print("LIGHT:" + actor.type_id)
            #     output._bbox = actor.get_light_boxes()
            else:
                #TODO: add more types
                continue
            output._loc = carla_location_to_location(actor.get_location())
            output._trans = carla_transform_to_transform(actor.get_transform())
            res.append(output)
        return res