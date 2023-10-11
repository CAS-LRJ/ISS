import argparse
import json
import os
import sys
import time
import signal
from pathlib import Path

import carla

# Project Root Path
ROOT_PATH = Path(__file__).parent.parent.as_posix()
# set env path
sys.path.append(ROOT_PATH)


# from ISS.algorithms.sensors.carla_camera import CarlaRgbCamera
# from ISS.algorithms.sensors.carla_lidar import CarlaLiDAR
# from ISS.algorithms.utils.dataexchange.sensor.camera import CameraOutput
# from ISS.algorithms.utils.dataexchange.sensor.lidar import LiDAROutput
# from ISS.algorithms.utils.dataexchange.perception.object_detection import ObjectDetectionOutput
# from ISS.algorithms.utils.sensorutils.transform import Transform, Location, Rotation
from ISS.algorithms.sensors.carla_gnss import CarlaGNSS
from ISS.algorithms.utils.dataexchange.sensor.gnss import GNSSOutput

RAW_DATA_PATH = "{}/resources/data/carla/{}".format(ROOT_PATH, 'raw_data')
DATASET_PATH = "{}/resources/data/carla/{}".format(ROOT_PATH, 'dataset')

def main():
    # create world by config file
    carla_client = carla.Client("localhost", 2000)
    carla_client.set_timeout(10.0)
    with open(ROOT_PATH + "/config/world_config_template.json") as handle:
        json_settings = json.loads(handle.read())
        carla_client.load_world(json_settings["map"])
        world = carla_client.get_world()
        dh = world.debug
        # random spawn things
        blib = world.get_blueprint_library()
        spoints = world.get_map().get_spawn_points()
        # spawn vehicle
        vehicle = world.spawn_actor(blib.find("vehicle.tesla.model3"), spoints[0])
        real_gnss = world.spawn_actor(blib.find("sensor.other.gnss"), carla.Transform(), attach_to=vehicle)

        time.sleep(5)

        # test data is correct
        # gnss = CarlaGNSS(uid=1, name="test", base_save_dir=".", parent=None, carla_actor=real_gnss)
        gnss = CarlaGNSS(1, "test", ".", None, real_gnss)
        gnss.save_to_disk(1, 1)
        
        # ...



        

    

if __name__ == "__main__":
    main()