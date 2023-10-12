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
        vehicle = world.spawn_actor(blib.find("vehicle.tesla.model3"), spoints[0])
        gnss = world.spawn_actor(blib.find("sensor.other.gnss"), carla.Transform(), attach_to=vehicle)

        time.sleep(5)

        # test data is correct
        gnss_data = CarlaGNSS(carla_actor=gnss)
        output = gnss_data.realtime_data()
        print(output)
    

if __name__ == "__main__":
    main()

# sig_interrupt = False


# def signal_handler(signal, frame):
#     global sig_interrupt
#     sig_interrupt = True


# class DataCollector:
#     def __init__(self, args):
#         self.host = args.host
#         self.port = args.port
#         self.carla_client = carla.Client(self.host, self.port)
#         self.carla_client.set_timeout(10.0)
#         self.world = self._get_world()
#         self.tm = self.carla_client.get_trafficmanager()
#         self.debug_helper = self.world.debug
#         self.record_name = None
#         self.base_save_dir = None
#         self.world_config_file = "{}/config/{}".format(ROOT_PATH, args.world_config_file)
#         self.actor_tree = CarlaActorTree(self.world)
#         self.frame_total = -1
#         self.frame_step = 1

#     def _get_world(self) -> carla.World:
#         return self.carla_client.get_world()

#     def destroy(self):
#         self.actor_tree.destroy()

#     def set_traffic_light_time(self, traffic_light_setting):
#         actor_list = self.world.get_actors()
#         for actor in actor_list:
#             if isinstance(actor, carla.TrafficLight):
#                 actor.set_red_time(traffic_light_setting["red_time"])
#                 actor.set_green_time(traffic_light_setting["green_time"])
#                 actor.set_yellow_time(traffic_light_setting["yellow_time"])

#     def setting_world_and_actors(self, json_file):
#         with open(json_file) as handle:
#             json_settings = json.loads(handle.read())
#             self.carla_client.load_world(json_settings["map"])
#             settings = self.world.get_settings()
#             settings.synchronous_mode = True

#             if json_settings["spectator_pose"] is not None:
#                 pose = json_settings["spectator_pose"]
#                 spectator = self.world.get_spectator()
#                 spectator_transform = Transform(Location(pose["x"], pose["y"], pose["z"]),
#                                                 Rotation(roll=pose["roll"], pitch=pose["pitch"], yaw=pose["yaw"]))
#                 spectator.set_transform(transform_to_carla_transform(spectator_transform))

#             # Make sure fixed_delta_seconds <= max_substep_delta_time * max_substeps
#             world_settings = json_settings["world_settings"]
#             settings.fixed_delta_seconds = world_settings["fixed_delta_seconds"]
#             settings.substepping = True
#             settings.max_substep_delta_time = world_settings["max_substep_delta_time"]
#             settings.max_substeps = world_settings["max_substeps"]
#             self.world.apply_settings(settings)
#             self.tm.set_synchronous_mode(True)

#             self.frame_total = json_settings["frame_total"]
#             self.frame_step = json_settings["frame_step"]

#             self.set_traffic_light_time(json_settings["traffic_light_setting"])

#             actor_config_file = json_settings["actor_settings"]
#             self.record_name = time.strftime("%Y_%m%d_%H%M", time.localtime())
#             self.base_save_dir = "{}/record_{}".format(RAW_DATA_PATH, self.record_name)
#             self.actor_tree = CarlaActorTree(self.world,
#                                         "{}/config/{}".format(ROOT_PATH,
#                                                               actor_config_file),
#                                         self.base_save_dir)
#             self.actor_tree.init()

#     def start_record(self):
#         self.setting_world_and_actors(self.world_config_file)
#         os.makedirs(self.base_save_dir, exist_ok=True)
#         carla_logfile = "{}/carla_raw_record.log".format(self.base_save_dir)
#         self.carla_client.start_recorder(carla_logfile)
#         try:
#             total_frame_count = 0
#             while True:
#                 print("----------")
#                 # Tick Control
#                 self.actor_tree.tick_controller()
#                 # Tick World
#                 tick_s = time.time()
#                 frame_id = self.world.tick(seconds=60.0)
#                 world_snapshot = self.world.get_snapshot()
#                 timestamp = world_snapshot.timestamp.elapsed_seconds
#                 print("World Tick -> FrameID: {} Timestamp: {} Cost: {:.3f}s".format(frame_id,
#                                                                                      timestamp,
#                                                                                      time.time()-tick_s))
#                 # Save data to disk
#                 if total_frame_count % self.frame_step == 0:
#                     save_s = time.time()
#                     self.actor_tree.tick_data_saving(frame_id, timestamp)
#                     print("Raw data saved, cost {:.3f}s".format(time.time()-save_s))

#                 global sig_interrupt
#                 if sig_interrupt:
#                     print("Exit step, wait 2 seconds...")
#                     time.sleep(2.0)
#                     break

#                 total_frame_count += 1
#                 if total_frame_count >= self.frame_total:
#                     time.sleep(2.0)
#                     break

#         except KeyboardInterrupt:
#             print("User interrupt, exit...")
#         else:
#             print("Unhandled error: reload the world and exit...")
#         self.destroy()
#         self.carla_client.reload_world()


# def main():
#     print("Project Root PATH: {}".format(ROOT_PATH))
#     signal.signal(signal.SIGINT, signal_handler)
#     argparser = argparse.ArgumentParser(description=__doc__)
#     argparser.add_argument(
#         '--host',
#         metavar='H',
#         default='127.0.0.1',
#         help='IP of the host server (default: 127.0.0.1)')
#     argparser.add_argument(
#         '-p', '--port',
#         metavar='P',
#         default=2000,
#         type=int,
#         help='TCP port to listen to (default: 2000)')
#     argparser.add_argument(
#         '-w', '--world_config_file',
#         metavar='W',
#         default='world_config_template.json',
#         type=str,
#         help='World configuration file')
#     args = argparser.parse_args()
#     data_collector = DataCollector(args)
#     data_collector.start_record()


# if __name__ == "__main__":
#     # execute only if run as a script
#     main()
