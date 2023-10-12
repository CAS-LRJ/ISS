import argparse
import json
import os
import signal
import time
import open3d
import yaml
from easydict import EasyDict
from pathlib import Path

import carla

import ISS
from ISS.algorithms.utils.dataexchange.detection_3d import Detection3DOutput, Detection3DInput
from ISS.algorithms.sensors.carla_vehicle import CarlaVehicle, CarlaOtherVehicle
from ISS.algorithms.sensors.carla_actor_factory import CarlaNodeType, CarlaNode
from ISS.algorithms.sensors.carla_actor_tree import CarlaActorTree
from ISS.algorithms.utils.sensorutils.transform import Transform, Location, Rotation
from ISS.algorithms.utils.sensorutils.transform import transform_to_carla_transform
from ISS.algorithms.perception.torchscriptwrapper import torchScriptWrapper

# Project Root Path
ROOT_PATH = Path(__file__).parent.as_posix()

RAW_DATA_PATH = "{}/resources/data/carla/{}".format(ROOT_PATH, 'raw_data')
DATASET_PATH = "{}/resources/data/carla/{}".format(ROOT_PATH, 'dataset')

DATASET_CONFIG = "{}/config/{}".format(ROOT_PATH, "model_cfgs/kitti_models/pointpillar.yaml")

sig_interrupt = False


def signal_handler(signal, frame):
    global sig_interrupt
    sig_interrupt = True


class DataCollector:
    def __init__(self, args):
        self.host = args.host
        self.port = args.port
        self.carla_client = carla.Client(self.host, self.port)
        self.carla_client.set_timeout(10.0)
        self.world = self._get_world()
        self.tm = self.carla_client.get_trafficmanager()
        self.debug_helper = self.world.debug
        self.record_name = None
        self.base_save_dir = None
        self.world_config_file = "{}/config/{}".format(ROOT_PATH, args.world_config_file)
        self.actor_tree = CarlaActorTree(self.world)
        self.frame_total = -1
        self.frame_step = 1

    def _get_world(self) -> carla.World:
        return self.carla_client.get_world()

    def destroy(self):
        self.actor_tree.destroy()
        
    def cfg_from_yaml_file(self, cfg_file):
        with open(cfg_file, 'r') as f:
            try:
                config = yaml.safe_load(f, Loader=yaml.FullLoader)
            except:
                config = yaml.safe_load(f)

        return EasyDict(config)


    def set_traffic_light_time(self, traffic_light_setting):
        actor_list = self.world.get_actors()
        for actor in actor_list:
            if isinstance(actor, carla.TrafficLight):
                actor.set_red_time(traffic_light_setting["red_time"])
                actor.set_green_time(traffic_light_setting["green_time"])
                actor.set_yellow_time(traffic_light_setting["yellow_time"])

    def setting_world_and_actors(self, json_file):
        with open(json_file) as handle:
            json_settings = json.loads(handle.read())
            self.carla_client.load_world(json_settings["map"])
            settings = self.world.get_settings()
            settings.synchronous_mode = True

            if json_settings["spectator_pose"] is not None:
                pose = json_settings["spectator_pose"]
                spectator = self.world.get_spectator()
                spectator_transform = Transform(Location(pose["x"], pose["y"], pose["z"]),
                                                Rotation(roll=pose["roll"], pitch=pose["pitch"], yaw=pose["yaw"]))
                spectator.set_transform(transform_to_carla_transform(spectator_transform))

            # Make sure fixed_delta_seconds <= max_substep_delta_time * max_substeps
            world_settings = json_settings["world_settings"]
            settings.fixed_delta_seconds = world_settings["fixed_delta_seconds"]
            settings.substepping = True
            settings.max_substep_delta_time = world_settings["max_substep_delta_time"]
            settings.max_substeps = world_settings["max_substeps"]
            self.world.apply_settings(settings)
            self.tm.set_synchronous_mode(True)

            self.frame_total = json_settings["frame_total"]
            self.frame_step = json_settings["frame_step"]

            self.set_traffic_light_time(json_settings["traffic_light_setting"])

            actor_config_file = json_settings["actor_settings"]
            self.record_name = time.strftime("%Y_%m%d_%H%M", time.localtime())
            self.base_save_dir = "{}/record_{}".format(RAW_DATA_PATH, self.record_name)
            self.actor_tree = CarlaActorTree(self.world,
                                        "{}/config/{}".format(ROOT_PATH,
                                                              actor_config_file),
                                        self.base_save_dir)
            self.actor_tree.init()
            
    def get_vehicle_node_by_name(self, name:str, root:CarlaNode=None) -> CarlaNode:
        res = None
        if root is None:
            root = self.actor_tree.root
        
        for node in self.dfs_search_node(CarlaNodeType.VEHICLE, root):
            if node.get_actor().name == name:
                res = node
                break
            
        return res
            
    def dfs_search_node(self, type: CarlaNodeType, root:CarlaNode=None) -> CarlaNode:
        if root is None:
            root = self.actor_tree.root
        
        if root.get_node_type() == type:
            yield root
        
        for child in root.get_children():
            for res in self.dfs_search_node(type, child):
                if res is not None:
                    yield res


    def start_record(self):
        self.setting_world_and_actors(self.world_config_file)
        os.makedirs(self.base_save_dir, exist_ok=True)
        carla_logfile = "{}/carla_raw_record.log".format(self.base_save_dir)
        self.carla_client.start_recorder(carla_logfile)
        
        primary_player = self.get_vehicle_node_by_name("tesla3_player")
        if primary_player is not None:
            lidar_sensor = primary_player.get_children()[0] 
            
        vis = open3d.visualization.Visualizer()
        vis.create_window()
        vis_view_control = vis.get_view_control()
        # vis_view_control.convert_from_pinhole_camera_parameters
        open3d.io.read_pinhole_camera_trajectory("./config/vis_config/open3d_config.json")
        # vis_view_control.set_lookat([ 34.5, 0, 4.75 ])
        # vis_view_control.set_zoom(0.7)
        # vis_view_control.change_field_of_view(60.0)
        # vis_view_control.set_front([ -0.90, 0.0018, 0.43 ])
        # vis_view_control.set_up([ 0.43, 0.007, 0.90 ])
        # vis_view_control.set_constant_z_far(70)
        dataset_cfg = self.cfg_from_yaml_file(DATASET_CONFIG)
        det_in = Detection3DInput(dataset_cfg.DATA_CONFIG)
        det_out = Detection3DOutput(class_names = ["null", 'Car', 'Pedestrian', 'Cyclist'], confidence_threshold = [1.0, 0.4, 0.5, 0.7])
        torch_model = torchScriptWrapper(path = "./resources/models/detection_3d/point_pillar_model.pt")
        torch_model.load_model()

        try:
            total_frame_count = 0
            while True:
                # print("----------")
                # Tick Control
                self.actor_tree.tick_controller()
                # Tick World
                tick_s = time.time()
                frame_id = self.world.tick(seconds=60.0)
                world_snapshot = self.world.get_snapshot()
                timestamp = world_snapshot.timestamp.elapsed_seconds

                if total_frame_count % self.frame_step == 0:
                    det_in.from_carla_lidar(lidar_sensor.get_actor())
                    det_out.from_output(det_in, torch_model)
                    det_out.preview_scene(vis)
                    
                    # {
                    #     "class_name" : "ViewTrajectory",
                    #     "interval" : 29,
                    #     "is_loop" : false,
                    #     "trajectory" : 
                    #     [
                    #         {
                    #             "boundingbox_max" : [ 69.118263244628906, 39.679920196533203, 16.415634155273438 ],
                    #             "boundingbox_min" : [ -0.059999999999999998, -39.679874420166016, -6.9146575927734375 ],
                    #             "field_of_view" : 60.0,
                    #             "front" : [ -0.90307097537632919, 0.0017988087570628851, 0.42948757574567964 ],
                    #             "lookat" : [ 34.529131622314452, 2.288818359375e-05, 4.75048828125 ],
                    #             "up" : [ 0.42948904059539766, 0.0070563614983622357, 0.90304450154510629 ],
                    #             "zoom" : 0.69999999999999996
                    #         }
                    #     ],
                    #     "version_major" : 1,
                    #     "version_minor" : 0
                    # }
                    
                    vis.poll_events()
                    vis.update_renderer()
                    vis.clear_geometries()


                global sig_interrupt
                if sig_interrupt:
                    print("Exit step, wait 2 seconds...")
                    time.sleep(2.0)
                    break

                total_frame_count += 1
                if total_frame_count >= self.frame_total:
                    time.sleep(2.0)
                    break


        except KeyboardInterrupt:
            print("User interrupt, exit...")
        else:
            print("Unhandled error: reload the world and exit...")
        self.destroy()
        self.carla_client.reload_world()


def main():
    print("Project Root PATH: {}".format(ROOT_PATH))
    signal.signal(signal.SIGINT, signal_handler)
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-w', '--world_config_file',
        metavar='W',
        default='world_config_template.json',
        type=str,
        help='World configuration file')
    args = argparser.parse_args()
    data_collector = DataCollector(args)
    data_collector.start_record()


if __name__ == "__main__":
    # execute only if run as a script
    main()
