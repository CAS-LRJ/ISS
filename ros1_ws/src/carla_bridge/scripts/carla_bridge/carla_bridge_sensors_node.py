#!/usr/bin/env python

import carla
import random
import numpy as np
import random
from scipy.spatial.transform import Rotation as R
import time
import rospy

from ros1_ws.src.carla_bridge.scripts.carla_bridge.object_detector import GTObjectDetector, LAVObjectDetector
from ros1_ws.src.carla_bridge.scripts.carla_bridge.state_estimator import GTStateEstimator, EKFStateEstimator
from carla_bridge.carla_visualizer import CARLAVisualizer
from carla_bridge.controller_bridge import ControllerBridge
from carla_bridge.sensor_utils import add_sensors, SensorInterface
from carla_bridge.route_manipulation import interpolate_trajectory, downsample_route

# from ISS.algorithms.end_to_end.lav.lav_agent import LAVAgent
from ISS.algorithms.end_to_end.lav.lav_agent_fast import LAVAgent

import ISS
import os

ISS_PATH = os.path.dirname(ISS.__file__)

DEBUG_MSGS = True
class CARLABridgeNode:
    def __init__(self, world, traffic_manager):
        self._ego_vehicle_name = rospy.get_param('robot_name', 'ego_vehicle')
        self.params =  {
            "fixed_delta_seconds": rospy.get_param('fixed_delta_seconds'),
            "num_non_ego_vehicles": rospy.get_param('num_non_ego_vehicles'),
            "graphic_rendering": rospy.get_param('graphic_rendering'),
            "simulation_duration": rospy.get_param('simulation_duration'),
            "ego_init": rospy.get_param('ego_init'),
            "ego_destination": rospy.get_param('ego_destination'),
            "agent_control_frequency": rospy.get_param('agent_control_frequency'),
        }
        self._world = world
        self._original_settings = self._world.get_settings()
        self._traffic_manager = traffic_manager
        self._traffic_manager_port = self._traffic_manager.get_port()
        self._traffic_manager.set_random_device_seed(42)
        settings = self._world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.params["fixed_delta_seconds"]
        settings.no_rendering_mode = not self.params["graphic_rendering"]
        self._world.apply_settings(settings)
        self._traffic_manager.set_synchronous_mode(True)
        self._map = self._world.get_map()
        self._spawn_points = self._map.get_spawn_points()
        if DEBUG_MSGS:
            rospy.loginfo("CARLABridgeNode.init: self._spawn_points[1]: " + str(self._spawn_points[1]))
        self._spectator = self._world.get_spectator()
        
        path_to_config = ISS_PATH + "/algorithms/end_to_end/lav/config.yaml"
        self._agent = LAVAgent(path_to_config)
        
        self._vehicles = {}
        self._sensors_list = []
        self._sensor_interface = SensorInterface()
        self._add_vehicles()
        add_sensors(self._vehicles[self._ego_vehicle_name], self._world, self._sensor_interface, self._agent.sensors(), self._sensors_list)
        self._world.tick()
        
        self._object_detector = LAVObjectDetector(self._vehicles[self._ego_vehicle_name].id, self._world)
        self._state_estimator = EKFStateEstimator()
        self._controller_bridge = ControllerBridge(self._vehicles[self._ego_vehicle_name])
        self._carla_visualizer = CARLAVisualizer(self._world)
        self._carla_timer = rospy.Timer(rospy.Duration(self.params["fixed_delta_seconds"]), self._carla_tick)
        self._set_global_plan_perception = False
        self._set_global_plan_planning = False
        self._num_frames = 0
    
    def run(self):
        start_location = self._spawn_points[self.params["ego_init"]].location
        end_location = self._spawn_points[self.params["ego_destination"]].location
        waypoints_trajectory = [start_location, end_location]
        global_plan_gps, global_plan_world_coord = interpolate_trajectory(self._world, waypoints_trajectory)
        self._agent.set_global_plan(global_plan_gps, global_plan_world_coord, downsample_route)
        self._set_global_plan_perception = True
        if self._controller_bridge.start_iss_agent(self._spawn_points[self.params["ego_destination"]]):
            self._set_global_plan_planning = True
            for key, vehicle in self._vehicles.items():
                if key == self._ego_vehicle_name:
                    continue
                vehicle.set_autopilot(True, self._traffic_manager_port)
        
    def _carla_tick(self, event):
        self._set_spectator(self._vehicles[self._ego_vehicle_name].get_transform())
        data = self._sensor_interface.get_data() 
        WARM_UP_FRAMES = 5
        if self._num_frames < WARM_UP_FRAMES: # need to throw away first few frames of data
            self._num_frames += 1
            self._world.tick()
            return
        
        control = self._controller_bridge.get_control()
        MAX_STEER_ANGLE = 40 # TODO
        steering_angle = -control.steer * np.deg2rad(MAX_STEER_ANGLE) # note the negative sign
        
        self._state_estimator.run_step(data, steering_angle)
        if self._set_global_plan_perception:
            control_command, det, other_cast_locs, other_cast_cmds = self._agent.run_step(data, None)
            # if det is not None:
            #     self._carla_visualizer.draw_perception(self._vehicles[self._ego_vehicle_name].get_transform(), det, other_cast_locs, other_cast_cmds)
            if self._set_global_plan_planning:
                if det is None:
                    print("No detection")
                    self._controller_bridge.apply_control(control_command)
                else:
                    ego_transform_matrix = self._vehicles[self._ego_vehicle_name].get_transform().get_matrix()
                    self._object_detector.publish_object_detection(det, ego_transform_matrix)
                    self._object_detector.publish_prediction(other_cast_locs, other_cast_cmds, ego_transform_matrix)
                    self._controller_bridge.apply_control()
                    ### use LAV output for emergency brake
                    # if control_command.brake == 1:
                    #     self._controller_bridge.apply_control(control_command)
                    # else:
                    #     self._controller_bridge.apply_control()
        self._num_frames += 1
        self._world.tick()
            
    def _add_ego_vehicle(self, spawn_point):
        blueprint_library = self._world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.tesla.model3'))
        vehicle = self._world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle != None:
            self._vehicles[self._ego_vehicle_name] = vehicle
        else:
            print("Ego vehicle spawn failed")
    
    def _add_non_ego_vehicle(self, spawn_point, role_name):
        blueprint_library = self._world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
        vehicle = self._world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle != None:
            self._vehicles[role_name] = vehicle
            vehicle.set_autopilot(False, self._traffic_manager_port)

    def _add_vehicles(self):
        ego_spawn_point = self._spawn_points[self.params["ego_init"]]
        nonego_spawn_points = []       
        for p in self._spawn_points:
            if p != ego_spawn_point and \
                np.hypot(p.location.x - ego_spawn_point.location.x,
                         p.location.y - ego_spawn_point.location.y) < 100:
                nonego_spawn_points.append(p)
        self._add_ego_vehicle(ego_spawn_point)        
        for i in range(self.params["num_non_ego_vehicles"]):
            random.shuffle(nonego_spawn_points)
            if i < len(nonego_spawn_points):
                self._add_non_ego_vehicle(nonego_spawn_points[i], "npc_" + str(i))
            else:
                break
            
    def _set_spectator(self, transform):
        new_transform = carla.Transform(transform.location, transform.rotation)
        r = R.from_euler('xyz', [0, 0, new_transform.rotation.yaw], degrees=True)
        loc = r.apply([-15, 0, 20])
        new_transform.location.x += loc[0]
        new_transform.location.y += loc[1]
        new_transform.location.z += loc[2]
        new_transform.rotation.pitch = -40
        self._spectator.set_transform(new_transform)
    
    def destory(self):
        # self._traffic_manager.set_synchronous_mode(False)
        for sensor in self._sensors_list:
            sensor.destroy()
        for vehicle in self._vehicles.values():
            vehicle.destroy()
        self._agent.destroy()
        self._world.apply_settings(self._original_settings)
        self._object_detector.shutdown()
        self._carla_timer.shutdown()

        

if __name__ == "__main__":
    rospy.init_node("carla_bridge_node")
    carla_host = rospy.get_param('carla_host', 'localhost')
    carla_port = rospy.get_param('carla_port', 2000)
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(5.0)
    map_name = rospy.get_param('map_name', 'Town06')
    client.load_world(map_name)
    simulator = CARLABridgeNode(client.get_world(), client.get_trafficmanager())
    rospy.on_shutdown(simulator.destory)
    simulator.run()
    rospy.spin()
