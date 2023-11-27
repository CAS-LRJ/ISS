import carla
import random
import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
from tqdm import tqdm

from carla_bridge.gt_object_detector import GTObjectDetector
from carla_bridge.gt_state_estimator import GTStateEstimator
from carla_agent.behavior_agent import BehaviorAgent
from iss_msgs.msg import ControlCommand
from iss_msgs.srv import SetGoal

class CARLABridgeNode:
    def __init__(self, world, traffic_manager):
        self.params =  {
            "fixed_delta_seconds": rospy.get_param('~fixed_delta_seconds', 0.05),
            "num_non_ego_vehicles": rospy.get_param('~num_non_ego_vehicles', 10),
            "graphic_rendering": rospy.get_param('~graphic_rendering', True),
            "simulation_duration": rospy.get_param('~simulation_duration', 10),
            "simple_agent_demo": rospy.get_param('~simple_agent_demo', True),
            "ego_init": rospy.get_param('~ego_init', 1),
            "ego_destination": rospy.get_param('~ego_destination', 2),
            "agent_control_frequency": rospy.get_param('~agent_control_frequency', 10),
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
        self._spectator = self._world.get_spectator()
        self._vehicles = {}
        self._add_vehicles()
        self._world.tick()
        self._set_spectator(self._vehicles["ego_vehicle"].get_transform())
        self._control = carla.VehicleControl()

    def run(self):
        if self.params["simple_agent_demo"]:
            self._agent = BehaviorAgent(self._vehicles["ego_vehicle"], behavior='normal')
            self._agent.set_destination(self._spawn_points[self.params["ego_destination"]].location)
            self._agent_timer = rospy.Timer(rospy.Duration(1 / self.params["agent_control_frequency"]), self._agent_tick)
        else:
            self._agent_sub = rospy.Subscriber("carla_bridge/control_command", ControlCommand, self._agent_sub_callback)
            self._call_set_goal_srv(self._spawn_points[self.params["ego_destination"]])
        self._gt_object_detector = GTObjectDetector(self._vehicles["ego_vehicle"].id, self._world)
        self._gt_state_estimator = GTStateEstimator(self._vehicles["ego_vehicle"])
        rospy.loginfo("Simulation started!")
        self._carla_timer = rospy.Timer(rospy.Duration(self.params["fixed_delta_seconds"]), self._carla_tick)
        self._total_steps = int(self.params["simulation_duration"] / self.params["fixed_delta_seconds"])
        self._progress_bar = tqdm(total=self.params["simulation_duration"] + 0.1, unit="sec")
        self._step_cnt = 0
        rospy.spin()
    
    def _call_set_goal_srv(self, goal):
        rospy.wait_for_service('planning/set_goal')
        try:
            set_goal = rospy.ServiceProxy('planning/set_goal', SetGoal)
            resp = set_goal(goal.location.x, -goal.location.y, -np.deg2rad(goal.rotation.yaw))
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False
    
    def _agent_tick(self, event):
        self._control = self._agent.run_step()

    def _agent_sub_callback(self, msg):
        self._control.steer = min(max(msg.steer, -1.0), 1.0)
        if msg.throtto < 0:
            self._control.throttle = 0
            self._control.brake = min(-msg.throtto, 1.0) / 2
        else:
            self._control.throttle = min(msg.throtto, 1.0)
            self._control.brake = 0
        
    def _carla_tick(self, event):
        self._progress_bar.update(self.params["fixed_delta_seconds"])
        self._step_cnt += 1
        
        self._vehicles["ego_vehicle"].apply_control(self._control)
        self._world.tick()
        if self._step_cnt >= self._total_steps:
            self._gt_object_detector.shutdown()
            self._gt_state_estimator.shutdown()
            self._carla_timer.shutdown()
            self._agent_timer.shutdown() if self.params["simple_agent_demo"] else None
            self._progress_bar.close()
            self.destory()
            self._world.tick()
            rospy.signal_shutdown("Simulation finished!")
            
    def _add_ego_vehicle(self, spawn_point):
        blueprint_library = self._world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.tesla.model3'))
        vehicle = self._world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle != None:
            self._vehicles["ego_vehicle"] = vehicle
        else:
            print("Ego vehicle spawn failed")
    
    def _add_non_ego_vehicle(self, spawn_point, role_name):
        blueprint_library = self._world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
        vehicle = self._world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle != None:
            self._vehicles[role_name] = vehicle
            vehicle.set_autopilot(True, self._traffic_manager_port)
    
    def _add_vehicles(self):
        self._add_ego_vehicle(self._spawn_points[self.params["ego_init"]])
        for i in range(self.params["num_non_ego_vehicles"]):
            spawn_point = random.choice(self._spawn_points)
            self._add_non_ego_vehicle(spawn_point, "non_ego_vehicle_" + str(i))
    
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
        for vehicle in self._vehicles.values():
            vehicle.destroy()
        self._world.tick()
        self._world.apply_settings(self._original_settings)
            

if __name__ == "__main__":
    rospy.init_node("carla_bridge_node")
    carla_host = rospy.get_param('~carla_host', 'localhost')
    carla_port = rospy.get_param('~carla_port', 2000)
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(5.0)
    map_name = rospy.get_param('~map_name', 'Town06')
    client.load_world(map_name)
    simulator = CARLABridgeNode(client.get_world(), client.get_trafficmanager())
    simulator.run()