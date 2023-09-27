import sys
sys.path.append('../')
import lanelet2
from lanelet2.projection import UtmProjector
import numpy as np
import time
import random
import carla
from ISS.algorithms.planning.global_planner.lanelet2_planner import Lanelet2Planner
from ISS.algorithms.planning.local_planner.lattice_planner import LatticePlanner
from ISS.algorithms.utils.vehicleutils.vehicleutils import CollisionChecker
from ISS.algorithms.control.pid.pid import VehiclePIDController
from ISS.algorithms.localization.gt_carla import GroundTruthLocalizationCarla
from ISS.algorithms.perception.detection_3d.gt import Detection3Dgt, Detection3DgtPred
from ISS.algorithms.utils.dataexchange.multiprocessing.manager import DequeManager
from ISS.algorithms.utils.dataexchange.multiprocessing.proxies import DequeProxy
from multiprocessing import Value

def get_solid_checker(loadedMap):
    ## Get Solid Points...
    inset = set()
    solid_points = []
    solid_id = []
    for lanelet in loadedMap.laneletLayer:
        if 'subtype' not in lanelet.attributes or lanelet.attributes['subtype'] != 'road':
            continue
        left_lane = lanelet.leftBound
        right_lane = lanelet.rightBound        
        if "subtype" in left_lane.attributes and left_lane.attributes['subtype'] == 'solid' and left_lane.id not in inset:
            inset.add(left_lane.id)
            for point in left_lane:
                solid_id.append(left_lane.id)
                solid_points.append((point.x, -point.y))

        if "subtype" in right_lane.attributes and right_lane.attributes['subtype'] == 'solid' and right_lane.id not in inset:
            inset.add(right_lane.id)
            for point in right_lane:
                solid_id.append(right_lane.id)
                solid_points.append((point.x, -point.y))
    ##
    solid_checker = CollisionChecker(solid_points, 4.4, 2.2)
    return solid_checker

def test_pid_mp():    
    lanelet2_town06 = 'resources/maps/Town06_hy.osm'
    projector = UtmProjector(lanelet2.io.Origin(0., 0.))
    loadedMap, load_errors = lanelet2.io.loadRobust(lanelet2_town06, projector) 

    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
    
    solid_checker = get_solid_checker(loadedMap)
    
    planner = Lanelet2Planner(loadedMap, traffic_rules, solid_checker, reverse_y=True)
    TURNING_RADIUS = 5.

    client = carla.Client('127.0.0.1', 2000)
    world = client.load_world('Town06')
    spawn_points = world.get_map().get_spawn_points()
    start_point = spawn_points[0]
    start_point = (start_point.location.x, start_point.location.y, np.deg2rad(start_point.rotation.yaw))
    end_point = spawn_points[10]
    end_point = (end_point.location.x, end_point.location.y, np.deg2rad(end_point.rotation.yaw))    
    traj = planner.plan(start_point, end_point, TURNING_RADIUS)
    traj.downsample(0.1)  

    lattice_settings = dict()
    lattice_settings['MAX_SPEED'] = 60.0 / 3.6     # maximum speed [m/s]
    lattice_settings['MAX_ACCEL'] = 4.0            # maximum acceleration [m/ss], tesla model3: 6.88
    lattice_settings['MAX_CURVATURE'] = 1.0      # maximum curvature [1/m], tesla model3's turning radius: 5.8    
    lattice_settings['D_S'] = 0.5                  # sample Frenet d
    lattice_settings['D_ROAD_W'] = 1.0             # road width sampling length [m]
    lattice_settings['DT'] = 1.0                   # prediction timestep length (s)
    lattice_settings['dt'] = 0.5                   # sample time
    lattice_settings['MAX_T'] = 6.0                # max prediction time [s]
    lattice_settings['MIN_T'] = 4.0                # min prediction time [s]
    lattice_settings['TARGET_SPEED'] = 15.0 / 3.6  # target speed [m/s]
    lattice_settings['D_T_S'] = 5.0 / 3.6          # target speed sampling length [m/s]
    lattice_settings['N_S_SAMPLE'] = 2             # sampling number of target speed    
    lattice_settings['ROBOT_RADIUS'] = 2.0         # robot radius [m]
    lattice_settings['K_J'] = 0.1
    lattice_settings['K_T'] = 0.1
    lattice_settings['K_D'] = 1.0
    lattice_settings['K_LAT'] = 1.0
    lattice_settings['K_LON'] = 0.8
    
    det3d_settings = dict()
    det3d_settings["T"] = 8     # prediction horizon [s]
    det3d_settings["dt"] = 0.5  # prediction timestep [s]
    det3d_settings["MAX_DISTANCE"] = 50.0  # maximum distance to consider [m]
    
    spawn_points = world.get_map().get_spawn_points()
    spawn_point_ego = spawn_points[0]
    vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    vehicle = world.spawn_actor(vehicle_bp, spawn_point_ego)      
    ## NPC Vehicles
    tm = client.get_trafficmanager()
    tm.global_percentage_speed_difference(60)
    npcs = []
    choices = np.random.choice(99, 60, replace=False)
    for index in choices:        
        npc_blueprint = random.choice(world.get_blueprint_library().filter('vehicle.*'))
        npcs.append(world.spawn_actor(npc_blueprint, spawn_points[index+1]))
    for npc_vehicle in npcs:
        npc_vehicle.set_autopilot()
    gt_localization = GroundTruthLocalizationCarla(vehicle)
    lattice =  LatticePlanner(loadedMap, traffic_rules, traj.waypoints, lattice_settings, solid_checker)
    det3d = Detection3DgtPred(det3d_settings)
    controller = VehiclePIDController()
    controller.set_goal(end_point)
    DequeManager.register('DequeProxy', DequeProxy, exposed=['__len__', '__getitem__', 'appendleft', 'append', 'pop', 'popleft'])
    manager = DequeManager()    
    manager.start()
    data_proxies = dict()
    location_queue = manager.DequeProxy(maxlen=1)
    local_traj_queue = manager.DequeProxy(maxlen=1)
    obstacle_detector_queue = manager.DequeProxy(maxlen=1)
    control_queue = manager.DequeProxy(maxlen=1)

    terminating_value = Value('i', 1)
    data_proxies['location_queue'] = location_queue
    data_proxies['local_traj_queue'] = local_traj_queue
    data_proxies['obstacle_detector_queue'] = obstacle_detector_queue
    data_proxies['terminating_value'] = terminating_value
    data_proxies['control_queue'] = control_queue
    process_localization = gt_localization.run_proxies(data_proxies, ip='127.0.0.1', port=2000)
    process_lattice = lattice.run_proxies(data_proxies)
    process_det3d = det3d.run_proxies(data_proxies, ip='127.0.0.1', port=2000, vehicle_id=vehicle.id)
    process_pid = controller.run_proxies(data_proxies)
    processes = [process_localization, process_lattice, process_det3d, process_pid]        
    last_time = time.time()     
    ## Set spectator
    spectator = world.get_spectator()
    ## Dummy Camera
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_transform = carla.Transform(carla.Location(x=-10, z=10), carla.Rotation(pitch=-30))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    while terminating_value.value:
        try:
            if vehicle.is_at_traffic_light(): 
                    traffic_light = vehicle.get_traffic_light()
                    if traffic_light.get_state() == carla.TrafficLightState.Red or traffic_light.get_state() == carla.TrafficLightState.Yellow:
                        control = carla.VehicleControl(steer = 0., throttle = 0., brake = 1., hand_brake = True, manual_gear_shift = False)
                        vehicle.apply_control(control)
                        continue
        except:
            pass
        # transform = vehicle.get_transform()
        # spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20), carla.Rotation(yaw=transform.rotation.yaw, pitch=-60)))
        spectator.set_transform(camera.get_transform())
        if time.time() - last_time > 0.5:
            last_time = time.time()
            try:
                tarj = local_traj_queue[-1]
                for point in tarj.waypoints:
                    world.debug.draw_string(carla.Location(point[0], point[1], 0), '*',
                            color=carla.Color(0, 0, 255), life_time=0.5)                                          
                cur_location = location_queue[-1]
                obstacles = obstacle_detector_queue[-1]                
                # collision_point = obstacles.check_point_index((cur_location.x, cur_location.y, cur_location.yaw))                
                # for ind, point in enumerate(obstacles.points):
                #     if ind in collision_point:
                #         world.debug.draw_string(carla.Location(point[0], point[1], 0), '*',
                #             color=carla.Color(255, 0, 0), life_time=0.5)                                          
                #     else:
                #         world.debug.draw_string(carla.Location(point[0], point[1], 0), '*',
                #             color=carla.Color(0, 255, 0), life_time=0.5)                                          
            except Exception as e:
                print(e)
                pass            
        try:
            ## Pass Control Signals to Carla Simulator
            control_ = control_queue[-1]                                                
            control = carla.VehicleControl()
            control.steer = control_.steer
            control.throttle = control_.throttle
            control.brake = control_.brake
            control.hand_brake = control_.hand_brake
            control.manual_gear_shift = control_.manual_gear_shift            
            vehicle.apply_control(control)            
        except:            
            pass
    terminating_value.value = 0
    for process in processes:
        process.join()

test_pid_mp()