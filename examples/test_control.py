import lanelet2
from ISS.algorithms.planning.global_planner.lanelet2_planner import Lanelet2Planner
from ISS.algorithms.planning.local_planner.lattice_planner import LatticePlanner
from lanelet2.projection import UtmProjector
from ISS.algorithms.utils.vehicleutils.vehicleutils import CollisionChecker
from ISS.algorithms.utils.mathutils.angle import calculate_rot_angle
from ISS.algorithms.control.pid.pid import VehiclePIDController
import numpy as np
import time

def test_pid():
    import carla
    from ISS.algorithms.localization.gt_carla import GroundTruthLocalizationCarla    

    lanelet2_town06 = 'resources/maps/Town06_hy.osm'
    projector = UtmProjector(lanelet2.io.Origin(0., 0.))
    loadedMap, load_errors = lanelet2.io.loadRobust(lanelet2_town06, projector) 

    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
    
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
    
    planner = Lanelet2Planner(loadedMap, traffic_rules, solid_checker, reverse_y=True)
    TURNING_RADIUS = 5.

    client = carla.Client('127.0.0.1', 2000)
    world = client.load_world('Town06')
    # settings = world.get_settings()
    # settings.fixed_delta_seconds = 0.1
    # settings.synchronous_mode = True
    # world.apply_settings(settings)

    spawn_points = world.get_map().get_spawn_points()
    start_point = spawn_points[0]
    start_point = spawn_points[310]
    start_point = (start_point.location.x, start_point.location.y, np.deg2rad(start_point.rotation.yaw))
    end_point = spawn_points[10]
    end_point = spawn_points[98]
    end_point = (end_point.location.x, end_point.location.y, np.deg2rad(end_point.rotation.yaw))
    ## To-DO: Calculate the Rot....
    print(start_point, end_point)
    traj = planner.plan(start_point, end_point, TURNING_RADIUS)
    traj.downsample(0.1)
    # if traj != None:
    #     for point in traj.waypoints:
    #         world.debug.draw_string(carla.Location(point[0], point[1], 0), '*',
    #                     color=carla.Color(0, 255, 0), life_time=1200)           
    
    spawn_points = world.get_map().get_spawn_points()
    spawn_point_ego = spawn_points[0]   
    spawn_point_ego = spawn_points[310] 
    vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    vehicle = world.spawn_actor(vehicle_bp, spawn_point_ego)    

    # spawn_point_npc = spawn_points[0]
    # spawn_point_npc.location.x = spawn_point_npc.location.x - 5.
    # vehicle_npc = world.spawn_actor(vehicle_bp, spawn_point_npc)   

    controller = VehiclePIDController()
    locator = GroundTruthLocalizationCarla(vehicle)
    time.sleep(5)
    controller.set_traj(traj)
    lattice_settings = dict()
    lattice_settings['MAX_SPEED'] = 30.0 / 3.6     # maximum speed [m/s]
    lattice_settings['MAX_ACCEL'] = 4.0            # maximum acceleration [m/ss], tesla model3: 6.88
    lattice_settings['MAX_CURVATURE'] = 1.0      # maximum curvature [1/m], tesla model3's turning radius: 5.8    
    lattice_settings['D_S'] = 0.1                  # sample Frenet d
    lattice_settings['D_ROAD_W'] = 1.0             # road width sampling length [m]
    lattice_settings['DT'] = 1.0                   # prediction timestep length (s)
    lattice_settings['dt'] = 0.5                   # sample time
    lattice_settings['MAX_T'] = 6.0                # max prediction time [s]
    lattice_settings['MIN_T'] = 4.0                # min prediction time [s]
    lattice_settings['TARGET_SPEED'] = 15.0 / 3.6  # target speed [m/s]
    lattice_settings['D_T_S'] = 5.0 / 3.6          # target speed sampling length [m/s]
    lattice_settings['N_S_SAMPLE'] = 2             # sampling number of target speed
    lattice_settings['N_S_SAMPLE'] = 1             # sampling number of target speed    
    lattice_settings['ROBOT_RADIUS'] = 2.0         # robot radius [m]
    lattice_settings['K_J'] = 0.1
    lattice_settings['K_T'] = 0.1
    lattice_settings['K_D'] = 1.0
    lattice_settings['K_LAT'] = 1.0
    lattice_settings['K_LON'] = 0.8
    lattice_planner = LatticePlanner(loadedMap, traffic_rules, traj.waypoints, lattice_settings, solid_checker)
    while True:
        vehicle_location = locator.run_step()
        state_cartesian = (vehicle_location.x, vehicle_location.y, vehicle_location.yaw, vehicle_location.velocity, vehicle_location.acceleration)        
        traj_local = lattice_planner.run_step(state_cartesian)
        if traj_local != None and len(traj_local.waypoints) > 0:
            for point in traj_local.waypoints:
                world.debug.draw_string(carla.Location(point[0], point[1], 0), '*',
                            color=carla.Color(255, 0, 0), life_time=0.3)                
            controller.set_traj(traj_local)
        control_ = controller.run_step(vehicle_location)
        control = carla.VehicleControl()
        control.steer = control_.steer
        control.throttle = control_.throttle
        control.brake = control_.brake
        control.hand_brake = control_.hand_brake
        control.manual_gear_shift = control_.manual_gear_shift
        vehicle.apply_control(control)
        # world.tick()
        spec = world.get_spectator()
        spec_trans = vehicle.get_transform()
        ego_vehicle_location = vehicle.get_transform().location
        ego_vehicle_rotation = vehicle.get_transform().rotation
        offset = carla.Vector3D(x=-20.0, y=0, z=50)        
        left_rear_location = ego_vehicle_location + ego_vehicle_rotation.get_right_vector() * offset.y + \
                             ego_vehicle_rotation.get_forward_vector() * offset.x + \
                             ego_vehicle_rotation.get_up_vector() * offset.z
        rotation_reset = carla.Rotation(pitch=-45.0,
                                        yaw=ego_vehicle_rotation.yaw,
                                        roll=ego_vehicle_rotation.roll
                                        )
        spectator_transform = carla.Transform(left_rear_location, rotation_reset)
        spec.set_transform(spectator_transform)
        # time.sleep(0.1)

def test_control_data():
    from ISS.algorithms.utils.dataexchange.control.vehiclecontrol import VehicleControl
    data = VehicleControl()
    import pickle
    print(len(pickle.dumps(data)), 'bytes')

if __name__ == '__main__':
    # test_control_data()
    test_pid()