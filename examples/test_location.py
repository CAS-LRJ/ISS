import time

def test_gt_carla():
    import carla
    from ISS.algorithms.localization.gt_carla import GroundTruthLocalizationCarla
    client = carla.Client('127.0.0.1', 2000)
    world = client.load_world('Town06')
    spawn_points = world.get_map().get_spawn_points()
    spawn_point_ego = spawn_points[0]
    vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    vehicle = world.spawn_actor(vehicle_bp, spawn_point_ego)
    vehicle.set_target_velocity(carla.Vector3D(2., 0., 0.))
    time.sleep(5)
    gt_carla = GroundTruthLocalizationCarla(vehicle)
    res = gt_carla.run_step()
    for item in res.__dict__:
        print(item, res.__dict__[item])

if __name__ == '__main__':
    test_gt_carla()
