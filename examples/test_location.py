import time
from ISS.algorithms.utils.dataexchange.multiprocessing.manager import DequeManager
from ISS.algorithms.utils.dataexchange.multiprocessing.proxies import DequeProxy
from multiprocessing.sharedctypes import Value

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

def test_gt_carla_mp_proxy():
    import carla
    from ISS.algorithms.localization.gt_carla import GroundTruthLocalizationCarla
    client = carla.Client('127.0.0.1', 2000)    
    world = client.load_world('Town06')
    spawn_points = world.get_map().get_spawn_points()
    spawn_point_ego = spawn_points[0]
    vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    vehicle = world.spawn_actor(vehicle_bp, spawn_point_ego)
    vehicle.set_autopilot()
    # world.tick()
    # world.wait_for_tick()
    gt_carla = GroundTruthLocalizationCarla(vehicle)
    DequeManager.register('DequeProxy', DequeProxy, exposed=['__len__', '__getitem__', 'appendleft', 'append', 'pop', 'popleft'])
    manager = DequeManager()
    # data_proxies = DequeManager.dict()    
    manager.start()
    data_proxies = dict()
    location_queue = manager.DequeProxy(maxlen=1)
    terminating_value = Value('i', 1)
    data_proxies['location_queue'] = location_queue
    data_proxies['terminating_value'] = terminating_value
    process = gt_carla.run_proxies(data_proxies, ip='127.0.0.1', port=2000)
    count = 0
    time.sleep(5)
    t_end = time.time() + 60 * 15
    while time.time() < t_end:
        # world.wait_for_tick()
        # print(vehicle.get_transform())
        try:
            res = location_queue[-1]
            print(res.x, res.y, res.yaw)
            count+=1
        except:
            # print('Not Set...')
            pass
    terminating_value.value = 0
    process.join()

    pass

if __name__ == '__main__':
    # test_gt_carla()
    test_gt_carla_mp_proxy()
