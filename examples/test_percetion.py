import time

def test_obstacle_detection_3d_gt():
    import carla
    from ISS.algorithms.perception.detection_3d.gt import Detection3Dgt
    det3d = Detection3Dgt()

    client = carla.Client('127.0.0.1', 2000)
    world = client.load_world('Town06')        
    ## To-DO: Calculate the Rot....        
    
    spawn_points = world.get_map().get_spawn_points()
    spawn_point_ego = spawn_points[0]       
    vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    vehicle = world.spawn_actor(vehicle_bp, spawn_point_ego)    
    time.sleep(3)
    res = det3d.detect(None, world)
    for item in res:
        # print(item._label item._bbox)
        if item._bbox != None:
            item.debug_print()

if __name__ == '__main__':
    test_obstacle_detection_3d_gt()    