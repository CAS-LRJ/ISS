import time

def test_obstacle_detection_3d_gt():
    import carla
    from ISS.algorithms.perception.detection_3d.gt import Detection3Dgt    

    client = carla.Client('127.0.0.1', 2000)
    world = client.load_world('Town06')        
    ## To-DO: Calculate the Rot....        
    
    spawn_points = world.get_map().get_spawn_points()
    spawn_point_ego = spawn_points[0]       
    vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    vehicle = world.spawn_actor(vehicle_bp, spawn_point_ego)    

    det3d = Detection3Dgt(world, vehicle)
    spawn_point_npc = spawn_points[0]
    spawn_point_npc.location.x = spawn_point_npc.location.x - 8.
    spawn_point_npc.rotation.yaw = spawn_point_npc.rotation.yaw + 90
    print(spawn_point_npc)
    vehicle_npc = world.spawn_actor(vehicle_bp, spawn_point_npc) 
    # time.sleep(3)
    world.wait_for_tick(3)
    res = det3d.detect()
    for item in res:
        # print(item._label item._bbox)
        if item._bbox != None:
            item.debug_print()
            print(item._bbox.proj_2d_points())
            for point in item._bbox.proj_2d_points():
                print(point[0], point[1], point[2])
                world.debug.draw_string(carla.Location(point[0], point[1], point[2]), '*',
                            color=carla.Color(255, 0, 0), life_time=1200)  

if __name__ == '__main__':
    test_obstacle_detection_3d_gt()    