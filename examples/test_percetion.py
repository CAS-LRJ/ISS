import time
import numpy as np
from ISS.algorithms.utils.sensorutils.transform import bbox_to_carla_bbox

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

    det3d = Detection3Dgt(None)
    spawn_point_npc = spawn_points[0]
    spawn_point_npc.location.x = spawn_point_npc.location.x - 8.
    spawn_point_npc.rotation.yaw = spawn_point_npc.rotation.yaw + 90    
    vehicle_npc = world.spawn_actor(vehicle_bp, spawn_point_npc) 
    time.sleep(3)
    world.wait_for_tick(3)
    res = det3d.detect(world)
    bottom_points = det3d.build_detector(res).points
    for point in bottom_points:
        world.debug.draw_string(carla.Location(point[0], point[1], 0.), '*',
                            color=carla.Color(255, 0, 0), life_time=1200) 
    for item in res:        
        if item._bbox != None:
            item.debug_print()
            o3d_bbox = item._bbox.to_open3d()
            bbox_vertices = np.asarray(o3d_bbox.get_box_points())            
            for i, point in enumerate(bbox_vertices):
                world.debug.draw_string(carla.Location(point[0], -point[1], point[2]), str(i),
                            color=carla.Color(255, 0, 0), life_time=1200) 
            
            # world.debug.draw_line(carla.Location(bbox_vertices[0][0],-bbox_vertices[0][1],bbox_vertices[0][2]), carla.Location(bbox_vertices[1][0],-bbox_vertices[1][1],bbox_vertices[1][2]), life_time = 1200)             

if __name__ == '__main__':
    test_obstacle_detection_3d_gt()    