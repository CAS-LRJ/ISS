import numpy as np
from ISS.algorithms.mapping.fast_lidar_mapping import fast_obstacle_detection
import cv2
from test_hybrid_a_star_config import config
import pickle
import math
from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Lidar
import os
import time
ego_pos = np.array((-709., -1342., 141.))

USERPATH = fr'{os.getenv("LOCALAPPDATA")}/BeamNG.tech'   
beamng = BeamNGpy('localhost', 64256, user=USERPATH)
bng = beamng.open(launch=True, extensions=['util/roadData'])
scenario = Scenario('italy', 'Map Mod Test')
vehicle = Vehicle('ego_vehicle', model='vivace', license='AI', extensions=['mapPath'], partConfig='vehicles/vivace/tograc_qE_Unbreakable.pc')
scenario.add_vehicle(vehicle, pos=(-709, -1342, 141), rot_quat=(0, 0, -0.82, 0.55))
scenario.make(bng)
bng.settings.set_deterministic(60) # Set simulator to 60hz temporal resolution
bng.scenario.load(scenario)
bng.scenario.start()
lidar = Lidar('lidar1', bng, vehicle, requested_update_time=0.01, vertical_resolution=128, vertical_angle=54, is_using_shared_memory=True, is_visualised=False)

def coords_transform(ego_pos, point):
    point_x = int(math.floor((point[0] - ego_pos[0]) / 0.1))
    point_y = int(math.floor((point[1] - ego_pos[1]) / 0.1))


## Test Italy Map
with open('resources/maps/italy_roadmap.pickle', 'rb') as f:
    roadmap = pickle.load(f)
road_map = roadmap['kdtree']
road_point = roadmap['points']
road_height = roadmap['heights']

while True:
    frame = np.zeros((640, 640))
    vehicle.sensors.poll()
    # ego_pos = np.array(vehicle.state['pos'])
    ego_pos = np.asarray(lidar.get_position())
    data = lidar.poll()
    # with open('resources/data/italy_pointcloud_example.pickle', 'rb') as f:
    #     data = pickle.load(f)    
    obstacles = fast_obstacle_detection(data['pointCloud'].astype('float64').reshape(-1,3), ego_pos, -65536., 65536., config)    
    ego_pos = ego_pos[:2]
    map_index = road_map.query_ball_point((ego_pos[0], ego_pos[1]), 32, p=float('inf'), return_sorted=True)
    obstacles = obstacles - ego_pos + 32.
    for obs_ in obstacles:
        point_x = int(math.floor(obs_[0] / 0.1))
        point_y = int(math.floor(obs_[1] / 0.1))    
        if point_x >= 0 and point_x <= 640 and point_y >=0 and point_y <= 640:
            frame[point_x, point_y] = 255
    for id_ in map_index:        
        point_x = int(math.floor((road_point[id_] - ego_pos + 32.)[0] / 0.1))
        point_y = int(math.floor((road_point[id_] - ego_pos + 32.)[1] / 0.1))    
        if point_x >= 0 and point_x <= 640 and point_y >=0 and point_y <= 640:
            frame[point_x, point_y] = 255

    cv2.imshow('image', frame)
    cv2.waitKey(1)
    time.sleep(0.1)