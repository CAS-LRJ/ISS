import carla
import rospy
import numpy as np
from iss_manager.msg import State, StateArray, ObjectDetection3DArray

color_map = { 
    'red': carla.Color(255, 0, 0),
    'green': carla.Color(0, 255, 0),
    'blue': carla.Color(0, 0, 255),
    'yellow': carla.Color(255, 255, 0),
}

def rotate(loc, yaw):
    loc_array = np.array(loc)
    rot_mat = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    loc_rotated = rot_mat @ loc_array
    return loc_rotated

class CARLAVisualizer:
    def __init__(self, world) -> None:
        self._world = world
        self._global_planner_sub = rospy.Subscriber("planning/global_planner/trajectory", StateArray, self._global_planner_callback)
        self._local_planner_sub = rospy.Subscriber("planning/local_planner/trajectory", StateArray, self._local_planner_callback)
        self._object_detection_sub = rospy.Subscriber(rospy.get_param("object_detection_topic"), ObjectDetection3DArray, self._object_detection_callback)
        self._state_estimation_sub = rospy.Subscriber(rospy.get_param("state_estimation_topic"), State, self._state_estimation_callback)
    
    def _global_planner_callback(self, msg):
        self._draw_trajectory_carla(msg, life_time=100, z=0.5, color=color_map['green'])
    
    def _local_planner_callback(self, msg):
        self._draw_trajectory_carla(msg, life_time=0.1, z=0.5, color=color_map['blue'])
    
    def _draw_trajectory_carla(self, msg, life_time, z=0.5, color=carla.Color(255, 0, 0)):
        # trajectory: StateArray
        for i, state in enumerate(msg.states):
            loc = carla.Location(x=state.x, y=-state.y, z=z)  # note: carla y is opposite to rviz y
            self._world.debug.draw_string(loc, str(i), life_time=life_time, color=color)
    
    def _object_detection_callback(self, msg):
        for detection in msg.detections:
            box = carla.BoundingBox(carla.Location(x=detection.state.x, y=-detection.state.y, z=0.1), carla.Vector3D(x=detection.bbox.size.x, y=detection.bbox.size.y, z=detection.bbox.size.z))
            rot = carla.Rotation(pitch=0, yaw=-np.rad2deg(detection.state.heading_angle), roll=0)
            self._world.debug.draw_box(box, rot, color=color_map['red'], life_time=0.1, thickness=0.05)
    
    def _state_estimation_callback(self, msg):
        vehicle_length = rospy.get_param("vehicle_info")["length"]
        vehicle_width = rospy.get_param("vehicle_info")["width"]
        box = carla.BoundingBox(carla.Location(x=msg.x, y=-msg.y, z=0.1), carla.Vector3D(x=vehicle_length*0.7, y=vehicle_width*0.7, z=1.5))
        rot = carla.Rotation(pitch=0, yaw=-np.rad2deg(msg.heading_angle), roll=0)
        self._world.debug.draw_box(box, rot, color=color_map['yellow'], life_time=0.1, thickness=0.05)
        
    
    def draw_perception(self, ego_trans, det, other_cast_locs, other_cast_cmds):
        duration = 0.05
        meter_per_pixel = 1 / 4
        
        ego_trans_matrix = ego_trans.get_matrix()
        
        # draw detection
        for x, y, ww, hh, cos, sin in det[1]:
            x = (x - 160) * meter_per_pixel
            y = (y - 280) * meter_per_pixel
            loc_rotated = rotate((x, y), -np.pi/2)
            rot_cos = sin
            rot_sin = -cos
            loc_vec = np.array([-loc_rotated[0], -loc_rotated[1], 0, 1])
            loc_transformed = ego_trans_matrix @ loc_vec
            ww = ww * meter_per_pixel
            hh = hh * meter_per_pixel
            height = 1.5
            
            # box = carla.BoundingBox(carla.Location(x=loc_transformed[0], y=loc_transformed[1], z=loc_transformed[2]), carla.Vector3D(x=ww, y=hh, z=height))
            # rot = carla.Rotation(pitch=0, yaw=0, roll=0)
            # self._world.debug.draw_box(box, rot, color=color_map['yellow'], life_time=duration, thickness=10)
            
            p1 = tuple((loc_transformed.tolist()[:2] + [-ww,-hh]@np.array([[-rot_sin,rot_cos],[-rot_cos,-rot_sin]])))
            p2 = tuple((loc_transformed.tolist()[:2] + [-ww, hh]@np.array([[-rot_sin,rot_cos],[-rot_cos,-rot_sin]])))
            p3 = tuple((loc_transformed.tolist()[:2] + [ ww, hh]@np.array([[-rot_sin,rot_cos],[-rot_cos,-rot_sin]])))
            p4 = tuple((loc_transformed.tolist()[:2] + [ ww,-hh]@np.array([[-rot_sin,rot_cos],[-rot_cos,-rot_sin]])))
            base_corners = [p1, p2, p3, p4]
            base_locations = [carla.Location(x=float(corner[0]), y=float(corner[1]), z=0) for corner in base_corners]
            top_locations = [carla.Location(x=float(corner[0]), y=float(corner[1]), z=height) for corner in base_corners]

            for i in range(4):
                self._world.debug.draw_string(base_locations[i], str(i), life_time=duration, color=color_map['green'])

            for i in range(len(base_corners)):
                self._world.debug.draw_line(base_locations[i], base_locations[(i+1)%len(base_corners)], life_time=duration, color=color_map['green'])
                self._world.debug.draw_line(base_locations[i], top_locations[i], life_time=duration, color=color_map['green'])
                self._world.debug.draw_line(top_locations[i], top_locations[(i+1)%len(base_corners)], life_time=duration, color=color_map['green'])


        # draw prediction
        # ego_trans_matrix = np.linalg.inv(ego_trans_matrix)
        cmd_thresh = 0.2
        for trajs, cmds in zip(other_cast_locs, other_cast_cmds):
            for traj, score in zip(trajs, cmds):
                if score < cmd_thresh:
                    continue
                
                for idx, loc in enumerate(traj):
                    loc_rotated = rotate(loc, -np.pi/2)
                    loc_vec = np.array([-loc_rotated[0], -loc_rotated[1], 0.5, 1])
                    loc_transformed = ego_trans_matrix @ loc_vec
                    self._world.debug.draw_string(carla.Location(x=loc_transformed[0], y=loc_transformed[1], z=loc_transformed[2]), str(idx), life_time=duration, color=color_map['red'])
            


