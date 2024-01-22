from leaderboard.autoagents.ros1_agent import ROS1Agent

def get_entry_point():
    return 'ISSLeaderboardAgent'


class ISSLeaderboardAgent(ROS1Agent):
        
    def get_ros_entrypoint(self):
        return {
            "package": "iss_manager",
            "launch_file": "iss_manager.launch",
        }
        
    def sensors(self):
        # parameters hardcoded for LAV stack
        CAMERA_YAWS = [-60,0,60]
        camera_x = 1.5
        camera_z = 2.4
        
        sensors = [
            {'type': 'sensor.speedometer', 'id': 'EGO'},
            {'type': 'sensor.other.gnss', 'x': 0., 'y': 0., 'z': camera_z, 'id': 'GPS'},
            {'type': 'sensor.other.imu',  'x': 0., 'y': 0., 'z': camera_z, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,'sensor_tick': 0.05, 'id': 'IMU'},
            
        ]

        # Add LiDAR
        sensors.append({
            'type': 'sensor.lidar.ray_cast', 'x': 0.0, 'y': 0.0, 'z': camera_z, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 
            'id': 'LIDAR'
        })

        # Add cameras
        for i, yaw in enumerate(CAMERA_YAWS):
            sensors.append({'type': 'sensor.camera.rgb', 'x': camera_x, 'y': 0.0, 'z': camera_z, 'roll': 0.0, 'pitch': 0.0, 'yaw': yaw,
            'width': 256, 'height': 288, 'fov': 64, 'id': f'RGB_{i}'})

        sensors.append({'type': 'sensor.camera.rgb', 'x': camera_x, 'y': 0.0, 'z': camera_z, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'width': 480, 'height': 288, 'fov': 40, 'id': 'TEL_RGB'})


        return sensors
    