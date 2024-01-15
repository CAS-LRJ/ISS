import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from leaderboard.envs.sensor_interface import CallBack, OpenDriveMapReader, SpeedometerReader, SensorInterface
from leaderboard.autoagents.autonomous_agent import Track


def setup_sensors(vehicle, world, sensor_interface, sensor_specs, sensors_list):
    """
    Create the sensors defined by the user and attach them to the ego-vehicle
    :param vehicle: ego vehicle
    :return:
    """
    bp_library = world.get_blueprint_library()
    for sensor_spec in sensor_specs:
        # These are the pseudosensors (not spawned)
        if sensor_spec['type'].startswith('sensor.opendrive_map'):
            # The HDMap pseudo sensor is created directly here
            sensor = OpenDriveMapReader(vehicle, sensor_spec['reading_frequency'])
        elif sensor_spec['type'].startswith('sensor.speedometer'):
            delta_time = world.get_settings().fixed_delta_seconds
            frame_rate = 1 / delta_time
            sensor = SpeedometerReader(vehicle, frame_rate)
        # These are the sensors spawned on the carla world
        else:
            bp = bp_library.find(str(sensor_spec['type']))
            if sensor_spec['type'].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(sensor_spec['width']))
                bp.set_attribute('image_size_y', str(sensor_spec['height']))
                bp.set_attribute('fov', str(sensor_spec['fov']))
                bp.set_attribute('lens_circle_multiplier', str(3.0))
                bp.set_attribute('lens_circle_falloff', str(3.0))
                bp.set_attribute('chromatic_aberration_intensity', str(0.5))
                bp.set_attribute('chromatic_aberration_offset', str(0))

                sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                    z=sensor_spec['z'])
                sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                    roll=sensor_spec['roll'],
                                                    yaw=sensor_spec['yaw'])
            elif sensor_spec['type'].startswith('sensor.lidar'):
                bp.set_attribute('range', str(85))
                bp.set_attribute('rotation_frequency', str(10))
                bp.set_attribute('channels', str(64))
                bp.set_attribute('upper_fov', str(10))
                bp.set_attribute('lower_fov', str(-30))
                bp.set_attribute('points_per_second', str(600000))
                bp.set_attribute('atmosphere_attenuation_rate', str(0.004))
                bp.set_attribute('dropoff_general_rate', str(0.45))
                bp.set_attribute('dropoff_intensity_limit', str(0.8))
                bp.set_attribute('dropoff_zero_intensity', str(0.4))
                sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                    z=sensor_spec['z'])
                sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                    roll=sensor_spec['roll'],
                                                    yaw=sensor_spec['yaw'])
            elif sensor_spec['type'].startswith('sensor.other.radar'):
                bp.set_attribute('horizontal_fov', str(sensor_spec['fov']))  # degrees
                bp.set_attribute('vertical_fov', str(sensor_spec['fov']))  # degrees
                bp.set_attribute('points_per_second', '1500')
                bp.set_attribute('range', '100')  # meters

                sensor_location = carla.Location(x=sensor_spec['x'],
                                                    y=sensor_spec['y'],
                                                    z=sensor_spec['z'])
                sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                    roll=sensor_spec['roll'],
                                                    yaw=sensor_spec['yaw'])

            elif sensor_spec['type'].startswith('sensor.other.gnss'):
                bp.set_attribute('noise_alt_stddev', str(0.000005))
                bp.set_attribute('noise_lat_stddev', str(0.000005))
                bp.set_attribute('noise_lon_stddev', str(0.000005))
                bp.set_attribute('noise_alt_bias', str(0.0))
                bp.set_attribute('noise_lat_bias', str(0.0))
                bp.set_attribute('noise_lon_bias', str(0.0))

                sensor_location = carla.Location(x=sensor_spec['x'],
                                                    y=sensor_spec['y'],
                                                    z=sensor_spec['z'])
                sensor_rotation = carla.Rotation()

            elif sensor_spec['type'].startswith('sensor.other.imu'):
                bp.set_attribute('noise_accel_stddev_x', str(0.001))
                bp.set_attribute('noise_accel_stddev_y', str(0.001))
                bp.set_attribute('noise_accel_stddev_z', str(0.015))
                bp.set_attribute('noise_gyro_stddev_x', str(0.001))
                bp.set_attribute('noise_gyro_stddev_y', str(0.001))
                bp.set_attribute('noise_gyro_stddev_z', str(0.001))

                sensor_location = carla.Location(x=sensor_spec['x'],
                                                    y=sensor_spec['y'],
                                                    z=sensor_spec['z'])
                sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                    roll=sensor_spec['roll'],
                                                    yaw=sensor_spec['yaw'])
            # create sensor
            sensor_transform = carla.Transform(sensor_location, sensor_rotation)
            sensor = world.spawn_actor(bp, sensor_transform, vehicle)
        # setup callback
        sensor.listen(CallBack(sensor_spec['id'], sensor_spec['type'], sensor, sensor_interface))
        sensors_list.append(sensor)