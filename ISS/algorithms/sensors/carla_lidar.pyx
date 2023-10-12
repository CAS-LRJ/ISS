#!/usr/bin/python3

import carla
import numpy as np

from ISS.algorithms.sensors.carla_sensor import CarlaSensor
from ISS.algorithms.sensors.sensor import SensorType
from ISS.algorithms.utils.dataexchange.sensor.lidar import LiDAROutput, LiDARSegOutput


class CarlaLiDAR(CarlaSensor):
    def __init__(self, uid, name: str, base_save_dir: str, parent, carla_actor: carla.Sensor):
        super().__init__(uid, name, base_save_dir, parent, carla_actor)
        self.set_stype(SensorType.LIDAR)

    def realtime_data(self, sensor_data) -> LiDAROutput:
        # Save as a Nx4 numpy array. Each row is a point (x, y, z, intensity)
        lidar_data = np.copy(np.frombuffer(bytes(sensor_data.raw_data),
                                   dtype=np.float32))
        lidar_data = np.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))

        # Convert point cloud to right-hand coordinate system
        lidar_data[:, 1] *= -1
        lidar_output = LiDAROutput(lidar_data)
        return lidar_output

    def save_to_disk_impl(self, save_dir, sensor_data) -> bool:
        # Save as a Nx4 numpy array. Each row is a point (x, y, z, intensity)
        lidar_data = np.fromstring(bytes(sensor_data.raw_data),
                                   dtype=np.float32)
        lidar_data = np.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))

        # Convert point cloud to right-hand coordinate system
        lidar_data[:, 1] *= -1

        # Save point cloud to [RAW_DATA_PATH]/.../[ID]_[SENSOR_TYPE]/[FRAME_ID].npy
        np.save("{}/{:0>10d}".format(save_dir,
                                     sensor_data.frame),
                lidar_data)
        return True


class CarlaSemanticLiDAR(CarlaSensor):
    def __init__(self, uid, name: str, base_save_dir: str, parent, carla_actor: carla.Sensor):
        super().__init__(uid, name, base_save_dir, parent, carla_actor)
        self.set_stype(SensorType.SEMANTICLIDAR)

    def realtime_data(self, sensor_data) -> LiDARSegOutput:
        # Save data as a Nx6 numpy array.
        lidar_data = np.frombuffer(bytes(sensor_data.raw_data),
                                   dtype=np.dtype([
                                       ('x', np.float32),
                                       ('y', np.float32),
                                       ('z', np.float32),
                                       ('CosAngle', np.float32),
                                       ('ObjIdx', np.uint32),
                                       ('ObjTag', np.uint32)
                                   ]))

        # Convert point cloud to right-hand coordinate system
        lidar_data['y'] *= -1
        
        lidar_output = LiDARSegOutput(lidar_data)
        return lidar_output

    def save_to_disk_impl(self, save_dir, sensor_data) -> bool:
        # Save data as a Nx6 numpy array.
        lidar_data = np.fromstring(bytes(sensor_data.raw_data),
                                   dtype=np.dtype([
                                       ('x', np.float32),
                                       ('y', np.float32),
                                       ('z', np.float32),
                                       ('CosAngle', np.float32),
                                       ('ObjIdx', np.uint32),
                                       ('ObjTag', np.uint32)
                                   ]))

        # Convert point cloud to right-hand coordinate system
        lidar_data['y'] *= -1

        # Save point cloud to [RAW_DATA_PATH]/.../[ID]_[SENSOR_TYPE]/[FRAME_ID].npy
        np.save("{}/{:0>10d}".format(save_dir,
                                     sensor_data.frame),
                lidar_data)
        return True
