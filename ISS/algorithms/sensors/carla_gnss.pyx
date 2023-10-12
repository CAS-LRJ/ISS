import carla
import numpy as np

from ISS.algorithms.sensors.sensor import SensorType
from ISS.algorithms.sensors.carla_sensor import CarlaSensor
from ISS.algorithms.utils.dataexchange.sensor.gnss import GNSSOutput

class CarlaGNSS(CarlaSensor):
    def __init__(self, uid, name: str, base_save_dir: str, parent, carla_actor: carla.Sensor):
        super().__init__(uid, name, base_save_dir, parent, carla_actor)
        self.set_stype(SensorType.GNSS)

    def realtime_data(self, sensor_data) -> GNSSOutput:
        return GNSSOutput(sensor_data.altitude, sensor_data.latitude, sensor_data.longitude)

    def save_to_disk_impl(self, save_dir, sensor_data) -> bool:
        gnss_data = np.array([sensor_data.altitude,
                                sensor_data.latitude,
                                sensor_data.longitude])
        # print(gnss_data)
        np.save("{}/{:0>10d}".format(save_dir,
                                    sensor_data.frame),
            gnss_data)
        return True