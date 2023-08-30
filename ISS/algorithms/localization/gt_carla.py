import carla
import math
from ISS.algorithms.utils.dataexchange.localization.transforms import VehicleTransform

class GroundTruthLocalizationCarla(object):

    def __init__(self, vehicle) -> None:
        self.vehicle = vehicle
    
    def run_step(self):
        vehicletransform = VehicleTransform()
        carla_transform = self.vehicle.get_transform()
        vehicle_location = carla_transform.location
        vehicle_rotation = carla_transform.rotation
        vehicle_velocity = self.vehicle.get_velocity()
        vehicle_acceleration = self.vehicle.get_acceleration()
        vehicletransform.x = vehicle_location.x
        vehicletransform.y = vehicle_location.y
        vehicletransform.z = vehicle_location.z
        vehicletransform.yaw = math.radians(vehicle_rotation.yaw)
        vehicletransform.pitch = vehicle_rotation.pitch
        vehicletransform.roll = vehicle_rotation.roll
        vehicletransform.acceleration_x = vehicle_acceleration.x
        vehicletransform.acceleration_y = vehicle_acceleration.y
        vehicletransform.acceleration_z = vehicle_acceleration.z
        vehicletransform.velocity_x = vehicle_velocity.x
        vehicletransform.velocity_y = vehicle_velocity.y
        vehicletransform.velocity_z = vehicle_velocity.z
        vehicletransform.velocity = math.hypot(vehicle_velocity.x, vehicle_velocity.y, vehicle_velocity.z)
        vehicletransform.acceleration = math.hypot(vehicle_acceleration.x, vehicle_acceleration.y, vehicle_acceleration.z)
        return vehicletransform