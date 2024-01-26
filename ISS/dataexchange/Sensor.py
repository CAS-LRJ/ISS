import numpy as np

class ISSSensor:
    def __init__(self, name, sensor_type, sensor_id, sensor_data):
        self.name = name
        self.sensor_type = sensor_type
        self.sensor_id = sensor_id
        # sensor_data no need to be initialized in base class
        # self.sensor_data = sensor_data

    def get_name(self):
        return self.name

    def get_sensor_type(self):
        return self.sensor_type

    def get_sensor_id(self):
        return self.sensor_id

    def get_sensor_data(self):
        return self.sensor_data

    def set_name(self, name):
        self.name = name

    def set_sensor_type(self, sensor_type):
        self.sensor_type = sensor_type

    def set_sensor_id(self, sensor_id):
        self.sensor_id = sensor_id

    def set_sensor_data(self, sensor_data):
        self.sensor_data = sensor_data

    def __str__(self):
        return "Sensor: " + self.name + ", " + self.sensor_type + ", " + self.sensor_id + ", " + self.sensor_data

class ISSCamera(ISSSensor):
    def __init__(self, name, sensor_type, sensor_id, sensor_data, camera_type, camera_model, camera_resolution, camera_fov, camera_fps):
        super().__init__(name, sensor_type, sensor_id, sensor_data)
        self.camera_type = camera_type
        self.camera_model = camera_model
        self.camera_resolution = camera_resolution
        self.camera_fov = camera_fov
        self.camera_fps = camera_fps

    def get_camera_type(self):
        return self.camera_type

    def get_camera_model(self):
        return self.camera_model

    def get_camera_resolution(self):
        return self.camera_resolution

    def get_camera_fov(self):
        return self.camera_fov

    def get_camera_fps(self):
        return self.camera_fps

    def set_camera_type(self, camera_type):
        self.camera_type = camera_type

    def set_camera_model(self, camera_model):
        self.camera_model = camera_model

    def set_camera_resolution(self, camera_resolution):
        self.camera_resolution = camera_resolution

    def set_camera_fov(self, camera_fov):
        self.camera_fov = camera_fov

    def set_camera_fps(self, camera_fps):
        self.camera_fps = camera_fps

    def __str__(self):
        return super().__str__() + ", " + self.camera_type + ", " + self.camera_model + ", " + self.camera_resolution + ", " + self.camera_fov + ", " + self.camera_fps