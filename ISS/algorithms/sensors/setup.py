from setuptools import Extension

extensions = [Extension('camera', ['camera.pyx']),
              Extension('carla_camera', ['carla_camera.pyx']),
              Extension('carla_lidar', ['carla_lidar.pyx']),
              Extension('carla_sensor', ['carla_sensor.pyx']),
              Extension('carla_actor', ['carla_actor.pyx'])]
