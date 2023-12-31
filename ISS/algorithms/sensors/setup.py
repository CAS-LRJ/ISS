from setuptools import Extension

extensions = [Extension('sensor', ['sensor.pyx']),
            #   Extension('beamng_camera', ['beamng_camera.pyx']),
              Extension('carla_camera', ['carla_camera.pyx']),
              Extension('carla_lidar', ['carla_lidar.pyx']),
              Extension('carla_sensor', ['carla_sensor.pyx']),
              Extension('carla_actor', ['carla_actor.pyx']),
              Extension('carla_radar', ['carla_radar.pyx']),
              Extension('carla_gnss', ['carla_gnss.pyx']),
              Extension('carla_vehicle', ['carla_vehicle.pyx']),
              Extension('carla_world', ['carla_world.pyx']),
              Extension('carla_infrastructure', ['carla_infrastructure.pyx']),
              Extension('carla_actor_factory', ['carla_actor_factory.pyx']),
              Extension('carla_actor_tree', ['carla_actor_tree.pyx']),
              ]
