from setuptools import Extension

extensions = [Extension('mathutils.angle', ["mathutils/angle.pyx"]),
              Extension("vehicleutils.vehicleutils", ["vehicleutils/vehicleutils.pyx"]),              
              Extension('dataexchange.detection_2d', ['dataexchange/detection_2d.pyx']),
              Extension('dataexchange.detection_3d', ['dataexchange/detection_3d.pyx']),
              # Extension('dataexchange.detection_lidar', ['dataexchange/detection_lidar.pyx']),
              # Extension('dataexchange.detection_radar', ['dataexchange/detection_radar.pyx']),
              Extension('dataexchange.perception.object_detection', ['dataexchange/perception/object_detection.pyx']),
              Extension('dataexchange.sensor.camera', ['dataexchange/sensor/camera.pyx']),
              Extension('dataexchange.sensor.lidar', ['dataexchange/sensor/lidar.pyx']),
              Extension('dataexchange.sensor.radar', ['dataexchange/sensor/radar.pyx']),
              Extension('sensorutils.transform', ['sensorutils/transform.pyx']),
              Extension('sensorutils.geometry_types', ['sensorutils/geometry_types.pyx']),
              Extension('sensorutils.label_types', ['sensorutils/label_types.pyx']),
              Extension('spconvutils.VoxelGeneratorWrapper', ['spconvutils/VoxelGeneratorWrapper.pyx']),
              Extension('visutils.open3d_vis_utils', ['visutils/open3d_vis_utils.pyx']),]