from setuptools import Extension

extensions = [Extension('mathutils.angle', ["mathutils/angle.pyx"]),
              Extension("vehicleutils.vehicleutils", ["vehicleutils/vehicleutils.pyx"]),              
              Extension('dataexchange.detection_2d', ['dataexchange/detection_2d.pyx']),
              Extension('sensorutils.transform', ['sensorutils/transform.pyx']),
              Extension('sensorutils.geometry_types', ['sensorutils/geometry_types.pyx']),
              Extension('sensorutils.label_types', ['sensorutils/label_types.pyx']),]