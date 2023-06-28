from setuptools import Extension

extensions = [
    Extension("fast_lidar_mapping.fast_lidar_mapping", ["./fast_lidar_mapping/fast_lidar_mapping.pyx"])
]
