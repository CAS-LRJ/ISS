from setuptools import Extension

extensions = [Extension('Carla.carla_collector', ['Carla/carla_collector.pyx'])]
