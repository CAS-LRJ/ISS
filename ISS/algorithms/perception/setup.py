from setuptools import Extension

extensions = [Extension('groundtruth.gt', ['groundtruth/gt.pyx'])]

