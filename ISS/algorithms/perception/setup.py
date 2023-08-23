from setuptools import Extension

extensions = [Extension('detection_3d.base', ['detection_3d/base.pyx']),
              Extension('detection_3d.gt', ['detection_3d/gt.pyx'])]

