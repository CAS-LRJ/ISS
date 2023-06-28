from setuptools import Extension

extensions = [Extension('ground_truth.localization_bbox', ["ground_truth/localization_bbox.pyx"]),
              Extension('ground_truth.localization_offset', ["ground_truth/localization_offset.pyx"])]              