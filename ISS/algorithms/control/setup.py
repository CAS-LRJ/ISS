from setuptools import Extension

extensions = [
    # Extension("pid", ["pid/pid.pyx"])
    Extension("pid", ["pid/pid.py"])
]
