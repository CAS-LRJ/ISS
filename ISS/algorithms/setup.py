from setuptools import Extension
from .sensors import setup as sensors_setup
from .control import setup as control_setup
from .localization import setup as localization_setup
from .mapping import setup as mapping_setup
from .orientation import setup as orientation_setup
from .perception import setup as perception_setup
from .planning import setup as planning_setup
from .prediction import setup as prediction_setup
from .utils import setup as utils_setup

setups = [sensors_setup, control_setup, localization_setup , mapping_setup , orientation_setup , perception_setup , planning_setup , prediction_setup, utils_setup]
setup_dirs = ['sensors', 'control', 'localization', 'mapping', 'orientation', 'perception', 'planning', 'prediction', 'utils']

extensions = []

for setup_, setup_dir_ in zip(setups, setup_dirs):
    setup_ext = setup_.extensions
    for ext_ in setup_ext:
        ext_.name = setup_dir_ + '.' + ext_.name
        ext_.sources = [setup_dir_ + '/' + source for source in ext_.sources]
        ext_.include_dirs = [setup_dir_ + '/' + include_dir for include_dir in ext_.include_dirs]
        extensions.append(ext_)