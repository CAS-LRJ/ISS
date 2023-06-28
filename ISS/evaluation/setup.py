from setuptools import Extension
from .attack import setup as attack_setup
from .testing import setup as testing_setup
from .verification import setup as verification_setup

setups = [attack_setup, testing_setup, verification_setup]
setup_dirs = ['attack', 'testing', 'verification']

extensions = []

for setup_, setup_dir_ in zip(setups, setup_dirs):
    setup_ext = setup_.extensions
    for ext_ in setup_ext:
        ext_.name = setup_dir_ + '.' + ext_.name
        ext_.sources = [setup_dir_ + '/' + source for source in ext_.sources]
        ext_.include_dirs = [setup_dir_ + '/' + include_dir for include_dir in ext_.include_dirs]
        extensions.append(ext_)