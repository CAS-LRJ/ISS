from setuptools import Extension
from .algorithms import setup as algorithms_setup
from .evaluation import setup as evluation_setup

setups = [algorithms_setup, evluation_setup]
setup_dirs = ['algorithms', 'evaluation']

extensions = [Extension('runc', ['runc.pyx'])]

for setup_, setup_dir_ in zip(setups, setup_dirs):
    setup_ext = setup_.extensions
    for ext_ in setup_ext:
        ext_.name = setup_dir_ + '.' + ext_.name
        ext_.sources = [setup_dir_ + '/' + source for source in ext_.sources]
        ext_.include_dirs = [setup_dir_ + '/' + include_dir for include_dir in ext_.include_dirs]
        extensions.append(ext_)        