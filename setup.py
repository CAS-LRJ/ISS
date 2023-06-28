from setuptools import Extension, setup
from Cython.Build import cythonize
import ISS.setup as ISS_setup
import numpy as np

setups = [ISS_setup]
setup_dirs = ['ISS']

extensions = []

include_dirs = ['ISS/algorithms/planning/dubins/include']

for setup_, setup_dir_ in zip(setups, setup_dirs):
    setup_ext = setup_.extensions
    for ext_ in setup_ext:
        ext_.name = setup_dir_ + '.' + ext_.name
        ext_.sources = [setup_dir_ + '/' + source for source in ext_.sources]
        ext_.include_dirs = [setup_dir_ + '/' + include_dir for include_dir in ext_.include_dirs]
        extensions.append(ext_)


for ext_ in extensions:
    print(ext_.name, ext_.sources, ext_.include_dirs)
    ext_.include_dirs.append(np.get_include())
    ext_.include_dirs += include_dirs
    if not ext_.language:
        ext_.language = 'c++'
    ext_.extra_compile_args = ["/O2"]    


compiler_directives = {"language_level": 3, "embedsignature": True}
setup(
    name="ISS",
    ext_modules=cythonize(extensions, compiler_directives=compiler_directives),
    # extra_compile_args=["/O3"],
)