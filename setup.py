from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext as _build_ext
from Cython.Build import cythonize
from torch.utils.cpp_extension import BuildExtension, CUDAExtension
import ISS.setup as ISS_setup
import numpy as np
import os
from copy import deepcopy

def make_cuda_ext(name, module, sources):
    cuda_ext = CUDAExtension(
        name='%s.%s' % (module, name),
        sources=[os.path.join(*module.split('.'), src) for src in sources]
    )
    return cuda_ext

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

## Make include_dirs absolute. Check https://github.com/pytorch/pytorch/issues/37707
cwd = os.getcwd()
for ext_ in extensions:
    print(ext_.name, ext_.sources, ext_.include_dirs)
    ext_.include_dirs.append(np.get_include())
    ext_.include_dirs += include_dirs
    ext_.include_dirs = [os.path.join(cwd, include_dir) for include_dir in ext_.include_dirs]
    if not ext_.language:
        ext_.language = 'c++'
    if os.name == 'nt':
        ext_.extra_compile_args = ["/O2"]    
    elif os.name == 'posix':
        ext_.extra_compile_args = ["-O3"]

cuda_modules = [
        make_cuda_ext(
                name='iou3d_nms_cuda',
                module='ISS.algorithms.utils.postprocessingutils.iou3d',
                sources=[                    
                    'src/iou3d_cpu.cpp',
                    'src/iou3d_nms_api.cpp',
                    'src/iou3d_nms.cpp',
                    'src/iou3d_nms_kernel.cu',
                ]
            )
]

compiler_directives = {"language_level": 3, "embedsignature": True}
cython_modules = cythonize(extensions, compiler_directives=compiler_directives)

setup(
    name="ISS",
    ext_modules=cython_modules + cuda_modules,
    cmdclass={
            'build_ext': BuildExtension#.with_options(use_ninja=False),
    },
    # extra_compile_args=["/O3"],
)