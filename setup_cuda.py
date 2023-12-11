from setuptools import setup
from torch.utils.cpp_extension import BuildExtension, CUDAExtension
import os

def make_cuda_ext(name, module, sources):
    cuda_ext = CUDAExtension(
        name='%s.%s' % (module, name),
        sources=[os.path.join(*module.split('.'), src) for src in sources]
    )
    return cuda_ext

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

setup(
    name="ISS",
    ext_modules=cuda_modules,
    cmdclass={
            'build_ext': BuildExtension,
    },
    extra_compile_args=["/O3"],
)