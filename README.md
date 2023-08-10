# ISS

## Build

Run following command to build the project:
```
python setup.py build_ext --inplace
```
We suggest using virtual environment under Windows system. Cython 3.0.0 is currently not supported, so please use Cython 0.29.xx instead. 0.29.{33, 36} are tested.

**Cautious**: Microsoft Visual C++ 14.0 or greater is required. To compile this project locally, you should have *visual-cpp-build-tools* pre-installed.