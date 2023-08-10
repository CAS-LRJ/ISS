# ISS

## Build

Run following command to build the project:
```
python setup.py build_ext --inplace
```
We suggest using virtual environment under Windows system. Cython 3.0.0 is currently not supported, so please use Cython 0.29.xx instead. 0.29.{33, 36} are tested. For simulators' version control, we are currently using **CARLA 0.9.13** and **BeamNG 0.27.2.0**, therefore you should use **carla==0.9.13** and **beamngpy==1.25.1** in your virtual enviroment.

**Cautious**: Microsoft Visual C++ 14.0 or greater is required. To compile this project locally, you should have *visual-cpp-build-tools* pre-installed.