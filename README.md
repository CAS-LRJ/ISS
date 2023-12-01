# ISS

**Note:** The code is tested in Ubuntu 20.04.

## Install
Anaconda can be used for installation. Run the following command to create a virtual environment:
```
git clone <this repo> && cd <this repo>
conda create --name iss python=3.8
conda activate iss
pip3 install -r requirements_mini.txt
```

## Build

Run the following command to build the project:
```bash
python setup.py build_ext --inplace
```
We suggest using virtual environment under Windows system. Cython 3.0.0 is currently not supported, so please use Cython 0.29.xx instead. 0.29.{33, 36} are tested. For simulators' version control, we are currently using **CARLA 0.9.13** and **BeamNG 0.27.2.0**, therefore you should use **carla==0.9.13** and **beamngpy==1.25.1** in your virtual enviroment.

**Cautious**: Microsoft Visual C++ 14.0 or greater is required. To compile this project locally, you should have *visual-cpp-build-tools* pre-installed.

## Run tasks

### CARLA Data Collector
Run following command to start CARLA 0.9.13 Server:
```
${CARLA_ROOT}/CarlaUE4.sh -prefernvidia
```

Run following command to collect various kinds of sensor data from CARLA:
```bash
python run_carla.py
```
Make sure the CARLA 0.9.13 Server is opened before execution of `run_carla.py`. Data will be saved in `resources/data/carla`, which can be changed by modifying `{RAW_DATA, DATASET}_PATH`.
