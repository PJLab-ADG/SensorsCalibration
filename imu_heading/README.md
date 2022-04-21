## Introduction

This is a project for imu heading angle calibration and evalution.

<img src="./images/imu_heading.png" width="50%" height="50%" alt="checkerboard" div align=center /><br>

## Prerequisites

- Cmake
- eigen 3

## Compile
Compile in their respective folders

```shell
# mkdir build
mkdir -p build && cd build
# build
cmake .. && make
```

## Calibration data collection

According to the figure below for data collection, the calibration vehicle records IMU and GPS data in a straight line.

<img src="./images/data_collect.png" width="100%" height="100%" alt="data collection" div align=center /><br>

## Run
run command:
```shell
# run imu heading calibration
./bin/run_imu_heading method_id <data_dir>
```
- **Imu heading calibration:** The imu heading angle is obtained by the registration calculation of GPS trajectory and imu data.

- **Imu heading evaluation:** Speed projection verification of imu through the calibrated heading angle of imu.
