## Introduction
This project is used to calibrate yaw angle from radar to car.
## Environment
```shell
docker pull xiaokyan/opencalib:v1
```
## Compile
```shell
# mkdir build
mkdir -p build && cd build
# build
cmake .. && make
```
## Data
We uploaded a small amount of data for testing.
```
Link(链接): https://pan.baidu.com/s/1B9M-M6LT5UukJmCAdzu9Mw
Extraction code(提取码): sc3i 
```
## Run example
```shell
./bin/run_radar2car conti data/conti/front_radar/ data/conti/novatel_enu.csv
./bin/run_radar2car delphi data/delphi/front_radar data/delphi/novatel_enu.csv 20 500
```
- input: radar data file and vehicle pose data file

