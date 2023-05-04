## Introduction
A project for LiDAR to car calibration. It calibrates the three rotation angles and the height of lidar to the ground.

## Environment
```shell
docker pull xiaokyan/opencalib:v1
```
if remote visualization is not set up, uncomment this sentence in file "src/LidarYawCalib.cpp" to make matplotlib run in the background.
```
plt::backend("Agg");
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
Link(链接): https://pan.baidu.com/s/1r9DcEuvz7IZ_rGpYo_WBBw
Extraction code(提取码): 5m2x
```
## Run example
```shell
./bin/run_lidar2car ./data/example/ ./output/
```
## Input
- <dataset_folder>: contain lidar files
- <output_dir>: save output file
## Output
- calib_result.txt: calibration result
- pose.txt: lidar pose result
- trajectory.png: lidar trajectory
- compared_yaw.png: comparison between lidar pose yaw and trajectory yaw, which can be used to verify the result roughly
## Note
It is recommended that the vehicle drives in a straight line for better yaw angle estimation and drives on flatter ground to get better pitch and roll estimation.
