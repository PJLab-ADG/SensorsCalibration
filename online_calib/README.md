# Introduction
This is a project for online calibration,includeing lidar2imu, camera2imu and radar2carcenter calibration.

# Prerequisites
* Cmake
* opencv2.4
* eigen 3
* PCL 1.9
* Pangolin

## Environment(Quick Start)

```shell
# pull docker image
sudo docker pull scllovewkf/opencalib:latest
# After the image is pulled down, start the docker image.  /home/sz3/ailab/ =  code root path on your host
docker run -it -v /home/sz3/ailab/:/share scllovewkf/opencalib:latest /bin/bash
```

# Compile
Compile in their respective folders

```shell
# mkdir build
mkdir -p build && cd build
#build
cmake .. && make
```

# Camera2imu calibration tool

Five input files:

```shell
 Usage: ./main yaml_car_config_path raw_imu_file video_file video_timestamp_file output_path 
```

* yaml_car_config_path: groundtruth for camera2imu extrinsic.
* raw_imu_fie: imu file
* video_file: vide file
* video_timestamp_file: timestamp
* output_path: result save path

# Lidar2imu calibration tool

Four input files:

```
Usage: ./main lidar_dir novatel_pose_path config_path output_dir
 start_second[default=10s] max_seconds[default=60s]
```

* lidar_dir: lidar pcd files dir
* novatel_pose_path: pose read from novatel
* config_path:  the calibration config file
* output_dir: result save path

# Radar2carcenter calibration tool

Four input files:

```shell
Usage: ./main radar_type radar_dir_path inspva_file output_dir 
```

* radar_type: the radar type
* radar_dir_path:  radar files
* inspva_file: read car pose 
* output_dir: result save path

