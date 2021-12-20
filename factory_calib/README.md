## Introduction

This is a project for factory calibration. Factory calibration is usually the last process in vehicle production. The calibration equipment is mainly composed of multiple calibration boards and four-wheel positioning. After the four-wheel positioning of the vehicle is finished, the calibration starts and the pose relationship between the sensors is calculated through the calibration board. We provide six types of calibration boards, the calibration board recognition program removes the Opencv dependency. The recognition performance is very high in different environments. 

## Prerequisites

- Cmake
- opencv 2.4
- eigen 3
- PCL 1.9

## Compile

```shell
# mkdir build
mkdir -p build && cd build
# build
cmake .. && make
```

## Calibration board recognition

| calibration board type  | original image | recognition result | 
| :--------------: | :------------------------------------------------------------------: | :------------------------------------------------------------------: | 
| chessboard        | <img src="./images/chessboard.jpg" width="100%" height="100%" div align=center> | <img src="./images/chessboard_detection.png" width="100%" height="100%" div align=center> |    
| circle board      | <img src="./images/circle_board.jpg" width="100%" height="100%" div align=center>  | <img src="./images/circleboard_detection.png" width="100%" height="100%" div align=center> |  
| vertical board    | <img src="./images/vertical_board.jpg" width="100%" height="100%" div align=center>| <img src="./images/verticalboard_detection.png" width="100%" height="100%" div align=center> |  
| apriltag board    | <img src="./images/april_board.jpg" width="100%" height="100%" div align=center>   | <img src="./images/apriltags_detection.png" width="100%" height="100%" div align=center> |   
| aruco marker board| <img src="./images/arucomarker.jpg" width="100%" height="100%" div align=center>   | <img src="./images/arucomarker_detection.png" width="100%" height="100%" div align=center> |   
| round hole board  | <img src="./images/round_hole.png" width="100%" height="100%" div align=center>      | <img src="./images/round_hole_detection.png" width="100%" height="100%" div align=center> |   

**Note:** Apriltag recognition code is derived from the open-source project [kalibr](https://github.com/ethz-asl/kalibr/tree/master/aslam_offline_calibration/ethz_apriltag2).

## Run
run command:
```shell
# run board detection
./bin/run_board_detect image board_type
# run round hole board detection
./bin/run_lidar_detect pcds
# run extrinsic calibration example
./bin/run_extrinsic_calib
# run camera homography matrix and vanishing point calibration example
./bin/run_homo_vp_calib image board_type output_dir
```
