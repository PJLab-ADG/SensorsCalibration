## Introduction

This is a project for intrinsic calibration and evalution.

It mainly includes two part: intrinsic calibration, distortion measurement. 

## Prerequisites

- Cmake
- opencv 2.4
- eigen 3

## Compile
Compile in their respective folders

```shell
# mkdir build
mkdir -p build && cd build
# build
cmake .. && make
```

## Input data
- <calibration_image_dir>: contains only seleted chessboard calibration image
- <distortion_image_path>: distortion harp image
 
## Run
run command:
```shell
# run intrinsic calibration
./bin/run_intrinsic_calibration <calibration_image_dir>
# run distortion measurement
./bin/run_distortion_measure <distortion_image_path>
```

- **Intrinsic Calibration:** Program will automatically run intrinsic calibration. Corner-detect result will be displayed. All input calibration images will be undistorted and save to `<calibration_image_dir>/undistort/` dir.

- **Distortion Evaluation:** Sampled points of original and undistorted images will be displayed. Undistorted distortion_image will be save to `<output_dir>`.
