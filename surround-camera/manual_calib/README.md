# manual_calib
This is a manual calibration tool for surround view cameras or surround view fisheye cameras.
## Compile

```shell
# mkdir build
mkdir -p build && cd build
# build
cmake .. && make
```
## Run the test sample:

The executable file is under the bin folder.

```
cd ~./manual_calib/
./bin/run_avm 
or
./bin/run_avm pinhole_samples/imgs pinhole_samples/intrinsic pinhole_samples/extrinsic 2
or
./bin/run_avm ocam_samples/imgs ocam_samples/intrinsic ocam_samples/extrinsic 1
```
The operation method can refer to [the link](https://github.com/PJLab-ADG/SensorsCalibration/blob/master/lidar2camera/README.md).