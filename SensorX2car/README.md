# SensorX2car
SensorX2car is a calibration toolbox for the online calibration of sensor-to-car coordinate systems in road scenes, which contains four commonly used sensors (pose_sensor (IMU + GNSS), LiDAR (Light Detection and Ranging), Camera, and millimeter-wave Radar) for autonomous driving. For more calibration codes, please refer to the link <a href="https://github.com/PJLab-ADG/SensorsCalibration" title="SensorsCalibration">SensorsCalibration</a>
<!-- CITATION -->

| calibration param |calibration type| calibration method | mannual calibration | auto calibration | usage documentation |
| :--------------: |:--------------:| :------------: | :--------------: | :------------: | :------------: |
| camera2car      | extrinsic |  target-less   |    &#10004; |  &#10004;  |[camera2car](camera2car/README.md)|
| lidar2car       | extrinsic |  target-less   |             |  &#10004;  |[lidar2car](lidar2car/README.md)|
| pose_sensor2car | extrinsic |  target-less   |             |  &#10004;  |[pose_sensor2car](pose_sensor2car/README.md)|
| radar2car       | extrinsic |  target-less   |             |  &#10004;  |[radar2car](radar2car/README.md)|

## Citation
If you find this project useful in your research, please consider cite:
```
@misc{2301.07279,
Author = {Guohang Yan and Zhaotong Luo and Zhuochun Liu and Yikang Li},
Title = {SensorX2car: Sensors-to-car calibration for autonomous driving in road scenarios},
Year = {2023},
Eprint = {arXiv:2301.07279},
}
```

## Contact
If you have questions about this repo, please contact Yan Guohang (`yanguohang@pjlab.org.cn`). 
