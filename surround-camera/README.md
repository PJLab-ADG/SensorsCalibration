# surround-camera_calib
surround-camera_calib is a calibration toolbox for surround view cameras or surround view fisheye cameras, which contains four tools, as shown in the table below.
<!-- CITATION -->

| calibration param |calibration type| calibration method | mannual calibration | auto calibration | usage documentation |
| :--------------: |:--------------:| :------------: | :--------------: | :------------: | :------------: |
| surround_cameras (fisheye) | extrinsic |  target-less    |    &#10004; |             |[manual_calib](manual_calib/README.md)|
| surround_cameras (fisheye) | extrinsic |  target-less    |             |  &#10004;  |[auto_calib_fisheye](auto_calib_fisheye/README.md)|
| surround_cameras            | extrinsic |  target-less   |             |  &#10004;  |[auto_calib](auto_calib/README.md)|
| surround_cameras            | extrinsic |  target        |             |  &#10004;  |[auto_calib_target](auto_calib_target/README.md)|

## Citation
If you find this project useful in your research, please consider cite:
```
@misc{2305.16840,
Author = {Jixiang Li and Jiahao Pi and Guohang Yan and Yikang Li},
Title = {Automatic Surround Camera Calibration Method in Road Scene for Self-driving Car},
Year = {2023},
Eprint = {arXiv:2305.16840},
}
```
