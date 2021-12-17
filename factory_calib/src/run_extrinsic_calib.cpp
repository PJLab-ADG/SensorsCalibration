/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#include "Eigen/Core"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "dataConvert.hpp"
#include "logging.hpp"

#include "aruco_marker/corner_detect.hpp"
#include "calibration/pnp_solver.hpp"
#include "round_hole_board/lidar_pattern.h"

/**
 * @brief An example of extrinsic parameter calibration
 * NOTE: If the translation of the extrinsic parameter is known and very
 * accurate, you can only optimize the rotation, so that the accuracy of the
 * solved external parameter is higher.
 *
 */

int main(int argc, char **argv) {
  if (argc > 1) {
    std::cerr << argv[0];
    return -1;
  }

  bool detection_success = true;
  bool real_data = false;
  // Camera to Car extrinsic
  if (detection_success && real_data) {
    std::vector<std::vector<double>> intrinsic; // Camera intrinsic
    std::vector<double> dist;                   // Camera distortion
    std::vector<std::vector<float>>
        obj_pts; // coordinates in the car coordinate system
    std::vector<std::vector<float>> pts2d; // detected image points
    std::vector<float> rvec, tvec;         // calibration result
    solveCamPnP(obj_pts, pts2d, intrinsic, dist, rvec, tvec); // solver
  }
  // LiDAR to Car extrinsic
  if (detection_success && real_data) {
    std::vector<std::vector<float>> lidar_pts; // detected lidar points
    std::vector<std::vector<float>>
        obj_pts;                   // coordinates in the car coordinate system
    std::vector<float> rvec, tvec; // calibration result
    solveLidarPnP(obj_pts, lidar_pts, rvec, tvec); // solversolver
  }

  return 0;
}
