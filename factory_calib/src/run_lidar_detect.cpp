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

#include "round_hole_board/lidar_pattern.h"

char usage[] = {"[Usage]: ./bin/run_lidar_detect pcds \n"
                "eg: ./bin/run_lidar_detect data/pcds/ \n"};

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << usage;
    return -1;
  }
  std::string pcds_dir = argv[1];
  // round hole board
  lidarcalib::LidarDetector lidar_detector;
  bool sta = lidar_detector.LidarCircleCenterDetection(pcds_dir, true, true);
  std::vector<lidarcalib::Point3D> lidar_output =
      lidar_detector.GetLidarDetectPoints();

  if (sta) {
    std::cout << "\nDetection Success!\n";
    std::cout << "Number of Centers is " << lidar_output.size() << std::endl;
    for (size_t i = 0; i < lidar_output.size(); i++) {
      std::cout << "marker" << i << ": " << lidar_output[i].x << " "
                << lidar_output[i].y << " " << lidar_output[i].z << std::endl;
    }

  } else {
    std::cout << "\nDetection Failed!\n";
  }

  return 0;
}
