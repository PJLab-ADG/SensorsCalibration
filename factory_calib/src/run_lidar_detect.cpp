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
  lidar_detector.LidarDetection(pcds_dir);

  return 0;
}
