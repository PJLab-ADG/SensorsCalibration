/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#include "Eigen/Core"
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "calibration/adas_calib.hpp"
#include "dataConvert.hpp"
#include "logging.hpp"

int main(int argc, char **argv) {
  if (argc != 4 && argc != 5) {
    std::cerr << "usage:";
    return -1;
  }
  std::cout << " " << std::endl;
  return 0;
}
