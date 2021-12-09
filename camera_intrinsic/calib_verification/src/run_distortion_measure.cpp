/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include <dirent.h>
#include <iostream>

#include "CalibrationHarp.hpp"
#include <opencv2/opencv.hpp>

const char usage[] = "\t./bin/run_distortion_measure <distortion_image_path>\n"
                     "example:\n\t"
                     "./bin/run_distortion_measure data/test.png\n";

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout << argv[0] << usage;
    return 1;
  }

  std::string input_image_path = argv[1];
  cv::Mat image = cv::imread(input_image_path.c_str(), 0);

  CalibrationHarp distortion_measurement;
  double d, d_max;
  auto flag = distortion_measurement.measure(input_image_path, d, d_max);

  return 0;
}