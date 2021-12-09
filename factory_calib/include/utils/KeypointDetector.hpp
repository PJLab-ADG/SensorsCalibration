/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#ifndef APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_KEYPOINTDETECTOR_HPP_
#define APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_KEYPOINTDETECTOR_HPP_

#include "Eigen/Core"
#include "utils/common.hpp"
#include "utils/corner.hpp"
#include "utils/filter.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

// use shi-Tomasi corner detection algorithm
class KeypointDetector {
public:
  KeypointDetector(int max_corners = 500, double quality_level = 0.01,
                   double min_distance = 15, int block_size = 3,
                   int gradient_size = 3)
      : max_corners_(max_corners), quality_level_(quality_level),
        min_distance_(min_distance), block_size_(block_size),
        gradient_size_(gradient_size) {}

  bool detect(const std::vector<std::vector<float>> &gray_img,
              std::vector<Point2f> *out_corners);

private:
  int max_corners_;
  double quality_level_;
  double min_distance_;

  int block_size_;
  int gradient_size_;

  std::vector<Point2f> corners_;
};

#endif // APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_KEYPOINTDETECTOR_HPP_
