/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#ifndef APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_COMMON_HPP_
#define APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_COMMON_HPP_

#include "Eigen/Core"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

// image point struture
struct Point2i {
  Point2i() {
    x = 0;
    y = 0;
  }
  Point2i(int x, int y) {
    this->x = x;
    this->y = y;
  }

  /**
   *  |------->x
   *  |
   *  |
   *  v y
  **/
  int x; // in col
  int y; // in row
};

// image point struture
struct Point2f {
  Point2f() {
    x = 0;
    y = 0;
  }
  Point2f(float x, float y) {
    this->x = x;
    this->y = y;
  }
  explicit Point2f(Point2i p) {
    this->x = static_cast<float>(p.x);
    this->y = static_cast<float>(p.y);
  }

  float x; // in col
  float y; // in row
};

#endif // APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_COMMON_HPP_
