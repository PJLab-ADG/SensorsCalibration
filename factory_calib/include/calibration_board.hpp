/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */

#ifndef APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_CALIBRATION_BOARD_HPP_
#define APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_CALIBRATION_BOARD_HPP_

#include "Eigen/Core"
#include "math.h"
#include "utils/common.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace cameracalib {

struct VerticalBoard {
  VerticalBoard() {}

  void set(const int &type) {
    lines.clear();
    if (type == 1) {
      lines.emplace_back(std::vector<int>(
          {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0}));
      lines.emplace_back(std::vector<int>(
          {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}));
      lines.emplace_back(std::vector<int>(
          {0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0}));
    } else if (type == 2) {
      lines.emplace_back(
          std::vector<int>({1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0}));
      lines.emplace_back(
          std::vector<int>({1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}));
      lines.emplace_back(
          std::vector<int>({0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0}));
    }
    num = 3;
  }

  /** board pattern
   * 1: corner 0: no corner
   * eg.
   * right line vector:  0 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 0
   *   mid line vector:  1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
   *  left line vector:  1 1 1 1 1 1 1 1 1 1 0 1 1 1 1 0 1 1 0
   * from left to right line
  **/
  std::vector<std::vector<int>> lines;
  int num;

  // check whether the board pattern meet requirements
  bool check() {
    if (lines.size() != 3) {
      // LOGE("Current program only support three-line board.\n");
      return false;
    }
    for (size_t i = 0; i < lines.size() - 1; ++i) {
      if (lines[i].size() != lines[i + 1].size()) {
        // LOGE("Patterns of lines should be the same length.\n");
        return false;
      }
    }
    for (size_t i = 0; i < lines.size(); ++i) {
      int corner_num = accumulate(lines[i].begin(), lines[i].end(), 0);
      if (corner_num < 6) {
        // LOGE("Too few corners on the line!\n");
        return false;
      }
    }
    return true;
  }
};

struct CircleBoard {
  CircleBoard() {}

  int width = 6;       // circle col num
  int height = 5;      // circle row num
  float big_r = 10.5;  // circle radius of big white circle
  float small_r = 9.8; // circle radius of small black circle
  float pt_dist = 28;  // circle distance in vertical direction

  // check whether the board pattern meet requirements
  bool check() {
    if (width % 2 != 0 || width < 4)
      return false;
    if (height % 2 == 0 || height < 3)
      return false;
    if (pt_dist < big_r * 2)
      return false;
    return true;
  }
};

struct ArucoMarker {
  ArucoMarker() {
    id_box.emplace_back(
        std::vector<bool>{0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0});
    id_box.emplace_back(
        std::vector<bool>{0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1});
    id_box.emplace_back(
        std::vector<bool>{0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1});
    id_box.emplace_back(
        std::vector<bool>{0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1});
    marker_size = 4;

    std::vector<Eigen::Vector3d> marker0 = {
        Eigen::Vector3d(5, 0.1, 1.6), Eigen::Vector3d(5, -0.1, 1.6),
        Eigen::Vector3d(5, -0.1, 1.4), Eigen::Vector3d(5, 0.1, 1.4)};
    std::vector<Eigen::Vector3d> marker1 = {
        Eigen::Vector3d(5, 0.4, 1.3), Eigen::Vector3d(5, 0.2, 1.3),
        Eigen::Vector3d(5, 0.2, 1.1), Eigen::Vector3d(5, 0.4, 1.1)};
    std::vector<Eigen::Vector3d> marker2 = {
        Eigen::Vector3d(5, -0.2, 1.3), Eigen::Vector3d(5, -0.4, 1.3),
        Eigen::Vector3d(5, -0.4, 1.1), Eigen::Vector3d(5, -0.2, 1.1)};
    std::vector<Eigen::Vector3d> marker3 = {
        Eigen::Vector3d(5, 0.1, 1.0), Eigen::Vector3d(5, -0.1, 1.0),
        Eigen::Vector3d(5, -0.1, 0.8), Eigen::Vector3d(5, 0.1, 0.8)};
    pos.emplace_back(marker0);
    pos.emplace_back(marker1);
    pos.emplace_back(marker2);
    pos.emplace_back(marker3);
  }

  std::vector<std::vector<bool>> id_box;
  std::vector<std::vector<Eigen::Vector3d>> pos;
  int marker_size;
  float grid_size = 0.2;

  // check whether the board pattern meet requirements
  bool check() {
    if (id_box.size() < 1)
      return false;
    size_t size = id_box[0].size();
    double marker_size = sqrt(static_cast<double>(size));
    if (pos.size() != id_box.size())
      return false;
    if (abs(marker_size - static_cast<int>(marker_size)) > 1e-06)
      return false;
    for (size_t i = 0; i < id_box.size(); ++i) {
      if (id_box[i].size() != size)
        return false;
      if (pos[i].size() != 4)
        return false;
    }
    return true;
  }
};

} // namespace cameracalib

#endif // APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_CALIBRATION_BOARD_HPP_
