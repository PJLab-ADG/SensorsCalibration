/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#ifndef APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_CIRCLE_BOARD_CORNER_DETECT_HPP_
#define APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_CIRCLE_BOARD_CORNER_DETECT_HPP_

#include "Eigen/Core"
#include "calibration_board.hpp"
#include "circle_board/circle_detect.hpp"
#include "utils/RansacFitLine.hpp"
#include "utils/common.hpp"
#include <math.h>
#include <memory>
#include <string>
#include <vector>

class LineModel;

namespace cameracalib {
namespace circleBoard {

struct CornerPoints {
  std::vector<std::vector<Point2f>> points;
  std::vector<float> line_slope; // in deg
};

struct BlackLines {
  int left_line_idx;
  int right_line_idx;
  std::vector<int> left_blackcircleidx; // index of circles
  std::vector<int> right_blackcircleidx;
  float estimated_line_dist;
};

class CornerDetector {
public:
  CornerDetector() {
    line_max_angle_diff_ = 3;
    min_line_dist_ = 10;
    max_line_dist_ = 500;
    dist_diff_ = 4;
    dist_diff_percent_ = 0.25;
  }
  bool detect(const std::vector<std::vector<float>> &gray_img,
              const CircleBoard &board_pattern, CornerPoints *corner_pts);

private:
  // estimate binary threshold value
  void EstimateGrayThres(const std::vector<std::vector<float>> &img_gray,
                         std::vector<int> *thres_box);

  bool findCirclePattern(const std::vector<Circle> &circles,
                         CornerPoints *corner_pts);

  bool clusterLines(const std::vector<Circle> &circles,
                    const std::vector<std::shared_ptr<LineModel>> &line_models,
                    CornerPoints *corner_pts);

  bool
  findTwoBlackLines(const std::vector<Circle> &circles,
                    const std::vector<std::shared_ptr<LineModel>> &line_models,
                    std::vector<BlackLines> *possible_black_lines);

  bool checkLinePattern(
      const std::vector<Circle> &circles, const int &left_black_circle_idx,
      const int &right_black_circle_idx, const std::vector<int> &line_pos_index,
      const std::vector<std::shared_ptr<LineModel>> &line_models,
      CornerPoints *corner_pts);

private:
  bool IsBlackCircleLine(const std::vector<int> &circle_idx,
                         const std::vector<Circle> &circles,
                         std::vector<int> *black_circle_idxs);

private:
  // configuration
  CircleBoard board_pattern_;
  LineConditions options_;

public:
  float line_max_angle_diff_;
  float min_line_dist_;
  float max_line_dist_;
  float dist_diff_;
  float dist_diff_percent_;
};

} // namespace circleBoard
} // namespace cameracalib

#endif // APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_CIRCLE_BOARD_CORNER_DETECT_HPP_
