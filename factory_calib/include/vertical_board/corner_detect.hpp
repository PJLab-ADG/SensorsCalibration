/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#ifndef APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_VERTICAL_BOARD_CORNER_DETECT_HPP_
#define APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_VERTICAL_BOARD_CORNER_DETECT_HPP_

#include "Eigen/Core"
#include "calibration_board.hpp"
#include "utils/LineModel.hpp"
#include <math.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

namespace cameracalib {
namespace verticalBoard {

struct CornerPoints {
  std::vector<std::vector<int>> pattern_idxs_;
  std::vector<std::shared_ptr<LineModel>> lines_;
};

struct CalibrationParam {
  CalibrationParam()
      : angle_diff_th(3), dist_diff_th(0.3), min_dist_th(15), max_dist_th(100) {
  }

  // calibration configuration
  double angle_diff_th; // thresh of angle difference between lines(in deg)
  double dist_diff_th;  // thresh of dist difference between lines(percent)
  double min_dist_th;   // min dist between lines(pixels)
  double max_dist_th;   // max dist between lines(pixels)
};

class CornerDetector {
public:
  CornerDetector() {}
  bool detect(const std::vector<std::vector<float>> &gray_img,
              const VerticalBoard &lp, CornerPoints *corner_pts);

private:
  void displayDetectedLines(
      const cv::Mat &img,
      const std::vector<std::shared_ptr<LineModel>> &line_models);
  // based on realtion between lines to select specific lines
  // find three clustered lines (left, middle, right)
  // project left and right line point on middle line to find joint point
  // adapt to the case of missed detection
  bool selectLinesByLineRelation(
      const std::vector<std::shared_ptr<LineModel>> &lines,
      const double &dist_thresh,
      std::vector<std::shared_ptr<LineModel>> *selected_lines,
      std::vector<std::vector<int>> *pattern_idxs);

  // utils
  void
  findLinesStartEndPoints(const std::vector<Eigen::Vector3i> &points_line_idx,
                          Eigen::Vector3i *start_idxs,
                          Eigen::Vector3i *end_idxs);

  // used in selectLinesByLineRelation
  bool findRightLeftIdx(const std::vector<size_t> &left_line_idx,
                        const std::vector<size_t> &right_line_idx,
                        const std::vector<double> &line_dists,
                        const double &dist_diff_th,
                        std::vector<Eigen::Vector2i> *left_right_idxs);

  // check whether the three line group fit requirements
  // used in selectLinesByLineRelation
  bool
  checkLineCluster(const std::shared_ptr<LineModel> mid_line,
                   const std::shared_ptr<LineModel> left_line,
                   const std::shared_ptr<LineModel> right_line,
                   const double &estimated_pt_dist, // based on line-line dist
                   const double &dist_th,           // in pixels
                   std::vector<Eigen::Vector3i> *points_line_idx,
                   std::vector<std::vector<int>> *pattern_idxs,
                   int *point_num_score);

  bool
  checkProjectLinePattern(const std::vector<int> &grid_dist_vec,
                          const std::vector<Eigen::Vector3i> &points_line_idx,
                          const int &start_pos, const int &end_pos,
                          std::vector<std::vector<int>> *pattern_idxs,
                          int *point_num);

  // used in selectLinesByLineRelation
  void projectJointLine(const std::shared_ptr<LineModel> mid_line,
                        const std::shared_ptr<LineModel> left_line,
                        const std::shared_ptr<LineModel> right_line,
                        const double &dist_th, // same as threshold(in pixel)
                        std::vector<Eigen::Vector2d> *joint_line,
                        std::vector<Eigen::Vector3i> *points_line_idx);

private:
  // configuration
  VerticalBoard line_pattern_;
  CalibrationParam param_;

public:
  cv::Mat image_;
};
} // namespace verticalBoard
} // namespace cameracalib

#endif // APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_VERTICAL_BOARD_CORNER_DETECT_HPP_
