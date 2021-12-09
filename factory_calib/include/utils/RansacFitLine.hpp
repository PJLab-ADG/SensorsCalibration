/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#ifndef APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_RANSACFITLINE_HPP_
#define APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_RANSACFITLINE_HPP_

#include "utils/LineModel.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <math.h>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace cameracalib {
// conditions for restricting lines
struct LineConditions {
  LineConditions()
      : slopeLimit(true), min_slope(70), max_slope(110), ptsNumLimit(true),
        min_pts_num(8), max_pts_num(1e5), ptsDistDiffLimit(true),
        max_pts_dist_diff_percent(5), lineDistDiffLimit(false),
        max_line_dist_diff(50) {}
  // restrict slope of a single line
  bool slopeLimit;
  double min_slope;
  double max_slope;
  // restrict amount of line points
  bool ptsNumLimit;
  int min_pts_num;
  int max_pts_num;
  // restrict distance between line points
  bool ptsDistDiffLimit;
  // percent of distance difference diff/dist
  double max_pts_dist_diff_percent;
  // restrict distance between lines
  bool lineDistDiffLimit;
  double max_line_dist_diff; // in pixel
};

class RansacFitLine {
public:
  /**
   * Ransac find lines with max inliners
   * [param in] line num
   * [param in] point-line distance threshold
   * [param in] max iterations
   **/
  RansacFitLine(LineConditions options,
                // int line_num,
                double threshold, int max_iternations = 10);
  ~RansacFitLine() {}

  bool Estimate(const std::vector<Eigen::Vector2d> &points,
                std::vector<std::shared_ptr<LineModel>> *lines);

  bool checkSingleLine(std::shared_ptr<LineModel> line);

  std::vector<std::shared_ptr<LineModel>> getDetectedLines() {
    return line_models_;
  }

private:
  // cluster and split line points
  void
  clusterLinePoints(const std::vector<Eigen::Vector2d> &points,
                    std::shared_ptr<LineModel> line, const double &pts_dist_th,
                    std::vector<std::shared_ptr<LineModel>> *clustered_lines);

  double Evaluate(const std::vector<Eigen::Vector2d> &points, double a,
                  double b, double c, std::vector<int> *inliner_idx);

  void GetLineSlope(double x1, double y1, double x2, double y2, double *a,
                    double *b, double *c);

  /*line conditions*/
  bool limitSlope(std::shared_ptr<LineModel> line, double min_slope,
                  double max_slope) {
    double angle = atan(line->m_a / line->m_b) * 180 / M_PI;
    if (angle < 0)
      angle += 180;
    if (angle < min_slope || angle > max_slope)
      return false;
    else
      return true;
  }

  bool limitPtsNum(std::shared_ptr<LineModel> line, double min_pts_num,
                   double max_pts_num) {
    int pts_num = line->m_inliner_index.size();
    if (pts_num < min_pts_num || pts_num > max_pts_num)
      return false;
    else
      return true;
  }

  bool whetherSameLine(std::shared_ptr<LineModel> a);

private:
  std::vector<double> x_;
  std::vector<double> y_;

  std::vector<std::shared_ptr<LineModel>> line_models_;
  std::vector<double> scores_;

  double threshold_;
  int max_iternations_;

  LineConditions options_;
};

} // namespace cameracalib

#endif // APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_RANSACFITLINE_HPP_
