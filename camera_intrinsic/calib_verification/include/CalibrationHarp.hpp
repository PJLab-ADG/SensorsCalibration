/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#pragma once

#include "LSD.hpp"
#include "common.hpp"
#include <opencv2/opencv.hpp>

class CalibrationHarp {
public:
  CalibrationHarp();
  ~CalibrationHarp() {}

  bool measure(const std::string &image_path, double &_d, double &_d_max);

private:
  cv::Mat origin_image_;
  std::vector<std::vector<Point2i>> selected_line_points_;
  cv::Mat line_support_region_;
  double distortion_error_;
  double max_distortion_error_;

  bool interpolate_edge_points(
      const std::vector<Vector4f> &lines,
      const std::vector<std::vector<Point2i>> &edge_points,
      std::vector<std::vector<Point2i>> &interpolated_edge_points);

  bool gaussion_subsample_points(
      const std::vector<std::vector<Point2i>> &interpolated_edge_points,
      std::vector<std::vector<Point2i>> &subsampled_edge_points,
      const int t = 30);

  bool calculate_error(
      // const std::vector<Vector4f> &lines,
      // const std::vector< std::vector<Point2i> > &edge_points,
      const std::vector<std::vector<Point2i>> &subsampled_edge_point, double &d,
      double &d_max, double &d_c_median);

  bool get_line_param(const std::vector<Point2i> &edge_points, double &alpha,
                      double &beta, double &gama);

  double get_theta(const std::vector<Point2i> &edge_points, const double &Ax,
                   const double &Ay);

  static inline bool sortLineByX_function(const Point2i &a, const Point2i &b) {
    return (a.x < b.x);
  }

  static inline bool sortLineByY_function(const Point2i &a, const Point2i &b) {
    return (a.y < b.y);
  }
};