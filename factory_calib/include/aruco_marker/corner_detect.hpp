/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ctime>
#include <iostream>
#include <stack>
#include <string>
#include <vector>

#include "aruco_marker/simplifyPath.hpp"
#include "calibration_board.hpp"
#include "dataConvert.hpp"
#include "utils/common.hpp"

namespace cameracalib {
namespace arucomarker {

struct CornerPoints {
  void clear() {
    points.clear();
    tag_ids.clear();
  }
  std::vector<std::vector<Point2f>> points;
  std::vector<int> tag_ids;
};

class ArucoMarkerDetector {
public:
  bool detect(const std::vector<std::vector<float>> &gray_img,
              const ArucoMarker &board_pattern, CornerPoints *corner_pts);

private:
  void EstimateGrayThres(const std::vector<std::vector<float>> &imgGray,
                         std::vector<int> *thres_box,
                         float edge_fill_percent = 0.1);

  void ImageThreshold(const std::vector<std::vector<float>> &imgGray,
                      const double &thresh_threshold,
                      std::vector<std::vector<bool>> *thresh_vec,
                      float edge_fill_percent = 0.1);

  std::vector<PointRDP> DepthFirstSearch(std::vector<std::vector<bool>> *img,
                                         int i, int j);

  void ApproxPolyDP(const std::vector<PointRDP> src_contour, double eps,
                    std::vector<PointRDP> *rt_contour);

  void findContours(const std::vector<std::vector<bool>> &_in,
                    std::vector<std::vector<PointRDP>> &contours);

  bool isContourConvex(const std::vector<PointRDP> &_contour);

  void GetContours(const std::vector<std::vector<bool>> &dilation_vec,
                   std::vector<std::vector<bool>> *contour_img,
                   std::vector<std::vector<PointRDP>> *contours,
                   float edge_fill_percent);

  void findMarkerContours(const std::vector<std::vector<bool>> &_in,
                          std::vector<std::vector<Point2f>> &candidates,
                          std::vector<std::vector<PointRDP>> &contoursOut,
                          double minPerimeterRate, double maxPerimeterRate,
                          double accuracyRate, double minCornerDistanceRate,
                          int minDistanceToBorder);

  Eigen::MatrixXf getPerspectiveTransform(std::vector<Point2f> src,
                                          std::vector<Point2f> dst);

  void warpPerspective(const std::vector<std::vector<bool>> &_src,
                       std::vector<std::vector<bool>> &_dst,
                       Eigen::MatrixXf transformation, int dstSize);

  int identifyOneCandidate(const std::vector<std::vector<bool>> &gray,
                           const std::vector<Point2f> &corners,
                           const ArucoMarker &board_pattern) {
    return extractBits(gray, corners, board_pattern.marker_size,
                       markerBorderBits_, perspectiveRemovePixelPerCell_,
                       perspectiveRemoveIgnoredMarginPerCell_, minOtsuStdDev_,
                       board_pattern.id_box);
  }

  bool getPixel(const std::vector<std::vector<bool>> &_src, float org_x,
                float org_y);

  int findID(std::vector<bool> ID_detect,
             std::vector<std::vector<bool>> ID_org);

  int extractBits(const std::vector<std::vector<bool>> &_image,
                  const std::vector<Point2f> &_corners, int markerSize,
                  int markerBorderBits, int cellSize, double cellMarginRate,
                  double minStdDevOtsu,
                  const std::vector<std::vector<bool>> &ID_org);

  void filterTooCloseCandidates(
      const std::vector<std::vector<Point2f>> &candidatesIn,
      std::vector<std::vector<std::vector<Point2f>>> &candidatesSetOut,
      const std::vector<std::vector<PointRDP>> &contoursIn,
      std::vector<std::vector<std::vector<PointRDP>>> &contoursSetOut,
      double minMarkerDistanceRate, bool detectInvertedMarker);

  std::vector<Point2f> alignContourOrder(Point2f corner,
                                         std::vector<Point2f> candidate);

  // parameters
  double minPerimeterRate_ = 0.03;
  double maxPerimeterRate_ = 4.0;
  double ApproxAccuracyRate_ = 0.03;
  double minCornerDistanceRate_ = 0.05;
  int minDistanceToBorder_ = 3;

  float minMarkerDistanceRate_ = 0.05;
  bool detectInvertedMarker_ = false;
  int markerBorderBits_ = 2;
  int perspectiveRemovePixelPerCell_ = 4;
  double perspectiveRemoveIgnoredMarginPerCell_ = 0.13;
  double minOtsuStdDev_ = 5.0;

  const int min_detected_marker_num_ = 4;
};

} // namespace arucomarker
} // namespace cameracalib