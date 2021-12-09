/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */

#pragma once

#include <time.h>

#include "utils/common.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <stack>
#include <string>
#include <vector>

namespace cameracalib {
namespace circleBoard {
struct Circle {
  Circle() {}
  Circle(Point2f mid, float r, std::vector<Point2i> pts)
      : center(mid), radius(r), points(pts) {}
  size_t size() { return points.size(); }

  Point2f center;
  float radius;
  std::vector<Point2i> points;
  // whether this circle is black circle
  bool black = false;
};

struct Contour {
  std::vector<Point2i> points;
  Point2f center;
};

struct circleDetectParam {
  float min_radius;
  float max_radius;
};

class CircleDetector {
public:
  CircleDetector() {
    min_contour_pt_num_ = 10;
    max_contour_pt_num_ = 1000;
    circle_diff_thresh_ = 0.25;
  }

  // TODO(liuzhuochun): add image mask
  bool detect(const std::vector<std::vector<int>> &thresh_img,
              std::vector<Circle> *circles);

  void generateParam(const int &img_w, const int &img_h);

private:
  bool FindContours(const std::vector<std::vector<int>> &thresh_vec,
                    std::vector<Contour> *contours);

  bool isCircle(const Contour &contour, Circle *circle);

  // check whether contour is valid
  bool isValidContour(const Contour &contour);

  // used in contour detector
  void DepthFirstSearch(std::vector<std::vector<int>> *img,
                        std::vector<Point2i> *contour_points, float *center_x,
                        float *center_y, int posx, int posy);

public:
  // parameters to filter invalid contour
  int min_contour_pt_num_;
  int max_contour_pt_num_;
  int min_contour_pt_dist_;
  int max_contour_pt_dist_;
  // parameter to filter invalid circle
  float min_radius_;
  float max_radius_;

public:
  float circle_diff_thresh_;
};

} // namespace circleBoard
} // namespace cameracalib
