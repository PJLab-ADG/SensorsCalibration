/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#pragma once

#include <assert.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include "common.hpp"
#include <opencv2/opencv.hpp>

// double VectorAngle(const double &x1, const double &y1,
//                    const double &x2, const double &y2)
// {
//     double f = (x1 * x2 + y1 * y2) / sqrt(x1*x1 + y1*y1) / sqrt(x2*x2 +
//     y2*y2);
//     return acos(f) * 180 / M_PI;
// }

struct BoardSquare {
public:
  BoardSquare(){};
  BoardSquare(const cv::Point &p1, const cv::Point &p2, const cv::Point &p3,
              const cv::Point &p4, const int &idx = 0) {
    square_vertex.clear();
    square_vertex.push_back(p1);
    square_vertex.push_back(p2);
    square_vertex.push_back(p3);
    square_vertex.push_back(p4);
    angle_left_top =
        VectorAngle(p2.x - p1.x, p2.y - p1.y, p3.x - p1.x, p3.y - p1.y);
    angle_right_top =
        VectorAngle(p1.x - p2.x, p1.y - p2.y, p4.x - p2.x, p4.y - p2.y);
    angle_left_bottom =
        VectorAngle(p1.x - p3.x, p1.y - p3.y, p4.x - p3.x, p4.y - p3.y);
    angle_right_bottom =
        360.0 - angle_left_top - angle_right_top - angle_left_bottom;
    midpoint.x = (p1.x + p2.x + p3.x + p4.x) / 4.0;
    midpoint.y = (p1.y + p2.y + p3.y + p4.y) / 4.0;

    max_x = std::max(p1.x, std::max(p2.x, std::max(p3.x, p4.x)));
    min_x = std::min(p1.x, std::min(p2.x, std::min(p3.x, p4.x)));
    max_y = std::max(p1.y, std::max(p2.y, std::max(p3.y, p4.y)));
    min_y = std::min(p1.y, std::min(p2.y, std::min(p3.y, p4.y)));

    double d12 =
        sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
    double d13 =
        sqrt((p3.x - p1.x) * (p3.x - p1.x) + (p3.y - p1.y) * (p3.y - p1.y));
    double d24 =
        sqrt((p2.x - p4.x) * (p2.x - p4.x) + (p2.y - p4.y) * (p2.y - p4.y));
    double d34 =
        sqrt((p3.x - p4.x) * (p3.x - p4.x) + (p3.y - p4.y) * (p3.y - p4.y));
    area = 1 / 2.0 * sin(angle_left_top * M_PI / 180.0) * d12 * d13 +
           1 / 2.0 * sin(angle_right_bottom * M_PI / 180.0) * d24 * d34;
    index = idx;
  }

  // computeRegion(const int &img_w){
  //     double appro_length = fabs(max_x - min_x + max_y - min_y) / 2.0;
  //     double mid_vec = img_w -
  // }
  double VectorAngle(const double &x1, const double &y1, const double &x2,
                     const double &y2) {
    double f =
        (x1 * x2 + y1 * y2) / sqrt(x1 * x1 + y1 * y1) / sqrt(x2 * x2 + y2 * y2);
    return acos(f) * 180 / M_PI;
  }

  std::vector<cv::Point> square_vertex;
  std::vector<std::vector<int>> square_boundary;
  cv::Point midpoint;
  double max_x, min_x, max_y, min_y;
  double angle_left_top;
  double angle_right_top;
  double angle_left_bottom;
  double angle_right_bottom;
  double area;

  int index;
  int score;
  // -1 = left region, 0 = middle region, 1 = right region
  int region;
};

class AutoImagePicker {
public:
  AutoImagePicker(const int &img_width, const int &img_height,
                  const int &board_width, const int &board_height);

  // input detected corner
  // return flag whether this image is picked
  bool addImage(const std::vector<cv::Point2f> &image_corners);

  // check the proportion of photos distributed in different regions
  // similar amount in left, middle, areas
  // return flags whether to stop input
  bool status();
  void reset();

private:
  // check if the board is too far or too inclined
  bool checkValidity(const BoardSquare &board);
  bool checkMoveThresh(const BoardSquare &board);
  bool checkAreaThresh(const BoardSquare &board);
  bool checkPoseAngleThresh(const BoardSquare &board);
  // return left/middle/right region that board belongs to
  // -1 = left, 0 = middle, 1 = right
  int getBoardRegion(const BoardSquare &board);

  // utils
private:
  int getAreaScore(const BoardSquare &board);
  void fillAreaBox(const BoardSquare &board);

private:
  int img_width_;
  int img_height_;
  int board_width_;
  int board_height_;

  std::vector<std::vector<int>> area_box_;
  std::vector<BoardSquare> candidate_board_;
  std::vector<std::vector<BoardSquare>>
      board_square_box_; // divided into left, middle, right 3 areas
};