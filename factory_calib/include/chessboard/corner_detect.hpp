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

// using namespace std;
struct Point {
  Point() {}
  Point(int x, int y) {
    this->x = x;
    this->y = y;
  }
  int x;
  int y;
};

struct Quad {
  Point center;
  Point p1;
  Point p2;
  Point p3;
  Point p4;
};

class CornerDetect {
public:
  bool CornerDetection(const std::vector<std::vector<float>> &gray_img,
                       int *vp_x, int *vp_y, Point *sp1, Point *sp2,
                       int *grid_x_dis, int *grid_y_dis,
                       std::vector<Point> *grid_center_points);

private:
  double CalculatePointDis(double x1, double y1, double x2, double y2);

  bool IsRectangle(const std::vector<Point> &approx_contour);

  bool IsSquare(const std::vector<Point> &approx_contour);

  bool IsRightTriangle(const std::vector<Quad> &quads);

  bool IsIsoscelesTriangle(const std::vector<Quad> &quads);

  bool IsQuadEqual(Quad q1, Quad q2);

  void ClusterByRansac(const std::vector<Quad> &cluster_tmp,
                       const double &dis_lower_thresh,
                       const double &dis_upper_thresh,
                       std::vector<std::vector<Quad>> *sets);

  void GetSlopPoint(const std::vector<Quad> &chess, Point *point);

  double Max(int x1, int x2, int x3, int x4);

  double Min(int x1, int x2, int x3, int x4);

  void GetQuadDis(const Quad quad, int *qx, int *qy);

  void GetGridDis(const std::vector<std::vector<Quad>> chess_sets, int *gx_dis,
                  int *gy_dis);

  double CalculateSlope(const std::vector<Quad> &first_chess,
                        const std::vector<Quad> &second_chess);

  double CalculateArea(const std::vector<Quad> &chess);

  bool CheckChessBoardArea(const std::vector<Quad> &first_chess,
                           const std::vector<Quad> &second_chess);

  bool CheckChessboard(const std::vector<std::vector<Quad>> &valid_sets,
                       std::vector<Quad> *valid_Chessboard1,
                       std::vector<Quad> *valid_Chessboard2);

  void EstimateGrayThres(const std::vector<std::vector<float>> &imgGray,
                         std::vector<int> *thres_box,
                         float edge_fill_percent = 0.2);

  void ImageThreshold(const std::vector<std::vector<float>> &imgGray,
                      const double &thresh_threshold,
                      std::vector<std::vector<int>> *thresh_vec,
                      float edge_fill_percent = 0.2);

  void ImageDilate(const std::vector<std::vector<int>> thresh_vec,
                   int kernal[3][3],
                   std::vector<std::vector<int>> *dilation_vec,
                   float edge_fill_percent = 0.2);

  double CalculateContourArea(std::vector<Point> contour);

  std::vector<Point> DepthFirstSearch(std::vector<std::vector<int>> *img, int i,
                                      int j);

  void FindContours(const std::vector<std::vector<int>> dilation_vec,
                    std::vector<std::vector<Point>> *rt_contours,
                    float edge_fill_percent = 0.2);

  void ApproxPolyDP(const std::vector<Point> src_contour, double eps,
                    std::vector<Point> *rt_contour);
};
