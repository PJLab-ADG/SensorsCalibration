/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#pragma once

#include "utils/common.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <omp.h>
#include <random>
#include <string>
#include <vector>

struct LineModel {
  // ax + by + c = 0
  double m_a;
  double m_b;
  double m_c;
  std::vector<int> m_inliner_index;

  LineModel() { m_inliner_index.clear(); }
  LineModel(double a, double b, double c, std::vector<int> inliner_index)
      : m_a(a), m_b(b), m_c(c), m_inliner_index(inliner_index) {}
  void Clear() { m_inliner_index.clear(); }
};

class RansacFitLine {
public:
  RansacFitLine(double threshold, int max_iternations = 10);
  ~RansacFitLine(){};

  bool Estimate(const std::vector<double> x, const std::vector<double> y,
                double &a, double &b, double &c,
                std::vector<int> &inliner_index);

private:
  bool Evaluate(double a, double b, double c, double &score,
                std::vector<int> &inliner_idx);
  void getRandomIndex();
  void LeastSquare(const std::vector<double> x, const std::vector<double> y,
                   double &a, double &b, double &c);
  void GetLineSlope(double x1, double y1, double x2, double y2, double &a,
                    double &b, double &c);

private:
  std::vector<double> x_;
  std::vector<double> y_;

  // std::vector<std::shared_ptr<LineModel>> line_models_;
  std::shared_ptr<LineModel> best_line_model_;
  double threshold_;
  int max_iternations_;
  double best_score_;
  // int best_model_idx_;

  std::vector<std::mt19937> rand_engines_;
};