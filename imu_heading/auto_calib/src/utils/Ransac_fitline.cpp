/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#include "utils/Ransac_fitline.h"

#include <stdlib.h>

RansacFitLine::RansacFitLine(double threshold, int max_iternations) {
  threshold_ = threshold;
  max_iternations_ = max_iternations;
  // TODO: add omp

  // int nThreads = std::max(1, omp_get_max_threads());
  // std::cout << "[ INFO ]: Maximum usable threads: " << nThreads << std::endl;
  // for (int i = 0; i < nThreads; ++i)
  // {
  // 	std::random_device SeedDevice;
  // 	rand_engines_.push_back(std::mt19937(SeedDevice()));
  // }
  best_score_ = 0;
}

bool RansacFitLine::Estimate(const std::vector<double> x,
                             const std::vector<double> y, double &a, double &b,
                             double &c, std::vector<int> &inliner_index) {
  x_ = x;
  y_ = y;
  /* generate random index vector */
  srand(time(NULL));
  std::vector<int> rand_idx_vec;
  int idx1, idx2;
  for (int i = 0; i < max_iternations_; i++) {
    idx1 = random() % x.size();
    idx2 = random() % x.size();
    while (idx2 == idx1) {
      idx2 = random() % x.size();
    }
    rand_idx_vec.push_back(idx1);
    rand_idx_vec.push_back(idx2);
  }
  // int nThreads = std::max(1, omp_get_max_threads());
  // omp_set_dynamic(0); // Explicitly disable dynamic teams
  // omp_set_num_threads(nThreads);

  // #pragma omp parallel for
  for (int i = 0; i < max_iternations_; i++) {
    idx1 = rand_idx_vec[i * 2];
    idx2 = rand_idx_vec[i * 2 + 1];
    // std::cout << "=>Iter " << i << " (" << idx1 << ", " << idx2 << ")\n";
    std::shared_ptr<LineModel> line_model = std::make_shared<LineModel>();
    this->GetLineSlope(x_[idx1], y_[idx1], x_[idx2], y_[idx2], line_model->m_a,
                       line_model->m_b, line_model->m_c);
    double score;
    this->Evaluate(line_model->m_a, line_model->m_b, line_model->m_c, score,
                   line_model->m_inliner_index);
    if (score > best_score_) {
      // std::cout << "best score: " << score << std::endl;
      best_score_ = score;
      best_line_model_ = line_model;
    }
  }

  inliner_index = best_line_model_->m_inliner_index;
  a = best_line_model_->m_a;
  b = best_line_model_->m_b;
  c = best_line_model_->m_c;

  return true;
}

bool RansacFitLine::Evaluate(double a, double b, double c, double &score,
                             std::vector<int> &inliner_idx) {
  if (a == 0 && b == 0) {
    std::cerr << "[RansacFitLine]Wrong line param." << std::endl;
    return false;
  }
  inliner_idx.clear();
  int inliner_num = 0;
  int total_num = x_.size();
  double line_dist_sqrt = sqrt(a * a + b * b);

  for (int i = 0; i < total_num; i++) {
    double cur_x = x_[i];
    double cur_y = y_[i];
    // compute distance between point and line
    double dist_value = fabs(a * cur_x + b * cur_y + c) / line_dist_sqrt;
    if (dist_value <= threshold_) {
      inliner_idx.push_back(i);
      inliner_num++;
    }
  }
  // std::cout << "inliner num: " << inliner_num << std::endl;
  score = static_cast<double>(inliner_num) / static_cast<double>(total_num);
  return true;
}

// ax + by + c = 0
void RansacFitLine::LeastSquare(const std::vector<double> x,
                                const std::vector<double> y, double &a,
                                double &b, double &c) {
  double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
  for (size_t i = 0; i < x.size(); ++i) {
    t1 += x[i] * x[i];
    t2 += x[i];
    t3 += x[i] * y[i];
    t4 += y[i];
  }
  if (t1 * x.size() == t2 * t2) {
    b = 0;
    a = 1;
    c = -x[0];
  } else {
    a = -(t3 * x.size() - t2 * t4) / (t1 * x.size() - t2 * t2);
    b = 1;
    c = -(t1 * t4 - t2 * t3) / (t1 * x.size() - t2 * t2);
  }
}

void RansacFitLine::GetLineSlope(double x1, double y1, double x2, double y2,
                                 double &a, double &b, double &c) {
  if (x1 == x2) {
    b = 0;
    a = 1;
    c = -x1;
  } else {
    a = -(y1 - y2) / (x1 - x2);
    b = 1;
    c = -(x1 * y2 - x2 * y1) / (x1 - x2);
  }
}
