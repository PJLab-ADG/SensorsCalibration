/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#include "utils/RansacFitLine.hpp"
#include <stdlib.h>

namespace cameracalib {

RansacFitLine::RansacFitLine(LineConditions options, double threshold,
                             int max_iternations) {
  options_ = options;
  threshold_ = threshold;
  max_iternations_ = max_iternations;
}

bool RansacFitLine::checkSingleLine(std::shared_ptr<LineModel> line) {
  if (options_.ptsNumLimit) {
    if (!limitPtsNum(line, options_.min_pts_num, options_.max_pts_num))
      return false;
  }
  if (options_.slopeLimit) {
    if (!limitSlope(line, options_.min_slope, options_.max_slope))
      return false;
  }
  return true;
}

bool RansacFitLine::Estimate(const std::vector<Eigen::Vector2d> &points,
                             std::vector<std::shared_ptr<LineModel>> *lines) {
  /* generate random index vector */
  srand(time(NULL));

  for (int i = 0; i < max_iternations_; ++i) {
    std::shared_ptr<LineModel> line_model = std::make_shared<LineModel>();
    // get random index
    int idx1 = random() % points.size();
    int idx2 = random() % points.size();
    double thresh_side = threshold_ * 150;
    while (idx2 == idx1 ||
           fabs(points[idx1](0) - points[idx2](0)) +
                   fabs(points[idx1](1) - points[idx2](1)) >
               thresh_side) {
      idx2 = random() % points.size();
    }
    // build the line and find inliners
    GetLineSlope(points[idx1](0), points[idx1](1), points[idx2](0),
                 points[idx2](1), &(line_model->m_a), &(line_model->m_b),
                 &(line_model->m_c));
    // check slope before evaluate to speed up
    if (options_.slopeLimit) {
      if (!limitSlope(line_model, options_.min_slope, options_.max_slope))
        continue;
    }
    Evaluate(points, line_model->m_a, line_model->m_b, line_model->m_c,
             &line_model->m_inliner_index);
    line_model->setOriginPts(points[idx1], points[idx2]);
    // rough check
    if (!this->checkSingleLine(line_model))
      continue;

    // cluster line points
    std::vector<std::shared_ptr<LineModel>> clustered_lines;
    this->clusterLinePoints(points, line_model,
                            options_.max_pts_dist_diff_percent,
                            &clustered_lines);

    // check each clustered line
    for (size_t lidx = 0; lidx < clustered_lines.size(); lidx++) {
      // check line attributes based on line conditions
      if (!this->checkSingleLine(clustered_lines[lidx]))
        continue;
      // check whether the line was found before
      if (whetherSameLine(clustered_lines[lidx]))
        continue;
      line_models_.emplace_back(clustered_lines[lidx]);
      scores_.emplace_back(clustered_lines[lidx]->size());
    }
  }
  *lines = line_models_;
  if (line_models_.size() < 1) {
    // LOGE("Detected 0 lines\n");
    return false;
  }
  return true;
}

bool RansacFitLine::whetherSameLine(std::shared_ptr<LineModel> a) {
  for (size_t lidx = 0; lidx < line_models_.size(); ++lidx) {
    std::shared_ptr<LineModel> b = line_models_[lidx];
    double angle_a = a->getLineSlopeAngle();
    double angle_b = b->getLineSlopeAngle();
    // whether slope is the same
    if (fabs(angle_a - angle_b) < 2) {
      Eigen::Vector2d end_pt1, end_pt2;
      a->getEndPoint(&end_pt1, &end_pt2);
      if (a->compareIndex(b->m_inliner_index, 0.99) &&
          (b->getPointLineDist(end_pt1) < threshold_ &&
           b->getPointLineDist(end_pt2) < threshold_)) {
        // if (a->size() > b->size() &&
        // a->compareIndex(b->m_inliner_index, 0.99)) {
        if (a->size() > b->size()) {
          // replace old one
          // std::cout << "replace old line\n";
          line_models_.erase(line_models_.begin() + lidx);
          scores_.erase(scores_.begin() + lidx);
          return false;
        } else {
          return true;
        }
      }
    }
  }
  return false;
}

double RansacFitLine::Evaluate(const std::vector<Eigen::Vector2d> &points,
                               double a, double b, double c,
                               std::vector<int> *inliner_idx) {
  if (a == 0 && b == 0) {
    // LOGE("[RansacFitLine]Wrong line param.");
    return false;
  }
  inliner_idx->clear();
  int inliner_num = 0;
  int total_num = points.size();
  double line_dist_sqrt = sqrt(a * a + b * b);

  for (int i = 0; i < total_num; ++i) {
    double cur_x = points[i](0);
    double cur_y = points[i](1);
    // compute distance between point and line
    double dist_value = fabs(a * cur_x + b * cur_y + c) / line_dist_sqrt;
    if (dist_value <= threshold_) {
      inliner_idx->emplace_back(i);
      inliner_num++;
    }
  }
  double score =
      static_cast<double>(inliner_num) / static_cast<double>(total_num);
  return score;
}

void RansacFitLine::clusterLinePoints(
    const std::vector<Eigen::Vector2d> &points, std::shared_ptr<LineModel> line,
    const double &pts_dist_th,
    std::vector<std::shared_ptr<LineModel>> *clustered_lines) {
  clustered_lines->clear();
  line->sort(points);
  // compute distance between points
  std::vector<double> dists;
  for (size_t i = 1; i < line->size(); ++i) {
    dists.emplace_back((line->m_line_pts[i] - line->m_line_pts[i - 1]).norm());
  }

  double dist_mean = dists[0];
  int pt_num = 1;
  int start_pos = 0;
  for (size_t i = 1; i < dists.size(); ++i) {
    double c_dist = dists[i];
    double dist_diff_p = fabs(c_dist - dist_mean) / std::min(c_dist, dist_mean);
    if (dist_diff_p > pts_dist_th) {
      // split line
      dist_mean = dists[i];
      pt_num = 1;
      int end_pos = i;
      // add new line model
      if (i - start_pos > 1) {
        std::shared_ptr<LineModel> new_line = std::make_shared<LineModel>();
        new_line->copyFromSortedSegmentLine(line, start_pos, end_pos);
        // get end points
        Eigen::Vector2d end_pt1, end_pt2;
        new_line->getEndPoint(points, &end_pt1, &end_pt2);
        new_line->setOriginPts(end_pt1, end_pt2);
        // recompute slope
        this->GetLineSlope(end_pt1(0), end_pt1(1), end_pt2(0), end_pt2(1),
                           &new_line->m_a, &new_line->m_b, &new_line->m_c);
        clustered_lines->emplace_back(new_line);
      }
      start_pos = end_pos;
    } else {
      dist_mean = (dist_mean * pt_num + dists[i]) / (pt_num + 1);
      pt_num++;
    }
  }
  int end_pos = dists.size();
  // add final segment
  if (end_pos - start_pos >= 1) {
    std::shared_ptr<LineModel> new_line = std::make_shared<LineModel>();
    new_line->copyFromSortedSegmentLine(line, start_pos, end_pos);
    // get end points
    Eigen::Vector2d end_pt1, end_pt2;
    new_line->getEndPoint(points, &end_pt1, &end_pt2);
    new_line->setOriginPts(end_pt1, end_pt2);
    // recompute slope
    this->GetLineSlope(end_pt1(0), end_pt1(1), end_pt2(0), end_pt2(1),
                       &new_line->m_a, &new_line->m_b, &new_line->m_c);
    clustered_lines->emplace_back(new_line);
  }
}

// ax + by + c = 0
void RansacFitLine::GetLineSlope(double x1, double y1, double x2, double y2,
                                 double *a, double *b, double *c) {
  *a = y2 - y1;
  *b = x1 - x2;
  *c = x2 * y1 - x1 * y2;
  *a *= 0.01;
  *b *= 0.01;
  *c *= 0.01;
}

} // namespace cameracalib
