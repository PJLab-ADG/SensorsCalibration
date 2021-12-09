/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#ifndef APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_LINEMODEL_HPP_
#define APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_LINEMODEL_HPP_

#include "Eigen/Core"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

// specific line model adapt to ransac
// save line points index only in default
class LineModel {
public:
  LineModel() { sorted_ = false; }
  LineModel(double a, double b, double c, std::vector<int> inliner_index)
      : m_a(a), m_b(b), m_c(c), m_inliner_index(inliner_index), sorted_(false) {
  }

  void Clear() {
    m_inliner_index.clear();
    m_origin_pts.clear();
    m_line_pts.clear();
    sorted_ = false;
  }

  size_t size() { return m_inliner_index.size(); }

  void setOriginPts(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2) {
    m_origin_pts.clear();
    if (p1(1) > p2(1)) {
      m_origin_pts.emplace_back(p2);
      m_origin_pts.emplace_back(p1);
    } else {
      m_origin_pts.emplace_back(p1);
      m_origin_pts.emplace_back(p2);
    }
  }

  // TODO(liuzhuochun): change to template function
  std::vector<Eigen::Vector2d>
  getLinePoints(const std::vector<Eigen::Vector2d> &all_pts) {
    m_line_pts.clear();
    for (auto idx : m_inliner_index) {
      m_line_pts.emplace_back(all_pts[idx]);
    }
    return m_line_pts;
  }

  // return line slope degree 0-180 degree
  double getLineSlopeAngle() {
    double angle = atan(m_a / m_b) * 180 / M_PI;
    if (angle < 0)
      angle += 180;
    return angle;
  }

  // get point dist on line based on origin_pts
  double getPointDistOnLine(const Eigen::Vector2d &p) {
    Eigen::Vector2d line_vec = m_origin_pts[1] - m_origin_pts[0];
    Eigen::Vector2d pt_proj_vec = p - m_origin_pts[0];
    double dot_num = pt_proj_vec.transpose() * line_vec;
    double directed_dist = pt_proj_vec.norm();
    if (dot_num < 0)
      directed_dist *= -1;
    return directed_dist;
  }

  // project points on line
  Eigen::Vector2d projectPointOnLine(const Eigen::Vector2d &p0) {
    Eigen::Vector2d p1;
    if (m_b != 0) {
      double k = -m_a / m_b;
      double b = -m_c / m_b;
      p1(0) = (k * (p0(1) - b) + p0(0)) / (k * k + 1);
      p1(1) = k * p1(0) + b;
    } else {
      p1(0) = -m_c / m_a;
      p1(1) = p0(1);
    }
    return p1;
  }

  // compute distance from point to line
  double getPointLineDist(const Eigen::Vector2d &p) {
    double dist = fabs(m_a * p(0) + m_b * p(1) + m_c);
    dist /= sqrt(m_a * m_a + m_b * m_b);
    return dist;
  }

  // return degree
  double getLineLineAngle(std::shared_ptr<LineModel> a) {
    double angle1 = atan(m_a / m_b) * 180 / M_PI;
    double angle2 = atan(a->m_a / a->m_b) * 180 / M_PI;
    if (angle1 < 0)
      angle1 += 180;
    if (angle2 < 0)
      angle2 += 180;
    return fabs(angle1 - angle2);
  }

  // if angle between lines is larger than thresh, return -1
  double getLineLineDist(std::shared_ptr<LineModel> b, double angle_diff_th) {
    double angle_diff = this->getLineLineAngle(b);
    if (angle_diff > angle_diff_th)
      return -1;
    double dist = 0;
    // select endpoints and mid point to compute average dist
    dist += this->getPointLineDist(b->m_line_pts[0]);
    dist += this->getPointLineDist(b->m_line_pts[b->size() - 1]);
    dist += this->getPointLineDist(b->m_line_pts[(b->size() - 1) / 2]);
    dist /= 3;
    return dist;
  }

  // identify whether the point is on the left/right side of the line,
  // or on the line
  // return -1=left side, 0=on the line, 1=right side
  int getPointLineSide(const Eigen::Vector2d &p) {
    double dist = m_a * p(0) + m_b * p(1) + m_c;
    if (dist < 0)
      return -1;
    else if (dist == 0)
      return 0;
    else if (dist > 0)
      return 1;
    else
      return 0;
  }

  // sort line points (from left to right/from up to down)
  void sort(const std::vector<Eigen::Vector2d> &all_pts) {
    this->getLinePoints(all_pts);
    this->sort();
    this->getLinePoints(all_pts);
    sorted_ = true;
  }

  void getEndPoint(const std::vector<Eigen::Vector2d> &all_pts,
                   Eigen::Vector2d *endpt1, Eigen::Vector2d *endpt2) {
    if (!sorted_)
      this->sort(all_pts);
    *endpt1 = all_pts[m_inliner_index[0]];
    *endpt2 = all_pts[m_inliner_index[m_inliner_index.size() - 1]];
  }

  void getEndPoint(Eigen::Vector2d *endpt1, Eigen::Vector2d *endpt2) {
    if (!sorted_) {
      // LOGE("please sort line before get end-points.\n");
      return;
    }
    *endpt1 = m_line_pts[0];
    *endpt2 = m_line_pts[m_line_pts.size() - 1];
  }

  // cluster line points and delete outliners
  void clusterPoints(const std::vector<Eigen::Vector2d> &all_pts,
                     const double &max_dist_thresh) {
    if (!sorted_)
      this->sort(all_pts);
    // compute distance between points
    std::vector<double> dists;
    for (size_t i = 1; i < m_inliner_index.size(); i++) {
      dists.emplace_back((m_line_pts[i] - m_line_pts[i - 1]).norm());
    }
  }

  // return true if same index percent is higher than thresh
  bool compareIndex(const std::vector<int> &idx2, double thresh) {
    std::vector<int> index2 = idx2;
    double percent = 0;
    for (int a : m_inliner_index) {
      for (int b : index2) {
        if (a == b)
          percent++;
      }
    }
    if (m_inliner_index.size() > idx2.size())
      percent /= static_cast<double>(idx2.size());
    else
      percent /= static_cast<double>(m_inliner_index.size());

    if (percent > thresh)
      return true;
    else
      return false;
  }

  void copyFromSortedSegmentLine(std::shared_ptr<LineModel> origin_line,
                                 int start_pos, int end_pos) {
    if (!origin_line->sorted_) {
      // LOGE("line should be sorted before segmentation.\n");
      return;
    }
    m_a = origin_line->m_a;
    m_b = origin_line->m_b;
    m_c = origin_line->m_c;
    m_inliner_index =
        std::vector<int>(origin_line->m_inliner_index.begin() + start_pos,
                         origin_line->m_inliner_index.begin() + end_pos + 1);
    m_line_pts = std::vector<Eigen::Vector2d>(
        origin_line->m_line_pts.begin() + start_pos,
        origin_line->m_line_pts.begin() + end_pos + 1);
    sorted_ = true;
  }

  // segment line [start_pos, end_pos]
  void segment(int start_pos, int end_pos) {
    if (!sorted_) {
      // LOGE("line should be sorted before segmentation.\n");
      return;
    }
    int inliner_size = m_inliner_index.size();
    if (start_pos == 0 && end_pos + 1 == inliner_size)
      return;
    std::cout << "segment " << start_pos << "-" << end_pos << std::endl;
    std::vector<int> old_inliner_index(m_inliner_index);
    std::vector<Eigen::Vector2d> old_line_pts(m_line_pts);
    m_inliner_index = std::vector<int>(old_inliner_index.begin() + start_pos,
                                       old_inliner_index.begin() + end_pos + 1);
    m_line_pts = std::vector<Eigen::Vector2d>(
        old_line_pts.begin() + start_pos, old_line_pts.begin() + end_pos + 1);
    // NOTE: slope and origin point is not changed
  }

  /** cluster line points to get equidistant segmentations
   * return position of segments and mean point-to-point distance
   * of each segments.
   * pts_dist_th = prev p-to-p distance / next p-to-p distance - 1
   * eg: segment pos vector: [(1,9), (10,12), (13,18)]
   *     segment length vector: [9, 3, 6]
   *     segment mean dist: 30.3,30,30.2
  **/
  void equidistanceCluster(std::vector<Eigen::Vector2i> *segment_pos,
                           std::vector<int> *segment_length,
                           std::vector<double> *segment_dists,
                           double pts_dist_th = 0.5) {
    if (!sorted_) {
      // LOGE("line should be sorted before equidistanceCluster.\n");
      return;
    }
    segment_pos->clear();
    segment_dists->clear();
    // get point-to-point distance
    double prev_dist = (m_line_pts[1] - m_line_pts[0]).norm();
    double dist_mean = prev_dist;
    int pt_num = 1;
    int start_pos = 0;
    double c_dist = 0;
    for (size_t i = 1; i < m_line_pts.size() - 1; ++i) {
      c_dist = (m_line_pts[i + 1] - m_line_pts[i]).norm();
      double dist_diff_p =
          fabs(c_dist - dist_mean) / std::min(c_dist, dist_mean);
      if (dist_diff_p > pts_dist_th) {
        // split line
        int end_pos = i;
        // add new segment
        if (end_pos - start_pos >= 1) {
          segment_pos->emplace_back(Eigen::Vector2i(start_pos, end_pos));
          segment_length->emplace_back(end_pos - start_pos + 1);
          segment_dists->emplace_back(dist_mean);
        }
        dist_mean = c_dist;
        pt_num = 1;
        start_pos = end_pos;
      } else {
        dist_mean = (dist_mean * pt_num + c_dist) / (pt_num + 1);
        pt_num++;
      }
    }
    int end_pos = m_line_pts.size() - 1;
    // add final segment
    if (end_pos - start_pos >= 1) {
      // LOGI("c_dist=%f, dist=%f, cluster %d - %d\n", c_dist, dist_mean,
      // start_pos, end_pos);
      segment_pos->emplace_back(Eigen::Vector2i(start_pos, end_pos));
      segment_length->emplace_back(end_pos - start_pos + 1);
      segment_dists->emplace_back(dist_mean);
    }
  }

private:
  // sort line points (from left to right/from up to down)
  void sort() {
    // compute point-to-origin directed distance on lines;
    std::vector<double> pt_origin_dists;
    Eigen::Vector2d line_vec = m_origin_pts[1] - m_origin_pts[0];
    for (size_t i = 0; i < m_line_pts.size(); ++i) {
      Eigen::Vector2d pt_proj_vec =
          projectPointOnLine(m_line_pts[i]) - m_origin_pts[0];
      double dot_num = pt_proj_vec.transpose() * line_vec;
      double directed_dist = pt_proj_vec.norm();
      if (dot_num < 0)
        directed_dist *= -1;
      pt_origin_dists.emplace_back(directed_dist);
    }
    // sort line points based on point-to-origin directed distance
    std::vector<size_t> sort_index(pt_origin_dists.size());
    std::iota(sort_index.begin(), sort_index.end(), 0);
    std::sort(sort_index.begin(), sort_index.end(),
              [&pt_origin_dists](size_t i1, size_t i2) {
                return pt_origin_dists[i1] < pt_origin_dists[i2];
              });
    std::vector<int> old_inliner_index(m_inliner_index);
    for (size_t i = 0; i < m_inliner_index.size(); ++i) {
      m_inliner_index[i] = old_inliner_index[sort_index[i]];
    }
    // Note: need get line points again;
  }

public:
  // ax + by + c = 0
  double m_a;
  double m_b;
  double m_c;
  std::vector<int> m_inliner_index;
  // two points that build the line
  std::vector<Eigen::Vector2d> m_origin_pts;
  std::vector<Eigen::Vector2d> m_line_pts;
  // save data status
  bool sorted_;
};

#endif // APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_LINEMODEL_HPP_
