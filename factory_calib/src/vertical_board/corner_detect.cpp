/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#include "vertical_board/corner_detect.hpp"
#include "logging.hpp"
#include "utils/KeypointDetector.hpp"
#include "utils/RansacFitLine.hpp"
#include "utils/common.hpp"
#include <time.h>

namespace cameracalib {
namespace verticalBoard {

bool CornerDetector::detect(const std::vector<std::vector<float>> &gray_img,
                            const VerticalBoard &lp,
                            CornerPoints *detected_corner_points) {
  // set calibration line pattern
  line_pattern_ = lp;
  // corner detection parameters
  int max_corners = 500;
  double quality_level = 0.03;
  double min_distance = 15;

  std::vector<Eigen::Vector2d> corner_pts;
  std::vector<Point2f> corners;
  KeypointDetector point_detector(max_corners, quality_level, min_distance);
  if (!point_detector.detect(gray_img, &corners)) {
    return false;
  }
  for (size_t i = 0; i < corners.size(); ++i) {
    corner_pts.emplace_back(Eigen::Vector2d(corners[i].x, corners[i].y));
  }

  // detect line points
  std::vector<double> thresh_box({4, 5});
  int max_iter = corner_pts.size() * 10;

  for (size_t i = 0; i < thresh_box.size(); ++i) {
    double threshold = thresh_box[i]; // pixels
    LineConditions options;
    options.min_pts_num =
        std::max(static_cast<int>(line_pattern_.lines[1].size() / 2.0 - 1), 7);
    RansacFitLine line_detector(options, threshold, max_iter);
    std::vector<std::shared_ptr<LineModel>> line_models;
    if (!line_detector.Estimate(corner_pts, &line_models)) {
      std::cout << "failed to detect lines.\n";
      return false;
    }
    if (this->selectLinesByLineRelation(line_models, threshold,
                                        &detected_corner_points->lines_,
                                        &detected_corner_points->pattern_idxs_))
      return true;
  }
  return false;
}

void CornerDetector::displayDetectedLines(
    const cv::Mat &img,
    const std::vector<std::shared_ptr<LineModel>> &line_models) {
  std::vector<cv::Scalar> color_box;
  color_box.emplace_back(cv::Scalar(255, 0, 0));
  color_box.emplace_back(cv::Scalar(0, 255, 0));
  color_box.emplace_back(cv::Scalar(0, 0, 255));
  color_box.emplace_back(cv::Scalar(205, 200, 0));
  color_box.emplace_back(cv::Scalar(200, 0, 200));

  cv::Mat display_img = img.clone();
  for (size_t lidx = 0; lidx < line_models.size(); ++lidx) {
    std::vector<Eigen::Vector2d> l_pts = line_models[lidx]->m_line_pts;
    cv::Scalar color = color_box[lidx % 5];

    cv::Point2f s_pt, e_pt;
    s_pt.x = l_pts[0](0);
    s_pt.y = l_pts[0](1);
    e_pt.x = l_pts[l_pts.size() - 1](0);
    e_pt.y = l_pts[l_pts.size() - 1](1);
    cv::line(display_img, s_pt, e_pt, color, 2);

    for (auto p : l_pts) {
      cv::Point2f cv_pt;
      cv_pt.x = p(0);
      cv_pt.y = p(1);
      cv::circle(display_img, cv_pt, 4, color, -1);
    }
  }
  cv::imshow("all line", display_img);
  cv::waitKey(0);
}

bool CornerDetector::selectLinesByLineRelation(
    const std::vector<std::shared_ptr<LineModel>> &lines,
    const double &dist_thresh, // in pixels, same with ransac line threshold
    std::vector<std::shared_ptr<LineModel>> *selected_lines,
    std::vector<std::vector<int>> *pattern_idxs) {
  selected_lines->clear();
  if (lines.size() < 3) {
    // LOGE("Detected lines less than minum requirements.\n");
    return false;
  }
  // cluster three lines
  double angle_diff_th = param_.angle_diff_th;
  double dist_diff_th = param_.dist_diff_th;
  double min_dist_th = param_.min_dist_th;
  double max_dist_th = param_.max_dist_th;

  int point_num_score = 0;
  // find middle line
  for (size_t i1 = 0; i1 < lines.size(); ++i1) {
    std::vector<double> line_dists(lines.size());
    std::vector<size_t> left_line_idx;
    std::vector<size_t> right_line_idx;
    std::shared_ptr<LineModel> c_line = lines[i1];
    for (size_t i2 = 0; i2 < lines.size(); ++i2) {
      std::shared_ptr<LineModel> line2 = lines[i2];
      if (i1 == i2)
        continue;
      double dist = c_line->getLineLineDist(line2, angle_diff_th);
      if (dist < min_dist_th)
        continue;
      if (dist > max_dist_th)
        continue;
      line_dists[i2] = dist;
      int side = c_line->getPointLineSide(line2->m_line_pts[0]);
      if (side < 0)
        left_line_idx.emplace_back(i2);
      if (side > 0)
        right_line_idx.emplace_back(i2);
    }
    // find whether there are three clustered line
    if (left_line_idx.size() < 1 || right_line_idx.size() < 1)
      continue;
    std::vector<Eigen::Vector2i> left_right_idxs;
    if (!this->findRightLeftIdx(left_line_idx, right_line_idx, line_dists,
                                dist_diff_th * 1.5, &left_right_idxs))
      continue;

    // check whether three-line group meets requirements
    for (size_t lridx = 0; lridx < left_right_idxs.size(); ++lridx) {
      int l_line_idx = left_right_idxs[lridx](0);
      int r_line_idx = left_right_idxs[lridx](1);

      // LOGI("Find clustered line: %d-%d-%d", l_line_idx, i1,
      // r_line_idx);
      double estimated_pt_dist =
          (line_dists[l_line_idx] + line_dists[r_line_idx]) / 2;
      std::vector<Eigen::Vector3i> points_line_idx;
      // Segment lines to calibration line and find corresponding pattern
      if (this->checkLineCluster(
              c_line, lines[l_line_idx], lines[r_line_idx], estimated_pt_dist,
              dist_thresh, &points_line_idx, pattern_idxs, &point_num_score)) {
        // LOGW("Find line: %d-%d-%d\n", l_line_idx, i1, r_line_idx);
        Eigen::Vector3i start_idxs;
        Eigen::Vector3i end_idxs;
        this->findLinesStartEndPoints(points_line_idx, &start_idxs, &end_idxs);
        std::shared_ptr<LineModel> left_line = std::make_shared<LineModel>();
        left_line->copyFromSortedSegmentLine(lines[l_line_idx], start_idxs(0),
                                             end_idxs(0));
        std::shared_ptr<LineModel> mid_line = std::make_shared<LineModel>();
        mid_line->copyFromSortedSegmentLine(c_line, start_idxs(1), end_idxs(1));
        std::shared_ptr<LineModel> right_line = std::make_shared<LineModel>();
        right_line->copyFromSortedSegmentLine(lines[r_line_idx], start_idxs(2),
                                              end_idxs(2));
        selected_lines->emplace_back(left_line);
        selected_lines->emplace_back(mid_line);
        selected_lines->emplace_back(right_line);
        return true;
      }
    }
  }

  return false;
}

void CornerDetector::findLinesStartEndPoints(
    const std::vector<Eigen::Vector3i> &points_line_idx,
    Eigen::Vector3i *start_idxs, Eigen::Vector3i *end_idxs) {
  *start_idxs = Eigen::Vector3i(-1, -1, -1);
  *end_idxs = Eigen::Vector3i(-1, -1, -1);
  for (size_t s = 0; s < points_line_idx.size() - 1; ++s) {
    Eigen::Vector3i point = points_line_idx[s];
    Eigen::Vector3i next_point = points_line_idx[s + 1];
    for (size_t i = 0; i < 3; i++) {
      if (point(i) != -1) {
        if ((*start_idxs)(i) == -1)
          (*start_idxs)(i) = point(i);
        if (next_point(i) == -1)
          (*end_idxs)(i) = point(i);
      }
      if (next_point(i) != -1 && s == points_line_idx.size() - 2)
        (*end_idxs)(i) = next_point(i);
    }
  }
}

bool CornerDetector::findRightLeftIdx(
    const std::vector<size_t> &left_line_idx,
    const std::vector<size_t> &right_line_idx,
    const std::vector<double> &line_dists, const double &dist_diff_th,
    std::vector<Eigen::Vector2i> *left_right_idxs) {
  left_right_idxs->clear();
  for (size_t li = 0; li < left_line_idx.size(); ++li) {
    for (size_t ri = 0; ri < right_line_idx.size(); ++ri) {
      int l_idx = left_line_idx[li];
      int r_idx = right_line_idx[ri];
      double dist_diff = fabs(line_dists[l_idx] - line_dists[r_idx]) /
                         std::min(line_dists[l_idx], line_dists[r_idx]);
      if (dist_diff < dist_diff_th)
        left_right_idxs->emplace_back(Eigen::Vector2i(l_idx, r_idx));
    }
  }
  if (left_right_idxs->size() == 0)
    return false;
  else
    return true;
}

bool CornerDetector::checkLineCluster(
    const std::shared_ptr<LineModel> mid_line,
    const std::shared_ptr<LineModel> left_line,
    const std::shared_ptr<LineModel> right_line,
    const double &estimated_pt_dist, // based on line-line dist
    const double &dist_th,           // in pixels
    // point index of mid/left/right line on joint line
    // index = -1, if mid/left/right line has no projection
    std::vector<Eigen::Vector3i> *points_line_idx,
    std::vector<std::vector<int>> *pattern_idxs, int *point_num_score) {
  points_line_idx->clear();
  // percent, point-to-point thresh
  double dist_diff_th = param_.dist_diff_th;

  // project left and right lines to middle line
  std::vector<Eigen::Vector2d> joint_line;
  std::vector<Eigen::Vector3i> rough_points_line_idx;
  this->projectJointLine(mid_line, left_line, right_line, dist_th * 2,
                         &joint_line, &rough_points_line_idx);

  // first and last corner must be detected after projection
  int joint_point_num = joint_line.size();
  // double total_len_diff_th = dist_diff_th/3.0;
  double estimated_total_len =
      (line_pattern_.lines[1].size() - 1) * estimated_pt_dist;
  double min_total_len_dist = estimated_total_len * (1.0 - dist_diff_th / 3.0);
  double max_total_len_dist = estimated_total_len * (1.0 + dist_diff_th / 3.0);
  // skip short lines
  if ((joint_line[joint_point_num - 1] - joint_line[0]).norm() <
      min_total_len_dist)
    return false;
  // compute point-to-point distance of joint line
  std::vector<double> dists;
  std::vector<double> distance_index;
  // 0: distance between (i,i+1) is not multiples of estimated grid distance
  // other: distance between (i,i+1) is similar to x multiples of estimated
  // grid distance
  std::vector<int> grid_dist_vec;
  double current_dist = 0;
  distance_index.emplace_back(current_dist);
  for (size_t i = 0; i < joint_line.size() - 1; ++i) {
    double dist = (joint_line[i + 1] - joint_line[i]).norm();
    // if distance in range of estimated equal dist
    double multiple = dist / estimated_pt_dist;
    int floor_multiple = std::floor(multiple);
    if (multiple - floor_multiple < dist_diff_th) {
      grid_dist_vec.emplace_back(floor_multiple);
    } else if (floor_multiple + 1 - multiple < dist_diff_th) {
      grid_dist_vec.emplace_back(floor_multiple + 1);
    } else {
      grid_dist_vec.emplace_back(0);
    }
    dists.emplace_back(dist);
    current_dist += dist;
    distance_index.emplace_back(current_dist);
  }
  // use total length of corner point in middle line to find possible end
  // points
  // [start_idx, end_idx]
  int max_mid_point = line_pattern_.lines[1].size();
  int max_point_num = 0;
  int real_start_pos, real_end_pos;
  for (int s_idx = 0; s_idx < joint_point_num - 2; ++s_idx) {
    int end_max = std::min(s_idx + max_mid_point - 1, joint_point_num - 1);
    for (int e_idx = s_idx + 1; e_idx <= end_max; ++e_idx) {
      double segment_dist = distance_index[e_idx] - distance_index[s_idx];
      if (segment_dist > max_total_len_dist)
        break;
      if (segment_dist < min_total_len_dist) {
        if (grid_dist_vec[e_idx] == 0) {
          s_idx = e_idx;
          break;
        }
        continue;
      }

      // found possible start and end pos of calibration line
      std::vector<std::vector<int>> current_pattern_idxs;
      int current_point_num = 0;
      if (this->checkProjectLinePattern(grid_dist_vec, rough_points_line_idx,
                                        s_idx, e_idx, &current_pattern_idxs,
                                        &current_point_num)) {
        if (current_point_num > max_point_num) {
          *pattern_idxs = current_pattern_idxs;
          max_point_num = current_point_num;
          real_start_pos = s_idx;
          real_end_pos = e_idx;
        }
      }
    }
  }
  if (max_point_num < 4)
    return false;
  *point_num_score = max_point_num;
  *points_line_idx = std::vector<Eigen::Vector3i>(
      rough_points_line_idx.begin() + real_start_pos,
      rough_points_line_idx.begin() + real_end_pos + 1);

  return true;
}

bool CornerDetector::checkProjectLinePattern(
    const std::vector<int> &grid_dist_vec,
    const std::vector<Eigen::Vector3i> &points_line_idx, const int &start_pos,
    const int &end_pos, std::vector<std::vector<int>> *pattern_idxs,
    int *point_num) {
  int grid_idx = 0;
  int line_corner_num = line_pattern_.lines[1].size();
  pattern_idxs->resize(3);
  std::vector<int> line_pt_num(3);
  for (int i = start_pos; i < end_pos; ++i) {
    if (grid_dist_vec[i] == 0)
      return false;
    Eigen::Vector3i pt_line_idx = points_line_idx[i];
    for (size_t l = 0; l < 3; ++l) {
      if (pt_line_idx(l) != -1) {
        // there should be no point on current pos
        if (line_pattern_.lines[l][grid_idx] == 0)
          return false;
        line_pt_num[l]++;
        (*pattern_idxs)[l].emplace_back(grid_idx);
      }
    }
    grid_idx += grid_dist_vec[i];
    if (grid_idx >= line_corner_num) {
      return false;
    }
  }
  // add last point
  Eigen::Vector3i pt_line_idx = points_line_idx[end_pos];
  for (size_t l = 0; l < 3; ++l) {
    if (pt_line_idx(l) != -1) {
      // there should be no point on current pos
      if (line_pattern_.lines[l][grid_idx] == 0)
        return false;
      line_pt_num[l]++;
      (*pattern_idxs)[l].emplace_back(grid_idx);
    }
  }

  // each line must have corners no less than mid corners/3 corner
  int min_corner_num = line_pattern_.lines[1].size() / 3;
  for (size_t l = 0; l < 3; ++l) {
    if (line_pt_num[l] < min_corner_num)
      return false;
    (*point_num) += line_pt_num[l];
  }

  return true;
}

// project left and right line to the middle line
void CornerDetector::projectJointLine(
    const std::shared_ptr<LineModel> mid_line,
    const std::shared_ptr<LineModel> left_line,
    const std::shared_ptr<LineModel> right_line,
    const double &dist_th, // same as threshold(in pixel)
    std::vector<Eigen::Vector2d> *joint_line,
    std::vector<Eigen::Vector3i> *points_line_idx) {
  joint_line->clear();
  points_line_idx->clear();
  // project left line and get point dist on mid line
  std::vector<Eigen::Vector2d> proj_left_line(left_line->m_line_pts);
  std::vector<double> left_dist(left_line->size());
  for (size_t i = 0; i < proj_left_line.size(); i++) {
    proj_left_line[i] = mid_line->projectPointOnLine(proj_left_line[i]);
    left_dist[i] = mid_line->getPointDistOnLine(proj_left_line[i]);
  }
  // project right line and get point dist on mid line
  std::vector<Eigen::Vector2d> proj_right_line(right_line->m_line_pts);
  std::vector<double> right_dist(right_line->size());
  for (size_t i = 0; i < proj_right_line.size(); i++) {
    proj_right_line[i] = mid_line->projectPointOnLine(proj_right_line[i]);
    right_dist[i] = mid_line->getPointDistOnLine(proj_right_line[i]);
  }
  // get point dist on mid line
  std::vector<Eigen::Vector2d> proj_mid_line(mid_line->m_line_pts);
  std::vector<double> mid_dist(mid_line->size());
  for (size_t i = 0; i < mid_dist.size(); i++) {
    proj_mid_line[i] = mid_line->projectPointOnLine(proj_mid_line[i]);
    mid_dist[i] = mid_line->getPointDistOnLine(proj_mid_line[i]);
  }
  // join lines together
  // left, mid, right
  std::vector<size_t> pos_idxs(3);
  std::vector<std::shared_ptr<LineModel>> lines;
  lines.emplace_back(left_line);
  lines.emplace_back(mid_line);
  lines.emplace_back(right_line);
  std::vector<std::vector<double>> dists;
  dists.emplace_back(left_dist);
  dists.emplace_back(mid_dist);
  dists.emplace_back(right_dist);
  std::vector<std::vector<Eigen::Vector2d>> proj_pts;
  proj_pts.emplace_back(proj_left_line);
  proj_pts.emplace_back(proj_mid_line);
  proj_pts.emplace_back(proj_right_line);

  while (pos_idxs[0] < lines[0]->size() || pos_idxs[1] < lines[1]->size() ||
         pos_idxs[2] < lines[2]->size()) {
    std::vector<double> c_dist(3, {1e15});
    for (size_t i = 0; i < 3; ++i) {
      if (pos_idxs[i] < lines[i]->size())
        c_dist[i] = dists[i][pos_idxs[i]];
    }
    Eigen::Vector3i pt_line_idx(-1, -1, -1);
    int min_pos =
        std::min_element(c_dist.begin(), c_dist.end()) - c_dist.begin();
    int joint_pt_num = 1;
    double min_dist = c_dist[min_pos];
    Eigen::Vector2d c_point = proj_pts[min_pos][pos_idxs[min_pos]];

    pt_line_idx(min_pos) = pos_idxs[min_pos];
    pos_idxs[min_pos]++;
    for (int i = 0; i < 3; ++i) {
      if (i == min_pos)
        continue;
      // whether points project together
      if (fabs(c_dist[i] - min_dist) < dist_th) {
        pt_line_idx(i) = pos_idxs[i];
        c_point = (c_point * joint_pt_num + proj_pts[i][pos_idxs[i]]) /
                  static_cast<double>(joint_pt_num + 1);
        pos_idxs[i]++;
        joint_pt_num++;
      }
    }

    joint_line->emplace_back(c_point);
    points_line_idx->emplace_back(pt_line_idx);
  }
}

} // namespace verticalBoard
} // namespace cameracalib
