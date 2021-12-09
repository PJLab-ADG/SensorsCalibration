/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#include "circle_board/corner_detect.hpp"
#include "utils/LineModel.hpp"
#include "utils/RansacFitLine.hpp"
#include "utils/corner.hpp"

namespace cameracalib {
namespace circleBoard {

bool CornerDetector::detect(const std::vector<std::vector<float>> &gray_img,
                            const CircleBoard &board_pattern,
                            CornerPoints *corner_pts) {
  int img_h = gray_img.size();
  if (img_h < 1)
    return false;
  int img_w = gray_img[0].size();
  if (img_w < 1)
    return false;
  CircleDetector circle_detector;
  circle_detector.generateParam(img_w, img_h);
  min_line_dist_ = circle_detector.min_radius_ * 4;
  max_line_dist_ = circle_detector.max_radius_ * 6;
  std::vector<int> thresh_box;
  this->EstimateGrayThres(gray_img, &thresh_box);

  board_pattern_ = board_pattern;
  int min_circle_num = 3 * 4;

  for (size_t i = 0; i < thresh_box.size(); ++i) {
    corner_pts->points.resize(board_pattern_.height,
                              std::vector<Point2f>(board_pattern_.width));
    int thres = thresh_box[i];
    std::vector<std::vector<int>> thresh_img;
    imgproc::binaryThreshold(gray_img, &thresh_img, thres);

    std::vector<Circle> circles;
    if (!circle_detector.detect(thresh_img, &circles))
      continue;
    if (static_cast<int>(circles.size()) < min_circle_num)
      continue;
    // std::cout << "detect " << circles.size() << " circles\n";
    if (this->findCirclePattern(circles, corner_pts)) {
      return true;
    }
  }
  return false;
}

void CornerDetector::EstimateGrayThres(
    const std::vector<std::vector<float>> &img_gray,
    std::vector<int> *thresh_box) {
  thresh_box->clear();
  int img_h = img_gray.size();
  int img_w = img_gray[0].size();

  float aver_gray = 0;
  float mid_gray = 0;

  std::vector<int> gray_pixel_num(256);
  std::vector<float> gray_prob(256);
  std::vector<float> gray_distribution(256);
  int pixel_num = 0;

  for (int i = 0; i < img_h; ++i) {
    for (int j = 0; j < img_w; ++j) {
      aver_gray = (aver_gray * pixel_num + img_gray[i][j]) /
                  static_cast<float>(pixel_num + 1);
      gray_pixel_num[static_cast<int>(img_gray[i][j])]++;
      pixel_num++;
    }
  }
  for (int i = 0; i < 256; i++) {
    gray_prob[i] = gray_pixel_num[i] / static_cast<float>(pixel_num);
  }
  gray_distribution[0] = gray_prob[0];
  for (int i = 1; i < 256; i++) {
    gray_distribution[i] = gray_distribution[i - 1] + gray_prob[i];
    if (gray_distribution[i - 1] <= 0.5 && gray_distribution[i] >= 0.5)
      mid_gray = i;
  }

  int start_thres = static_cast<int>(aver_gray);
  if (aver_gray > 127.5)
    start_thres = static_cast<int>(mid_gray + mid_gray - aver_gray);
  else
    start_thres = static_cast<int>(aver_gray + aver_gray - mid_gray);
  int stride = static_cast<int>((127.5 - abs(127.5 - aver_gray)) *
                                (1.0 -
                                 (127.5 - aver_gray) * (127.5 - mid_gray) /
                                     static_cast<float>(127.5 * 127.5)) /
                                6.0);
  if (aver_gray > 127.5 && (mid_gray - aver_gray) < 0)
    stride = stride * (-1);
  if (aver_gray < 127.5 && (mid_gray - aver_gray) > 0)
    stride = stride * (-1);

  int iter = static_cast<int>(256.0 / static_cast<float>(abs(stride)));
  thresh_box->push_back((start_thres - stride + 256) % 256);
  thresh_box->push_back(start_thres);
  for (int i = 2; i <= iter; i++) {
    if (size_t(i - 1) >= thresh_box->size())
      break;
    if ((*thresh_box)[i - 1] + stride >= 255 ||
        (*thresh_box)[i - 1] + stride < 0) {
      thresh_box->push_back(start_thres - 2 * stride);
      stride *= -1;
    } else {
      thresh_box->push_back((*thresh_box)[i - 1] + stride);
    }
  }
}

bool CornerDetector::findCirclePattern(const std::vector<Circle> &circles,
                                       CornerPoints *corner_pts) {
  // ransac to get circle lines
  int max_iter = circles.size() * 10;
  std::vector<Eigen::Vector2d> center_points;
  for (auto circle : circles) {
    // lateral line, inverse the x/y coordinate
    center_points.emplace_back(
        Eigen::Vector2d(circle.center.x, circle.center.y));
  }
  // LineConditions options;
  options_.min_pts_num = std::max(board_pattern_.height - 2, 3);
  RansacFitLine line_detector(options_, dist_diff_, max_iter);
  std::vector<std::shared_ptr<LineModel>> line_models;

  if (!line_detector.Estimate(center_points, &line_models)) {
    std::cout << "failed to detect circle lines.\n";
    return false;
  }
  return this->clusterLines(circles, line_models, corner_pts);
}

bool CornerDetector::IsBlackCircleLine(const std::vector<int> &circle_idx,
                                       const std::vector<Circle> &circles,
                                       std::vector<int> *black_circle_idxs) {
  black_circle_idxs->clear();
  for (auto i : circle_idx) {
    if (circles[i].black) {
      black_circle_idxs->emplace_back(i);
    }
  }
  if (black_circle_idxs->size() > 0)
    return true;
  return false;
}

bool CornerDetector::clusterLines(
    const std::vector<Circle> &circles,
    const std::vector<std::shared_ptr<LineModel>> &line_models,
    CornerPoints *corner_pts) {
  std::vector<BlackLines> possible_black_lines;
  if (!this->findTwoBlackLines(circles, line_models, &possible_black_lines))
    return false;

  std::vector<int> cluster_lines_idxs(board_pattern_.width);
  int mid_index = board_pattern_.width / 2;
  int line_num = line_models.size();
  int min_valid_line_num = std::min(2, board_pattern_.width - 2);
  for (auto black_pair : possible_black_lines) {
    // cluster parallel lines
    // clear cluster line index
    for (int a = 0; a < board_pattern_.width; ++a)
      cluster_lines_idxs[a] = -1;
    cluster_lines_idxs[mid_index - 1] = black_pair.left_line_idx;
    cluster_lines_idxs[mid_index] = black_pair.right_line_idx;
    float estimated_dist = black_pair.estimated_line_dist;
    auto left_line = line_models[black_pair.left_line_idx];
    int valid_line_num = 0;

    for (int i = 0; i < line_num; ++i) {
      if (i == black_pair.left_line_idx)
        continue;
      if (i == black_pair.right_line_idx)
        continue;
      auto c_line = line_models[i];

      // whether parallel
      float line_dist =
          left_line->getLineLineDist(c_line, line_max_angle_diff_);
      if (line_dist < 0)
        continue;

      // whether is a multiple of estimated line dist
      float mul = line_dist / estimated_dist;
      int multiple = std::round(mul);

      if (multiple < 1)
        continue;
      if (std::fabs(mul - multiple) > dist_diff_percent_)
        continue;

      // determine if it's left or right
      int side = left_line->getPointLineSide(c_line->m_line_pts[0]);
      if (side == 0)
        continue;
      int line_pos_idx = multiple * side + mid_index - 1;

      if (line_pos_idx < 0 || line_pos_idx >= board_pattern_.width ||
          line_pos_idx == mid_index || line_pos_idx == mid_index - 1)
        continue;
      cluster_lines_idxs[line_pos_idx] = i;
      valid_line_num++;
    }
    // must find 4 parallel lines
    if (valid_line_num < min_valid_line_num)
      continue;

    for (size_t bidx = 0; bidx < black_pair.left_blackcircleidx.size();
         ++bidx) {
      // check center point pattern and remove outliers in lines
      int lb_idx = black_pair.left_blackcircleidx[bidx];
      int rb_idx = black_pair.right_blackcircleidx[bidx];
      if (this->checkLinePattern(circles, lb_idx, rb_idx, cluster_lines_idxs,
                                 line_models, corner_pts))
        return true;
    }
  }
  return false;
}

bool CornerDetector::findTwoBlackLines(
    const std::vector<Circle> &circles,
    const std::vector<std::shared_ptr<LineModel>> &line_models,
    std::vector<BlackLines> *possible_black_lines) {
  std::vector<std::vector<int>> black_circle_idxs;
  std::vector<size_t> black_line_idxs;
  int black_num = 0;
  for (size_t i = 0; i < line_models.size(); ++i) {
    std::vector<int> black_circle_index;
    if (this->IsBlackCircleLine(line_models[i]->m_inliner_index, circles,
                                &black_circle_index)) {
      black_num++;
      black_line_idxs.emplace_back(i);
      black_circle_idxs.emplace_back(black_circle_index);
    }
  }
  // find middle two black line
  if (black_num < 2)
    return false;

  possible_black_lines->clear();
  for (int i = 0; i < black_num - 1; ++i) {
    int a_idx = black_line_idxs[i];
    auto a_line = line_models[a_idx];
    for (int j = i + 1; j < black_num; ++j) {
      int b_idx = black_line_idxs[j];
      auto b_line = line_models[b_idx];
      // whether two black line is parallel, dist = -1 if not parallel
      float line_dist = a_line->getLineLineDist(b_line, line_max_angle_diff_);
      if (line_dist > min_line_dist_ && line_dist < max_line_dist_) {
        // classify left right line
        int left_idx, right_idx;
        int side = a_line->getPointLineSide(b_line->m_line_pts[0]);
        if (side == -1) {
          left_idx = b_idx;
          right_idx = a_idx;
        } else if (side == 1) {
          left_idx = a_idx;
          right_idx = b_idx;
        } else {
          continue;
        }
        // whether black circle can project on the same point
        std::vector<int> a_blackcircleidx;
        std::vector<int> b_blackcircleidx;
        float min_r = line_dist / 8;
        float max_r = line_dist / 4;
        for (size_t a = 0; a < black_circle_idxs[i].size(); ++a) {
          Point2f a_center = circles[black_circle_idxs[i][a]].center;
          float ra = circles[black_circle_idxs[i][a]].radius;
          if (ra < min_r || ra > max_r)
            continue;
          Eigen::Vector2d a_project = b_line->projectPointOnLine(
              Eigen::Vector2d(a_center.x, a_center.y));
          for (size_t b = 0; b < black_circle_idxs[j].size(); ++b) {
            Point2f b_center = circles[black_circle_idxs[j][b]].center;
            float rb = circles[black_circle_idxs[j][b]].radius;
            if (rb < min_r || rb > max_r)
              continue;
            // compare radius
            float r_diff = fabs(ra - rb);
            if (r_diff > dist_diff_)
              continue;
            // compute project point dist
            Eigen::Vector2d b_pt(b_center.x, b_center.y);
            double dist_diff_square = (b_pt - a_project).norm();
            // TODO(liuzhuochun) : !!!!!!!!!!!!!!!!!
            // found corresponding black circles that can project on
            // same pos
            if (dist_diff_square < 10) {
              a_blackcircleidx.emplace_back(black_circle_idxs[i][a]);
              b_blackcircleidx.emplace_back(black_circle_idxs[j][b]);
            }
          }
        }
        // push back black line pair
        if (a_blackcircleidx.size() > 0) {
          BlackLines black_pair;
          black_pair.left_line_idx = left_idx;
          black_pair.right_line_idx = right_idx;
          black_pair.estimated_line_dist = line_dist;
          if (side == -1) {
            black_pair.left_blackcircleidx = b_blackcircleidx;
            black_pair.right_blackcircleidx = a_blackcircleidx;
          } else {
            black_pair.left_blackcircleidx = a_blackcircleidx;
            black_pair.right_blackcircleidx = b_blackcircleidx;
          }
          possible_black_lines->emplace_back(black_pair);
        }
      }
    }
  }

  if (possible_black_lines->size() < 1)
    return false;
  return true;
}

bool CornerDetector::checkLinePattern(
    const std::vector<Circle> &circles, const int &left_black_circle_idx,
    const int &right_black_circle_idx, const std::vector<int> &line_pos_index,
    const std::vector<std::shared_ptr<LineModel>> &line_models,
    CornerPoints *corner_pts) {
  corner_pts->line_slope.clear();
  int mid_index = board_pattern_.width / 2;
  auto left_bline = line_models[line_pos_index[mid_index - 1]];
  auto right_bline = line_models[line_pos_index[mid_index]];
  auto left_bc = circles[left_black_circle_idx];
  auto right_bc = circles[right_black_circle_idx];
  Eigen::Vector2d l_bc(left_bc.center.x, left_bc.center.y);

  float estimated_big_radius = (left_bc.radius + right_bc.radius) / 2.0;
  estimated_big_radius *= board_pattern_.big_r / board_pattern_.small_r;
  float estimated_pt_dist =
      estimated_big_radius * board_pattern_.pt_dist / board_pattern_.big_r;

  int hmul = (board_pattern_.height - 1) / 2;
  for (int i = 0; i < static_cast<int>(line_pos_index.size()); ++i) {
    int lidx = line_pos_index[i];
    if (lidx == -1)
      continue;
    int valid_pt_num = 0;
    auto c_line = line_models[lidx];
    int pt_num = c_line->size();
    Eigen::Vector2d project_bc = c_line->projectPointOnLine(l_bc);
    float left_bc_dist = c_line->getPointDistOnLine(project_bc);

    corner_pts->line_slope.emplace_back(c_line->getLineSlopeAngle());

    for (int p = 0; p < pt_num; ++p) {
      int circle_idx = c_line->m_inliner_index[p];
      // skip black circles
      if ((i == mid_index - 1 && circle_idx == left_black_circle_idx) ||
          (i == mid_index && circle_idx == right_black_circle_idx)) {
        valid_pt_num++;
        continue;
      }
      auto c_circle = circles[circle_idx];
      Eigen::Vector2d pt(c_circle.center.x, c_circle.center.y);
      // compute point-to-blackcircle distance
      float pt_dist = c_line->getPointDistOnLine(pt);
      float pt_c_dist = pt_dist - left_bc_dist;

      float mul = std::fabs(pt_c_dist) / estimated_pt_dist;
      int multiple = std::round(mul);
      if (multiple > hmul)
        continue;
      float diff = std::fabs(multiple - mul);
      // check dist whether is multiple of estimated pt dist
      if (diff < dist_diff_percent_) {
        // check whether radius is valid
        float radius_diff = std::fabs(c_circle.radius - estimated_big_radius);
        if (radius_diff > dist_diff_)
          return false;
        // check whether white circle
        if (c_circle.black)
          return false;

        int row = multiple;
        if (pt_c_dist < 0)
          row *= -1;
        row += hmul;
        corner_pts->points[row][i] = c_circle.center;
        valid_pt_num++;
      } else {
        return false;
      }
    }
    // check whether there's enough valid points
    if (valid_pt_num < options_.min_pts_num)
      return false;
  }
  corner_pts->points[2][mid_index - 1] = left_bc.center;
  corner_pts->points[2][mid_index] = right_bc.center;
  return true;
}

} // namespace circleBoard
} // namespace cameracalib
