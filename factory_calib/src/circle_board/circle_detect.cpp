/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */

#include "circle_board/circle_detect.hpp"
#include "utils/corner.hpp"

namespace cameracalib {
namespace circleBoard {

bool CircleDetector::detect(const std::vector<std::vector<int>> &thresh_img,
                            std::vector<Circle> *circles) {
  circles->clear();

  // find contours
  std::vector<Contour> contours;
  if (!this->FindContours(thresh_img, &contours))
    return false;
  // pick circle contours
  for (size_t i = 0; i < contours.size(); ++i) {
    Circle circle;
    if (this->isCircle(contours[i], &circle)) {
      // check whether this circle is black circle
      if (thresh_img[std::round(circle.center.y)]
                    [std::round(circle.center.x)] == 0) {
        circle.black = true;
      }
      circles->emplace_back(circle);
    }
  }
  if (circles->size() < 1)
    return false;
  return true;
}

bool CircleDetector::FindContours(const std::vector<std::vector<int>> &img,
                                  std::vector<Contour> *contours) {
  contours->clear();
  std::vector<std::vector<int>> img_tmp = img;
  size_t height = img.size();
  size_t width = img[0].size();

  for (size_t i = 1; i < height - 1; ++i) {
    for (size_t j = 1; j < width - 1; ++j) {
      if (img[i + 1][j] == 0 && img[i + 1][j - 1] == 0 && img[i][j - 1] == 0 &&
          img[i - 1][j - 1] == 0 && img[i - 1][j] == 0 &&
          img[i - 1][j + 1] == 0 && img[i][j + 1] == 0 &&
          img[i + 1][j + 1] == 0) {
        img_tmp[i][j] = 255;
      }
    }
  }
  for (size_t i = 1; i < height - 1; ++i) {
    for (size_t j = 1; j < width - 1; ++j) {
      if (img_tmp[i][j] == 0) {
        Contour contour;
        this->DepthFirstSearch(&img_tmp, &(contour.points), &(contour.center.x),
                               &(contour.center.y), i, j);
        int pt_size = contour.points.size();
        if (pt_size < max_contour_pt_num_ && pt_size > min_contour_pt_num_)
          contours->emplace_back(contour);
      }
    }
  }
  return true;
}

void CircleDetector::DepthFirstSearch(std::vector<std::vector<int>> *img,
                                      std::vector<Point2i> *contour_points,
                                      float *center_x, float *center_y,
                                      int posi, int posj) {
  contour_points->clear();
  std::stack<Point2i> S;
  S.push(Point2i(posi, posj));
  (*img)[posi][posj] = 0;
  int height = img->size();
  int width = (*img)[0].size();
  *center_x = 0, *center_y = 0;

  // contour_points.emplace_back(Point2i(posj, posi));
  // center_x += posj;
  // center_y += posi;
  while (!S.empty()) {
    Point2i node = S.top();
    int i = node.x;
    int j = node.y;
    S.pop();
    if ((i + 1) < height && (*img)[i + 1][j] == 0) {
      (*img)[i + 1][j] = 255;
      S.push(Point2i(i + 1, j));
      contour_points->emplace_back(Point2i(j, i + 1));
      *center_x += j;
      *center_y += i + 1;
    }
    if ((j + 1) < width && (*img)[i][j + 1] == 0) {
      (*img)[i][j + 1] = 255;
      S.push(Point2i(i, j + 1));
      contour_points->emplace_back(Point2i(j + 1, i));
      *center_x += j + 1;
      *center_y += i;
    }
    if ((i - 1) >= 0 && (*img)[i - 1][j] == 0) {
      (*img)[i - 1][j] = 255;
      S.push(Point2i(i - 1, j));
      contour_points->emplace_back(Point2i(j, i - 1));
      *center_x += j;
      *center_y += i - 1;
    }
    if ((j - 1) >= 0 && (*img)[i][j - 1] == 0) {
      (*img)[i][j - 1] = 255;
      S.push(Point2i(i, j - 1));
      contour_points->emplace_back(Point2i(j - 1, i));
      *center_x += j - 1;
      *center_y += i;
    }
  }
  float ptsize = static_cast<float>(contour_points->size());
  if (ptsize > 1) {
    *center_x /= ptsize;
    *center_y /= ptsize;
  }
}

void CircleDetector::generateParam(const int &img_w, const int &img_h) {
  min_contour_pt_num_ = 80 * img_w / 1920;
  max_contour_pt_num_ = 900 * img_w / 1920;
  min_contour_pt_dist_ = 20 * img_w / 1920;
  max_contour_pt_dist_ = 300 * img_w / 1920;
  min_radius_ = 10.0 * img_w / 1920;
  max_radius_ = 100.0 * img_w / 1920;
}

bool CircleDetector::isCircle(const Contour &contour, Circle *circle) {
  auto pts = contour.points;
  float cx = contour.center.x;
  float cy = contour.center.y;

  float min_dist = 1e09;
  float max_dist = 0;
  float radius = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    float dx = pts[i].x - cx;
    float dy = pts[i].y - cy;
    float dist = std::sqrt(dx * dx + dy * dy);
    radius += dist;
    if (dist > max_dist) {
      max_dist = dist;
      if (dist > max_radius_)
        return false;
      if (min_dist != 0) {
        float dist_diff = max_dist / min_dist - 1;
        if (dist_diff > circle_diff_thresh_)
          return false;
      }
    }
    if (dist < min_dist) {
      min_dist = dist;
      if (dist < min_radius_)
        return false;
      if (min_dist != 0) {
        float dist_diff = max_dist / min_dist - 1;
        if (dist_diff > circle_diff_thresh_)
          return false;
      }
    }
  }
  radius /= static_cast<float>(pts.size());
  *circle = Circle(contour.center, radius, pts);
  return true;
}

} // namespace circleBoard
} // namespace cameracalib
