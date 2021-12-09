/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#include "utils/KeypointDetector.hpp"

struct greaterThanPtr {
  // Ensure a fully deterministic result of the sort
  bool operator()(const Point2f a, const Point2f b) const {
    return (a.y > b.y) ? true : (a.y < b.y) ? false : (a.x > b.x);
  }
};

bool KeypointDetector::detect(const std::vector<std::vector<float>> &gray_img,
                              std::vector<Point2f> *out_corners) {
  std::vector<std::vector<float>> eig, tmp;
  imgproc::cornerMinEigenVal(gray_img, &eig, block_size_, gradient_size_);
  double maxVal = 0;
  // find max value
  imgproc::minMaxLoc(eig, &maxVal);
  imgproc::threshold(&eig, maxVal * quality_level_, 0);
  imgproc::cornerDilate(&eig); // different with opencv dilate

  int height = gray_img.size();
  int width = gray_img[0].size();
  std::vector<Point2f> tmpCorners;
  for (int y = 1; y < height - 1; ++y) {
    auto eig_data = eig[y].begin();
    // TODO(liuzhuochun): Mask
    for (int x = 1; x < width - 1; ++x) {
      float val = eig_data[x];
      if (val != 0) {
        tmpCorners.emplace_back(Point2f(y * width + x, val));
      }
    }
  }

  std::vector<Point2f> corners;
  std::vector<float> cornersQuality;
  size_t total = tmpCorners.size();
  size_t i, j;
  size_t ncorners = 0;
  if (total == 0)
    return false;
  // LOGI("total corners %d.\n", total);
  std::sort(tmpCorners.begin(), tmpCorners.end(), greaterThanPtr());

  if (min_distance_ >= 1) {
    // patition the image into larger grids
    int w = width;
    int h = height;

    const int cell_size = std::round(min_distance_);
    const int grid_width = (w + cell_size - 1) / cell_size;
    const int grid_height = (h + cell_size - 1) / cell_size;

    std::vector<std::vector<Point2f>> grid(grid_width * grid_height);
    double min_dist = min_distance_ * min_distance_;

    for (i = 0; i < total; ++i) {
      int ofs = tmpCorners[i].x;
      int y = ofs / width;
      int x = ofs % width;

      bool good = true;

      int x_cell = x / cell_size;
      int y_cell = y / cell_size;

      int x1 = x_cell - 1;
      int y1 = y_cell - 1;
      int x2 = x_cell + 1;
      int y2 = y_cell + 1;

      // boundary check
      x1 = std::max(0, x1);
      y1 = std::max(0, y1);
      x2 = std::min(grid_width - 1, x2);
      y2 = std::min(grid_height - 1, y2);

      for (int yy = y1; yy <= y2; ++yy) {
        for (int xx = x1; xx <= x2; ++xx) {
          std::vector<Point2f> &m = grid[yy * grid_width + xx];
          if (m.size()) {
            for (j = 0; j < m.size(); ++j) {
              float dx = x - m[j].x;
              float dy = y - m[j].y;
              if (dx * dx + dy * dy < min_dist) {
                good = false;
                goto break_out;
              }
            }
          }
        }
      }
    break_out:

      if (good) {
        grid[y_cell * grid_width + x_cell].emplace_back(Point2f(x, y));
        cornersQuality.emplace_back(eig[y][x]);
        corners.emplace_back(Point2f(x, y));
        ++ncorners;
        if (max_corners_ > 0 && static_cast<int>(ncorners) == max_corners_)
          break;
      }
    }
  }
  *out_corners = corners;

  return true;
}
