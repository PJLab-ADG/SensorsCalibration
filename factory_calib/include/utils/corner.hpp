/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#ifndef APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_CORNER_HPP_
#define APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_CORNER_HPP_

#include "Eigen/Core"
#include "utils/common.hpp"
#include "utils/filter.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <time.h>
#include <vector>

// from opencv to calc corners
namespace imgproc {

static inline void calcMinEigenVal(const std::vector<std::vector<float>> &_cov,
                                   std::vector<std::vector<float>> *_dst,
                                   int cn) {
  int height = _dst->size();
  int width = (*_dst)[0].size();

  for (int i = 0; i < height; ++i) {
    const auto cov = _cov[i].begin();
    auto dst = (*_dst)[i].begin();
    for (int j = 0; j < width; ++j) {
      float a = cov[j * 3] * 0.5f;
      float b = cov[j * 3 + 1];
      float c = cov[j * 3 + 2] * 0.5f;
      dst[j] = ((a + c) - std::sqrt((a - c) * (a - c) + b * b));
    }
  }
}

static inline void
cornerEigenValsVecs(const std::vector<std::vector<float>> &src,
                    std::vector<std::vector<float>> *eig, int block_size = 3,
                    int aperture_size = 3, double k = 0) {
  int height = src.size();
  int width = src[0].size();
  // three channel
  int channel = 3;
  std::vector<std::vector<float>> cov(height,
                                      std::vector<float>(width * channel));

  // img:0-255
  double scale =
      static_cast<double>(1 << ((aperture_size > 0 ? aperture_size : 3) - 1)) *
      block_size;
  scale = 1.0 / scale;

  std::vector<std::vector<float>> Dx, Dy;
  // calc cov data with sobel
  DoubleSobel(src, &Dx, &Dy, &cov, aperture_size, scale);
  boxFilter(&cov, channel, block_size, block_size, Point2i(-1, -1), false);
  calcMinEigenVal(cov, eig, channel);
}

static inline void cornerMinEigenVal(const std::vector<std::vector<float>> &img,
                                     std::vector<std::vector<float>> *eig,
                                     int block_size = 3, int ksize = 3) {
  if (img.size() == 0 || img[0].size() == 0) {
    // LOGE("wrong input img size.\n");
    return;
  }
  eig->resize(img.size(), std::vector<float>(img[0].size()));
  cornerEigenValsVecs(img, eig, block_size, ksize);
}

static inline void minMaxLoc(const std::vector<std::vector<float>> &src,
                             double *maxVal) {
  int height = src.size();
  // int width = src[0].size();
  *maxVal = 0;

  // TODO(liuzhuochun): optimize speed
  for (int i = 0; i < height; ++i) {
    double max_elem = *std::max_element(src[i].begin(), src[i].end());
    if (max_elem > *maxVal)
      *maxVal = max_elem;
  }
}

static inline void threshold(std::vector<std::vector<float>> *dst,
                             double thresh, double other) {
  int height = dst->size();
  int width = (*dst)[0].size();
  int i, j;

  // TODO(liuzhuochun): optimize speed
  for (i = 0; i < height; ++i) {
    auto row_data = (*dst)[i].begin();
    for (j = 0; j < width; ++j) {
      if (row_data[j] < thresh)
        row_data[j] = other;
    }
  }
}

static inline void binaryThreshold(const std::vector<std::vector<float>> &src,
                                   std::vector<std::vector<int>> *dst,
                                   int thresh) {
  int height = src.size();
  int width = src[0].size();
  int i, j;
  dst->resize(height, std::vector<int>(width));
  // TODO(liuzhuochun): optimize speed
  for (i = 0; i < height; ++i) {
    auto src_row_data = src[i].begin();
    auto dst_row_data = (*dst)[i].begin();
    for (j = 0; j < width; ++j) {
      if (src_row_data[j] > thresh)
        dst_row_data[j] = 255;
    }
  }
}

static inline void cornerDilate(std::vector<std::vector<float>> *dst) {
  int height = dst->size();
  int width = (*dst)[0].size();
  int i, j, kx, ky;

  // TODO(liuzhuochun): optimize speed
  for (i = 1; i < height - 1; ++i) {
    auto row_dst = (*dst)[i].begin();
    for (j = 1; j < width - 1; ++j) {
      float center_data = row_dst[j];
      if (center_data == 0)
        continue;
      // block 3
      for (kx = -1; kx <= 1; ++kx) {
        for (ky = -1; ky <= 1; ++ky) {
          if ((*dst)[i + kx][j + ky] > center_data) {
            goto break_out;
          }
        }
      }
      row_dst[j] = center_data;
      ++j; // skip one col
    break_out:
      row_dst[j] = 0;
    }
  }
}

static inline void
EstimateGrayThres(const std::vector<std::vector<float>> &imgGray,
                  std::vector<int> *thres_box, float edge_fill_percent = 0) {
  if (thres_box == nullptr)
    return;
  if (imgGray.size() == 0)
    return;
  if (edge_fill_percent > 0.29)
    return;
  int height = imgGray.size();
  int width = imgGray[0].size();
  int edge_fill_thickness_x = static_cast<int>(edge_fill_percent * height);
  int edge_fill_thickness_y = static_cast<int>(edge_fill_percent * width);
  float aver_gray = 0;
  float mid_gray = 0;

  std::vector<int> gray_pixel_num(256);
  std::vector<float> gray_prob(256);
  std::vector<float> gray_distribution(256);
  int pixel_num = 0;

  for (int i = edge_fill_thickness_x; i < height - edge_fill_thickness_x; i++) {
    for (int j = edge_fill_thickness_y; j < width - edge_fill_thickness_y;
         j++) {
      aver_gray = (aver_gray * pixel_num + imgGray[i][j]) /
                  static_cast<float>(pixel_num + 1);
      gray_pixel_num[static_cast<int>(imgGray[i][j])]++;
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

  std::vector<int> possible_box;
  int start_thres = static_cast<int>(aver_gray);
  if (aver_gray > 127.5)
    start_thres = static_cast<int>(mid_gray + mid_gray - aver_gray) % 256;
  else
    start_thres = static_cast<int>(aver_gray + aver_gray - mid_gray) % 256;
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
  possible_box.push_back((start_thres - stride + 256) % 256);
  possible_box.push_back(start_thres);
  for (int i = 2; i <= iter; i++) {
    if (size_t(i - 1) >= possible_box.size())
      break;
    if (possible_box[i - 1] + stride >= 255 ||
        possible_box[i - 1] + stride < 0) {
      possible_box.push_back(start_thres - 2 * stride);
      stride *= -1;
    } else {
      possible_box.push_back(possible_box[i - 1] + stride);
    }
  }

  std::cout << "possible box:\n";
  for (size_t i = 0; i < possible_box.size(); i++) {
    std::cout << possible_box[i] << " ";
  }
  std::cout << std::endl;

  *thres_box = possible_box;
}

} // namespace imgproc
#endif // APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_CORNER_HPP_
