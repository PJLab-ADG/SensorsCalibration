/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#ifndef APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_FILTER_HPP_
#define APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_FILTER_HPP_

#include "Eigen/Core"
#include "common.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

// used for corner detection
namespace imgproc {

/* NOTE:
 *
 * Sobel-x: -1  0  1
 *          -2  0  2
 *          -1  0  1
 *
 * Sobel-y: -1 -2 -1
 *           0  0  0
 *           1  2  1
 */
static void getSobelKernels(std::vector<float> *sobel_kx,
                            std::vector<float> *sobel_ky, int dx, int dy,
                            int block_size = 3, bool normalize = false) {
  int i, j, ksizeX = block_size, ksizeY = block_size;
  // must larger than 1
  if (ksizeX == 1 && dx > 0)
    ksizeX = 3;
  if (ksizeY == 1 && dx > 0)
    ksizeY = 3;

  sobel_kx->resize(ksizeX);
  sobel_ky->resize(ksizeY);

  if (block_size % 2 == 0 || block_size > 31) {
    // LOGE("The kernel size must be odd and not larger than 31\n");
    return;
  }
  if ((dx != 1 && dy != 1) || (dx == 1 && dy == 1)) {
    // LOGE("wrong param.\n");
    return;
  }

  std::vector<int> kerI(std::max(ksizeX, ksizeY) + 1);

  for (int k = 0; k < 2; ++k) {
    std::vector<float> *kernel = k == 0 ? sobel_kx : sobel_ky;
    int order = k == 0 ? dx : dy;
    int ksize = k == 0 ? ksizeX : ksizeY;
    // ksize > order

    if (ksize == 1) {
      kerI[0] = 1;
    } else if (ksize == 3) {
      if (order == 0)
        kerI[0] = 1, kerI[1] = 2, kerI[2] = 1;
      else if (order == 1)
        kerI[0] = -1, kerI[1] = 0, kerI[2] = 1;
      else
        kerI[0] = 1, kerI[1] = -2, kerI[2] = 1;
    } else {
      int oldval, newval;
      kerI[0] = 1;
      for (i = 0; i < ksize; ++i)
        kerI[i + 1] = 0;

      for (i = 0; i < ksize - order - 1; ++i) {
        oldval = kerI[0];
        for (j = 1; j <= ksize; ++j) {
          newval = kerI[j] + kerI[j - 1];
          kerI[j - 1] = oldval;
          oldval = newval;
        }
      }

      for (i = 0; i < order; ++i) {
        oldval = -kerI[0];
        for (j = 1; j <= ksize; ++j) {
          newval = kerI[j - 1] - kerI[j];
          kerI[j - 1] = oldval;
          oldval = newval;
        }
      }
    }
    *kernel = std::vector<float>(kerI.begin(), kerI.begin() + kernel->size());
    float scale = !normalize ? 1. : 1. / (1 << (ksize - order - 1));
    // add scale
    int kernel_size = kernel->size();
    for (int m = 0; m < kernel_size; ++m)
      (*kernel)[m] *= scale;
  }
}

static void DoubleFilter2D(const std::vector<std::vector<float>> &src,
                           std::vector<std::vector<float>> *dstx,
                           std::vector<std::vector<float>> *dsty,
                           std::vector<std::vector<float>> *cov,
                           const std::vector<float> &kx,
                           const std::vector<float> &ky) {
  // TODO(liuzhuochun): border
  // width?
  int width = src[0].size();
  int height = src.size();
  int i, j, k;
  int ksizeX = kx.size(), ksizeY = ky.size();
  int gapX = (ksizeX - 1) / 2, gapY = (ksizeY - 1) / 2;

  std::vector<std::vector<float>> tmp_dstx(height, std::vector<float>(width));
  std::vector<std::vector<float>> tmp_dsty(height, std::vector<float>(width));
  // row
  for (j = 0; j < height; ++j) {
    auto row_src = src[j].begin();
    auto row_tmp_dstx = tmp_dstx[j].begin();
    auto row_tmp_dsty = tmp_dsty[j].begin();
    // abandon border region
    for (i = 0; i < width - ksizeX + 1; ++i) {
      float row_d = row_src[i];
      float sx = kx[0] * row_d;
      float sy = ky[0] * row_d;
      for (k = 1; k < ksizeX; ++k) {
        float row_dk = row_src[i + k];
        sx += kx[k] * row_dk;
        sy += ky[k] * row_dk;
      }
      row_tmp_dstx[i + gapX] = sx;
      row_tmp_dsty[i + gapX] = sy;
    }
  }

  for (j = 0; j < height - ksizeY + 1; ++j) {
    auto row_tmp_dstx = tmp_dstx[j].begin();
    auto row_tmp_dsty = tmp_dsty[j].begin();
    auto row_dstx = (*dstx)[j + gapY].begin();
    auto row_dsty = (*dsty)[j + gapY].begin();
    auto cov_data = (*cov)[j + gapY].begin();
    for (i = gapX; i < width - gapX; ++i) {
      float sx = ky[0] * row_tmp_dstx[i];
      float sy = kx[0] * row_tmp_dsty[i];
      for (k = 1; k < ksizeY; ++k) {
        sx += ky[k] * tmp_dstx[j + k][i];
        sy += kx[k] * tmp_dsty[j + k][i];
      }
      row_dstx[i] = sx;
      row_dsty[i] = sy;
      cov_data[i * 3] = sx * sx;
      cov_data[i * 3 + 1] = sx * sy;
      cov_data[i * 3 + 2] = sy * sy;
    }
  }
}

// calc cov matrix at the same time
static void DoubleSobel(const std::vector<std::vector<float>> &img,
                        std::vector<std::vector<float>> *Dx,
                        std::vector<std::vector<float>> *Dy,
                        std::vector<std::vector<float>> *cov, int ksize,
                        double scale) {
  if (img.size() == 0 || img[0].size() == 0) {
    // LOGE("wrong input img size.\n");
    return;
  }
  Dx->resize(img.size(), std::vector<float>(img[0].size()));
  Dy->resize(img.size(), std::vector<float>(img[0].size()));
  std::vector<float> kx, ky;
  getSobelKernels(&kx, &ky, 1, 0, ksize, false);
  for (float &data : ky)
    data *= scale;
  DoubleFilter2D(img, Dx, Dy, cov, kx, ky);
}

/* NOTE:
 * 1  1  1
 * 1  1  1
 * 1  1  1
 */
static void boxFilter(std::vector<std::vector<float>> *src,
                      int cn, // channel
                      int ksizeX, int ksizeY, Point2i anchor, bool normalize) {
  int width = (*src)[0].size();
  // int img_width = width / cn;
  int height = src->size();
  int gapX = (ksizeX - 1) / 2, gapY = (ksizeY - 1) / 2;
  std::vector<float> value(cn);
  int i, j, k;
  int ksizeX_cn = cn * ksizeX;
  int gapX_cn = cn * gapX;

  float scale = !normalize ? 1. : 1. / static_cast<float>(ksizeX * ksizeY);

  std::vector<std::vector<float>> tmp_dst(height, std::vector<float>(width));

  // row
  for (j = 0; j < height; ++j) {
    auto row_src = (*src)[j].begin();
    auto row_tmp_dst = tmp_dst[j].begin();
    // abandon border region
    for (k = 0, value = std::vector<float>({0, 0, 0}); k < ksizeX_cn; k += cn) {
      value[0] += row_src[k];
      value[1] += row_src[k + 1];
      value[2] += row_src[k + 2];
    }
    row_tmp_dst[gapX_cn] = value[0];
    row_tmp_dst[gapX_cn + 1] = value[1];
    row_tmp_dst[gapX_cn + 2] = value[2];
    for (i = cn; i < width - ksizeX_cn + cn; i += cn) {
      value[0] += row_src[i + ksizeX_cn - cn] - row_src[i - cn];
      value[1] += row_src[i + ksizeX_cn - cn + 1] - row_src[i - cn + 1];
      value[2] += row_src[i + ksizeX_cn - cn + 2] - row_src[i - cn + 2];
      row_tmp_dst[i + gapX_cn] = value[0];
      row_tmp_dst[i + gapX_cn + 1] = value[1];
      row_tmp_dst[i + gapX_cn + 2] = value[2];
    }
  }

  std::vector<float> col_sum(width);
  float colvalue = 0;
  // col
  for (k = 0; k < ksizeY - 1; ++k) {
    auto row_line = tmp_dst[k].begin();
    for (i = gapX_cn; i < width - gapX_cn; ++i)
      col_sum[i] += row_line[i];
  }

  for (j = 0; j < height - ksizeY + 1; ++j) {
    auto dst_line = (*src)[j + gapY].begin();
    auto addpos = tmp_dst[j + ksizeY - 1].begin();
    auto subpos = tmp_dst[j].begin();
    for (i = 0; i < width; ++i) {
      colvalue = col_sum[i] + addpos[i];
      dst_line[i] = colvalue * scale;
      col_sum[i] = colvalue - subpos[i];
    }
  }
}

} // namespace imgproc

#endif // APPS_TOOLS_SENSOR_CALIBRATION_INCLUDE_UTILS_FILTER_HPP_
