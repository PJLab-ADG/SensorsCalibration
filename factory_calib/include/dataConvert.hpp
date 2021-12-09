/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#ifndef UTILS_DATACONVERT_HPP_
#define UTILS_DATACONVERT_HPP_

#include "utils/common.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>
#include <vector>

static void stdvec2cvmat(const std::vector<std::vector<bool>> &img) {
  std::vector<float> img_uchar;
  for (size_t i = 0; i < img.size(); i++) {
    for (size_t j = 0; j < img[0].size(); j++) {
      img_uchar.push_back(static_cast<float>(img[i][j]));
    }
  }

  cv::Mat mat = cv::Mat(img_uchar);
  cv::Mat dest = mat.reshape(1, img.size()).clone();
  cv::imshow("dest", dest);
  cv::waitKey();
}

static void stdvec2cvmat(const std::vector<std::vector<float>> &img,
                         cv::Mat &dest) {
  std::vector<float> img_uchar;
  for (int i = 0; i < img.size(); i++) {
    for (int j = 0; j < img[0].size(); j++) {
      img_uchar.push_back(static_cast<float>(img[i][j]));
    }
  }

  cv::Mat mat = cv::Mat(img_uchar);
  dest = mat.reshape(1, img.size()).clone();
}

static void stdvec2cvmat(const std::vector<std::vector<int>> &img,
                         cv::Mat &dest) {
  std::vector<float> img_uchar;
  for (int i = 0; i < img.size(); i++) {
    for (int j = 0; j < img[0].size(); j++) {
      img_uchar.push_back(static_cast<float>(img[i][j]));
    }
  }

  cv::Mat mat = cv::Mat(img_uchar);
  dest = mat.reshape(1, img.size()).clone();
}

static void cvmat2stdvec(const cv::Mat &src,
                         std::vector<std::vector<float>> &img) {
  img.resize(src.rows, std::vector<float>(src.cols));
  std::cout << "img.size(): " << img.size() << std::endl;
  std::cout << "img[0].size(): " << img[0].size() << std::endl;
  for (int i = 0; i < img.size(); i++) {
    for (int j = 0; j < img[0].size(); j++) {
      img[i][j] = src.at<float>(i, j);
      // img[i][j] = float(src.at<unsigned char>(i,j));
    }
  }
  std::cout << std::endl;
}

#endif /*UTILS_DATACONVERT_HPP_*/