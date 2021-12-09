/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#include "camera_feature.hpp"

#include "logging.hpp"
#include <opencv2/core/eigen.hpp>

cv::Mat CameraFeature::undistortion(const std::string &image_path,
                                    const Eigen::Matrix3d &intrinsic,
                                    const cv::Mat &distortion) {
  cv::Mat image = cv::imread(image_path, 1);
  cv::Mat threshold;
  threshold = image;
  cv::Mat K;
  cv::eigen2cv(intrinsic, K);
  cv::Mat map1;
  cv::Mat map2;
  cv::initUndistortRectifyMap(K, distortion, cv::Mat::eye(3, 3, CV_32FC1), K,
                              threshold.size(), CV_32FC1, map1, map2);
  cv::remap(threshold, threshold, map1, map2, cv::INTER_LINEAR);
  return threshold;
}
