/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#ifndef CAMERA_FEATURE_HPP_
#define CAMERA_FEATURE_HPP_

#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

class CameraFeature {

public:
  CameraFeature() = default;
  ~CameraFeature() = default;

  static cv::Mat undistortion(const std::string &image_path,
                              const Eigen::Matrix3d &intrinsic,
                              const cv::Mat &distortion);
};

#endif //  CAMERA_FEATURE_HPP_