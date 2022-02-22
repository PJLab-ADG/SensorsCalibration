/*
 * Copyright (C) 2022 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include <fstream>
#include <iostream>
#include <sstream>

#include "camera_calibrator.hpp"

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cout << "Usage: ./main camera_dir csv file"
                 "\nexample:\n\t"
                 "./bin/lidar2camera data/intrinsic/ data/circle.csv"
              << std::endl;
    return 0;
  }

  std::string image_dir = argv[1];
  std::string csv_file = argv[2];
  std::cout << csv_file << std::endl;
  std::ifstream fin(csv_file);
  std::string line;
  bool is_first = true;
  std::vector<std::vector<std::string>> lidar_3d_pts;
  while (getline(fin, line)) {
    if (is_first) {
      is_first = false;
      continue;
    }

    std::istringstream sin(line);
    std::vector<std::string> fields;
    std::string field;
    while (getline(sin, field, ',')) {
      fields.push_back(field);
    }
    lidar_3d_pts.push_back(fields);
  }

  std::cout << image_dir << std::endl;
  std::vector<cv::String> images;
  cv::glob(image_dir, images);
  std::vector<cv::Mat> vec_mat;
  std::vector<std::string> images_name;
  for (const auto &path : images) {
    std::cout << path << std::endl;
    cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
    vec_mat.push_back(img);
    images_name.push_back(path);
  }

  CameraCalibrator m;
  cv::Mat camera_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
  cv::Mat k = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
  std::vector<cv::Mat> tvecsMat;
  std::vector<cv::Mat> rvecsMat;
  m.set_input(images_name, vec_mat, cv::Size{17, 7}, lidar_3d_pts);
  m.get_result(camera_matrix, k, cv::Size{1920, 1200}, rvecsMat, tvecsMat);
  return 0;
}