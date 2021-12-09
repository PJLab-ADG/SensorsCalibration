/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#ifndef COMMON_IMUDATA_READER_HPP_
#define COMMON_IMUDATA_READER_HPP_

#include "logging.hpp"
#include "time.h"
#include <Eigen/Core>
#include <cmath>
#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdio.h>
#include <string>
#include <vector>

void LoadExtrinsic(const std::string &filename, Eigen::Matrix4d &extrinsic) {
  Json::Reader reader;
  Json::Value root;

  std::ifstream in(filename, std::ios::binary);
  if (!in.is_open()) {
    std::cout << "Error Opening " << filename << std::endl;
    exit(1);
  }

  if (reader.parse(in, root, false)) {
    auto name = root.getMemberNames();
    std::string id = *(name.begin());
    std::cout << id << std::endl;
    Json::Value data = root[id]["param"]["sensor_calib"]["data"];
    extrinsic << data[0][0].asDouble(), data[0][1].asDouble(),
        data[0][2].asDouble(), data[0][3].asDouble(), data[1][0].asDouble(),
        data[1][1].asDouble(), data[1][2].asDouble(), data[1][3].asDouble(),
        data[2][0].asDouble(), data[2][1].asDouble(), data[2][2].asDouble(),
        data[2][3].asDouble(), data[3][0].asDouble(), data[3][1].asDouble(),
        data[3][2].asDouble(), data[3][3].asDouble();
  }
  in.close();
  return;
}

void LoadIntrinsic(const std::string &filename, Eigen::Matrix3d &intrinsic,
                   cv::Mat &distortion) {
  Json::Reader reader;
  Json::Value root;

  std::ifstream in(filename, std::ios::binary);
  if (!in.is_open()) {
    std::cout << "Error Opening " << filename << std::endl;
    exit(1);
  }

  if (reader.parse(in, root, false)) {
    auto name = root.getMemberNames();
    std::string id = *(name.begin());
    std::cout << id << std::endl;
    Json::Value data = root[id]["param"]["cam_K"]["data"];
    intrinsic << data[0][0].asDouble(), data[0][1].asDouble(),
        data[0][2].asDouble(), data[1][0].asDouble(), data[1][1].asDouble(),
        data[1][2].asDouble(), data[2][0].asDouble(), data[2][1].asDouble(),
        data[2][2].asDouble();

    Json::Value data_distort = root[id]["param"]["cam_dist"]["data"];
    int size = root[id]["param"]["cam_dist"]["cols"].asInt();
    std::vector<float> distortions;
    for (int i = 0; i < size; i++) {
      distortions.push_back(data_distort[0][i].asFloat());
    }
    distortion = cv::Mat(distortions, true);
  }
  // std::cout<<distortion<<std::endl;
  in.close();
  return;
}

#endif //  COMMON_IMUDATA_READER_HPP_
