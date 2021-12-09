/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#pragma once

#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>

void LoadHomography(const std::string &filename, Eigen::Matrix3d &homo) {
  Json::Reader reader;
  Json::Value root;

  std::ifstream in(filename, std::ios::binary);

  if (!in.is_open()) {
    std::cout << "Error Opening " << filename << std::endl;
    exit(1);
  }
  if (reader.parse(in, root, false)) {
    Json::Value::Members name = root.getMemberNames();
    std::string id = *(name.begin());
    std::cout << id << std::endl;

    Json::Value dst = root[id]["param"]["dst_quad"];
    Json::Value src = root[id]["param"]["src_quad"];
    if (sizeof(dst) != sizeof(src)) {
      std::cout << "[ERROR]Dst points num not equal to src points num."
                << std::endl;
      exit(1);
    }
    int num = sizeof(dst);
    std::vector<cv::Point2f> dst_pts;
    std::vector<cv::Point2f> src_pts;
    for (int i = 0; i < num; i++) {
      cv::Point2f dst_pt(dst[i]["x"].asDouble(), dst[i]["y"].asDouble());
      cv::Point2f src_pt(src[i]["x"].asDouble(), src[i]["y"].asDouble());
      dst_pts.push_back(dst_pt);
      src_pts.push_back(src_pt);
    }
    cv::Mat homography = cv::findHomography(src_pts, dst_pts);
    cv::cv2eigen(homography, homo);
  }
  in.close();

  return;
}
