/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#pragma once

#include <fstream>
#include <iostream>
#include <json/json.h>
#include <stdio.h>
#include <string>

void LoadExtrinsic(const std::string &filename, Eigen::Matrix4d &extrinsic) {
  Json::Reader reader;
  Json::Value root;

  std::ifstream in(filename, std::ios::binary);
  // std::ifstream in;
  // in.open(filename);
  if (!in.is_open()) {
    std::cout << "Error Opening " << filename << std::endl;
    return;
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