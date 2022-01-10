/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "calibration.hpp"
#include "extrinsic_param.hpp"
#include "registration.hpp"
#include <iostream>

#include "logging.hpp"
#include "transform_util.hpp"
using namespace std;

int main(int argc, char **argv) {
  if (argc != 4) {
    cout << "Usage: ./run_lidar2imu <lidar_pcds_dir> <poses_path> "
            "<extrinsic_json> "
            "\nexample:\n\t"
            "./bin/run_lidar2imu data/top_center_lidar/ "
            "data/NovAtel-pose-lidar-time.txt "
            "data/gnss-to-top_center_lidar-extrinsic.json "
         << endl;
    return 0;
  }
  string lidar_pcds_dir = argv[1];
  string poses_path = argv[2];
  string extrinsic_json = argv[3];
  string stitching_path = "stitching.pcd";
  // load extrinsic
  Eigen::Matrix4d json_param;
  LoadExtrinsic(extrinsic_json, json_param);
  LOGI("Load extrinsic!");
  // convert to lidar 2 imu
  Eigen::Matrix4d lidar2imu_extrinsic = json_param.inverse().eval();
  std::cout << json_param << std::endl;
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  // Registrator registrator;
  // registrator.LoadOdometerData(poses_path, lidar2imu);
  // registrator.LoadLidarPCDs(lidar_pcds_dir);
  // registrator.RegistrationByGroundPlane(transform);
  // registrator.RegistrationByVoxelOccupancy(transform);
  // registrator.SaveStitching(stitching_path);
  // std::cout << "the calibration result is " << std::endl;
  // std::cout << transform << std::endl;
  Calibrator calibrator;
  calibrator.Calibration(lidar_pcds_dir, poses_path, lidar2imu_extrinsic);

  return 0;
}