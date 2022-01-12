/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#include <Eigen/Core>
#include <chrono>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "extrinsic_param.hpp"
#include "registration.hpp"
#include <iostream>

#include "logging.hpp"
#include "transform_util.hpp"
using namespace std;

int main(int argc, char **argv) {
  if (argc != 4) {
    cout << "Usage: ./run_lidar2lidar <target_pcd_path> <source_pcd_path> "
            "<extrinsic_json> "
            "\nexample:\n\t"
            "./bin/run_lidar2lidar data/top_center_lidar.pcd "
            "data/front_lidar.pcd "
            "data/front_lidar-to-top_center_lidar-extrinsic.json  "
         << endl;
    return 0;
  }
  string target_lidar_path = argv[1];
  string source_lidar_path = argv[2];
  string extrinsic_json = argv[3];
  // load target lidar points
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_lidar_path, *target_cloud) ==
      -1) {
    LOGE("Couldn't read target lidar file \n");
    return (-1);
  }
  // load source lidar points
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(source_lidar_path, *source_cloud) ==
      -1) {
    LOGE("Couldn't read source lidar file \n");
    return (-1);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_copy(
      new pcl::PointCloud<pcl::PointXYZ>);
  *source_cloud_copy = *source_cloud;
  // load extrinsic
  Eigen::Matrix4f json_param;
  LoadExtrinsic(extrinsic_json, json_param);
  pcl::transformPointCloud(*source_cloud, *source_cloud, json_param);
  LOGI("Loading data completed!");
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f curr_transform = Eigen::Matrix4f::Identity();
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_filter_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_filter_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  auto time_begin = std::chrono::steady_clock::now();
  // registration
  Registrator registrator;
  registrator.PointCloudDownSampling(source_cloud, source_filter_cloud);
  registrator.PointCloudDownSampling(target_cloud, target_filter_cloud);
  registrator.RegistrationByGroundPlane(source_filter_cloud,
                                        target_filter_cloud, transform);
  curr_transform = transform * curr_transform;
  NDTParameter ndt_param;
  ndt_param.ndt_step_size = 0.2;
  ndt_param.resolution = 10;
  ndt_param.ndt_transformation_epsilon = 0.1;
  bool is_suceed = registrator.RegistrationByNDT(
      source_filter_cloud, target_filter_cloud, ndt_param, transform);
  if (!is_suceed) {
    LOGE("registration failed");
    return (-1);
  }
  curr_transform = transform * curr_transform;
  ndt_param.ndt_step_size = 0.1;
  ndt_param.resolution = 1;
  ndt_param.ndt_transformation_epsilon = 0.02;
  is_suceed = registrator.RegistrationByNDT(
      source_filter_cloud, target_filter_cloud, ndt_param, transform);
  if (!is_suceed) {
    LOGE("registration failed");
    return (-1);
  }
  curr_transform = transform * curr_transform;
  registrator.RegistrationByVoxelOccupancy(source_filter_cloud,
                                           target_filter_cloud, transform);
  curr_transform = transform * curr_transform;

  // save result
  curr_transform = curr_transform * json_param;
  pcl::transformPointCloud(*source_cloud_copy, *source_cloud_copy,
                           curr_transform);
  pcl::io::savePCDFile<pcl::PointXYZ>("source_cloud_trans.pcd",
                                      *source_cloud_copy);
  LOGI("calibration complete!");
  std::cout << "the calibration result is " << std::endl;
  std::cout << curr_transform << std::endl;
  auto time_end = std::chrono::steady_clock::now();
  std::cout << "calib cost "
            << std::chrono::duration<double>(time_end - time_begin).count()
            << "s" << std::endl;
  return 0;
}