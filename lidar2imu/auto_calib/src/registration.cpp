/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include "registration.hpp"

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "logging.hpp"
#include "omp.h"
#include "transform_util.hpp"

Registrator::Registrator() {
  all_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  all_octree_.reset(
      new pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>(0.1));
  all_octree_->setInputCloud(all_cloud_);
};

Registrator::~Registrator() {
  pcds_.clear();
  pcds_.shrink_to_fit();
  pcds_g_cloud_.clear();
  pcds_g_cloud_.shrink_to_fit();
  pcds_ng_cloud_.clear();
  pcds_ng_cloud_.shrink_to_fit();
};

void Registrator::LoadOdometerData(const std::string odometer_file,
                                   const Eigen::Matrix4d &initial_extrinsic) {
  lidar2imu_initial_ = initial_extrinsic;
  std::ifstream file(odometer_file);
  if (!file.is_open()) {
    LOGW("can not open %s", odometer_file);
    return;
  }
  // load pose and timestamp
  std::string line;
  while (getline(file, line)) {
    std::stringstream ss(line);
    std::string timeStr;
    ss >> timeStr;
    timestamp_.emplace_back(timeStr);
    Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
    ss >> Ti(0, 0) >> Ti(0, 1) >> Ti(0, 2) >> Ti(0, 3) >> Ti(1, 0) >>
        Ti(1, 1) >> Ti(1, 2) >> Ti(1, 3) >> Ti(2, 0) >> Ti(2, 1) >> Ti(2, 2) >>
        Ti(2, 3);
    Ti *= initial_extrinsic;
    lidar_poses_.emplace_back(Ti);
  }
  file.close();
}

void Registrator::LoadLidarPCDs(const std::string &pcds_dir) {
  pcds_g_cloud_.reserve(timestamp_.size());
  pcds_g_cloud_.resize(timestamp_.size());
  pcds_ng_cloud_.reserve(timestamp_.size());
  pcds_ng_cloud_.resize(timestamp_.size());
  pcds_.resize(timestamp_.size());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr g_filter_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ng_filter_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ng_filter_cloud_roi(
      new pcl::PointCloud<pcl::PointXYZI>);
  PointCloudBbox roi;
  roi.max_x = 100;
  roi.min_x = -100;
  roi.max_y = 50;
  roi.min_y = -50;
  roi.max_z = 5;
  roi.min_z = -5;
  PlaneParam plane;
  for (size_t i = 0; i < timestamp_.size(); ++i) {
    std::string lidar_file_name = pcds_dir + "/" + timestamp_[i] + ".pcd";
    if (pcl::io::loadPCDFile(lidar_file_name, *cloud) < 0) {
      LOGW("can not open %s", lidar_file_name);
      return;
    }
    pcds_[i] = *cloud;
    GroundPlaneExtraction(cloud, g_cloud, ng_cloud, plane);

    PointCloudDownSampling(g_cloud, 1, g_filter_cloud);
    pcds_g_cloud_[i] = *g_filter_cloud;
    PointCloudDownSampling(ng_cloud, 0.5, ng_filter_cloud);
    // PointCloudFilterByROI(ng_filter_cloud, roi, ng_filter_cloud_roi);
    pcds_ng_cloud_[i] = *ng_filter_cloud_roi;
    printf("\rload: %lu/%lu, %s", i, timestamp_.size() - 1,
           lidar_file_name.c_str());
  }
  LOGI("load done!");
  LOGI("the number of pcd is %d", pcds_g_cloud_.size());
}

void Registrator::SaveStitching(const std::string &stitching_path) {
  int interval = 10;
  Eigen::Matrix4d delta_pose = TransformUtil::GetDeltaT(curr_best_extrinsic_);
  for (size_t i = 0; i < pcds_.size(); i++) {
    if (i % interval != 0)
      continue;

    Eigen::Matrix4d lidar_pose = lidar_poses_[i];
    lidar_pose *= delta_pose;

    for (const auto &src_pt : pcds_[i].points) {
      if (!pcl_isfinite(src_pt.x) || !pcl_isfinite(src_pt.y) ||
          !pcl_isfinite(src_pt.z))
        continue;
      Eigen::Vector3d p(src_pt.x, src_pt.y, src_pt.z);
      Eigen::Vector3d p_res;
      p_res = lidar_pose.block<3, 3>(0, 0) * p + lidar_pose.block<3, 1>(0, 3);
      pcl::PointXYZI dst_pt;
      dst_pt.x = p_res(0);
      dst_pt.y = p_res(1);
      dst_pt.z = p_res(2);
      dst_pt.intensity = src_pt.intensity;
      if (dst_pt.intensity >= intensity_threshold_ &&
          !all_octree_->isVoxelOccupiedAtPoint(dst_pt)) {
        all_octree_->addPointToCloud(dst_pt, all_cloud_);
      }
    }
  }
  all_cloud_->width = all_cloud_->points.size();
  all_cloud_->height = 1;
  pcl::io::savePCDFileASCII(stitching_path, *all_cloud_);

  all_cloud_->clear();
  all_octree_->deleteTree();
}

bool Registrator::RegistrationByVoxelOccupancy(Eigen::Matrix4d &transform) {
  //
  const float rpy_resolution = 0.05;
  const float xyz_resolution = 0.01;
  float var[6] = {0}, bestVal[6] = {0};
  std::string varName[6] = {"roll", "pitch", "yaw", "tx", "ty", "tz"};
  int direction[2] = {1, -1};
  for (size_t i = 0; i < 6; i++) {
    var[i] = curr_best_extrinsic_[i];
    bestVal[i] = curr_best_extrinsic_[i];
  }

  size_t min_voxel_occupancy = ComputeVoxelOccupancy(var);
  int max_iteration = 60;
  // yaw
  for (int j = 0; j <= 1; j++) {
    for (int iter = 1; iter < max_iteration; iter++) {
      var[2] = iter * direction[j] * rpy_resolution;
      std::cout << varName[2] << ": " << var[2] << std::endl;
      size_t cnt = ComputeVoxelOccupancy(var);
      if (cnt < min_voxel_occupancy * (1 - 1e-4)) {
        min_voxel_occupancy = cnt;
        bestVal[2] = var[2];
        std::cout << "points decrease to: " << min_voxel_occupancy << std::endl;

      } else {
        std::cout << "points increase to: " << cnt << std::endl;
        break;
      }
    }
  }
  var[2] = bestVal[2];
  // tx
  for (int j = 0; j <= 1; j++) {
    for (int iter = 1; iter < max_iteration; iter++) {

      var[3] = iter * direction[j] * xyz_resolution;
      std::cout << varName[3] << ": " << var[3] << std::endl;
      size_t cnt = ComputeVoxelOccupancy(var);
      if (cnt < min_voxel_occupancy * (1 - 1e-4)) {
        min_voxel_occupancy = cnt;
        bestVal[3] = var[3];
        std::cout << "points decrease to: " << min_voxel_occupancy << std::endl;

      } else {
        std::cout << "points increase to: " << cnt << std::endl;
        break;
      }
    }
  }
  var[3] = bestVal[3];
  // ty
  for (int j = 0; j <= 1; j++) {
    for (int iter = 1; iter < max_iteration; iter++) {

      var[4] = iter * direction[j] * xyz_resolution;
      std::cout << varName[4] << ": " << var[4] << std::endl;
      size_t cnt = ComputeVoxelOccupancy(var);
      if (cnt < min_voxel_occupancy * (1 - 1e-4)) {
        min_voxel_occupancy = cnt;
        bestVal[4] = var[4];
        std::cout << "points decrease to: " << min_voxel_occupancy << std::endl;

      } else {
        std::cout << "points increase to: " << cnt << std::endl;
        break;
      }
    }
  }
  var[4] = bestVal[4];
  for (size_t i = 0; i < 6; i++) {
    curr_best_extrinsic_[i] = bestVal[i];
  }
  std::cout << "roll: " << bestVal[0] << ", pitch: " << bestVal[1]
            << ", yaw: " << bestVal[2] << ", tx: " << bestVal[3]
            << ", ty: " << bestVal[4] << ", tz: " << bestVal[5] << std::endl;
  std::cout << "points: " << min_voxel_occupancy << std::endl;

  // calib result
  Eigen::Matrix4d deltaT = TransformUtil::GetDeltaT(bestVal);
  transform = lidar2imu_initial_ * deltaT;
  transform = transform.inverse().eval();

  return true;
}

bool Registrator::RegistrationByGroundPlane(Eigen::Matrix4d &transform) {

  const float rpy_resolution = 0.02;
  float var[6] = {0}, bestVal[6] = {0};
  std::string varName[6] = {"roll", "pitch", "yaw", "tx", "ty", "tz"};
  int direction[2] = {1, -1};
  int max_iteration = 300;
  double min_error = ComputeGroundRegistrationError(var);
  // roll
  for (int j = 0; j <= 1; j++) {
    for (int iter = 1; iter < max_iteration; iter++) {

      var[0] = iter * direction[j] * rpy_resolution;
      std::cout << varName[0] << ": " << var[0] << std::endl;
      double error = ComputeGroundRegistrationError(var);
      if (error < min_error) {
        min_error = error;
        bestVal[0] = var[0];
        std::cout << "error decrease to: " << min_error << std::endl;

      } else {
        std::cout << "error increase to: " << error << std::endl;
        break;
      }
    }
  }
  var[0] = bestVal[0];
  // pitch
  for (int j = 0; j <= 1; j++) {
    for (int iter = 1; iter < max_iteration; iter++) {

      var[1] = iter * direction[j] * rpy_resolution;
      std::cout << varName[1] << ": " << var[1] << std::endl;
      double error = ComputeGroundRegistrationError(var);
      if (error < min_error) {
        min_error = error;
        bestVal[1] = var[1];
        std::cout << "error decrease to: " << min_error << std::endl;

      } else {
        std::cout << "error increase to: " << error << std::endl;
        break;
      }
    }
  }
  var[1] = bestVal[1];
  for (size_t i = 0; i < 6; i++) {
    curr_best_extrinsic_[i] = bestVal[i];
  }
  // calib result
  Eigen::Matrix4d deltaT = TransformUtil::GetDeltaT(bestVal);
  transform = lidar2imu_initial_ * deltaT;
  transform = transform.inverse().eval();
  return true;
}

void Registrator::PointCloudDownSampling(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud, double voxel_size,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud) {
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud(in_cloud);
  sor.setLeafSize(voxel_size, voxel_size, voxel_size);
  sor.filter(*out_cloud);
}

void Registrator::PointCloudFilterByROI(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
    const PointCloudBbox &roi,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud) {
  out_cloud->clear();
  for (const auto &src_pt : in_cloud->points) {
    if (src_pt.x > roi.min_x && src_pt.x < roi.max_x) {
      if (src_pt.y > roi.min_y && src_pt.y < roi.max_y) {
        if (src_pt.z > roi.min_z && src_pt.z < roi.max_z) {
          out_cloud->points.push_back(src_pt);
        }
      }
    }
  }
}

bool Registrator::GroundPlaneExtraction(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud, PlaneParam &plane) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.2);
  seg.setInputCloud(in_cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return (-1);
  }
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(inliers);
  extract.filter(*g_cloud);
  extract.setNegative(true);
  extract.filter(*ng_cloud);
  plane.normal(0) = coefficients->values[0];
  plane.normal(1) = coefficients->values[1];
  plane.normal(2) = coefficients->values[2];
  plane.intercept = coefficients->values[3];
  return true;
}

bool Registrator::GroundPlaneExtraction(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud, PlaneParam &plane) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.2);
  seg.setInputCloud(in_cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    exit(1);
  }

  plane.normal(0) = coefficients->values[0];
  plane.normal(1) = coefficients->values[1];
  plane.normal(2) = coefficients->values[2];
  plane.intercept = coefficients->values[3];
  return true;
}

size_t Registrator::ComputeVoxelOccupancy(float var[6]) {
  int interval = 10;
  Eigen::Matrix4d delta_pose = TransformUtil::GetDeltaT(var);
  for (size_t i = 0; i < pcds_.size(); i++) {
    if (i % interval != 0)
      continue;

    Eigen::Matrix4d lidar_pose = lidar_poses_[i];
    lidar_pose *= delta_pose;

#pragma omp parallel for
    for (const auto &src_pt : pcds_[i].points) {
      if (!pcl_isfinite(src_pt.x) || !pcl_isfinite(src_pt.y) ||
          !pcl_isfinite(src_pt.z))
        continue;

      Eigen::Vector3d p(src_pt.x, src_pt.y, src_pt.z);
      Eigen::Vector3d p_res;
      p_res = lidar_pose.block<3, 3>(0, 0) * p + lidar_pose.block<3, 1>(0, 3);
      pcl::PointXYZI dst_pt;
      dst_pt.x = p_res(0);
      dst_pt.y = p_res(1);
      dst_pt.z = p_res(2);
      dst_pt.intensity = src_pt.intensity;
      if (dst_pt.intensity >= intensity_threshold_ &&
          !all_octree_->isVoxelOccupiedAtPoint(dst_pt)) {

#pragma omp critical
        all_octree_->addPointToCloud(dst_pt, all_cloud_);
      }
    }
    printf("\rprocessing: %lu/%lu", i, pcds_.size() - 1);
    std::fflush(stdout);
  }
  size_t pcdCnt = all_cloud_->size();
  all_cloud_->clear();
  all_octree_->deleteTree();
  return pcdCnt;
}

double Registrator::ComputeGroundRegistrationError(float var[6]) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<Eigen::Vector3d> normals_list;
  std::vector<double> intercepts_list;
  Eigen::Vector3d normals_mean(0, 0, 0);
  double intercepts_mean = 0;
  PlaneParam plane_param;

  Eigen::Matrix4d delta_pose = TransformUtil::GetDeltaT(var);
  for (size_t i = 0; i < pcds_g_cloud_.size(); i++) {
    Eigen::Matrix4d lidar_pose = lidar_poses_[i];
    lidar_pose *= delta_pose;
    pcl::transformPointCloud(pcds_g_cloud_[i], *transformed_cloud, lidar_pose);
    GroundPlaneExtraction(transformed_cloud, plane_param);
    normals_list.push_back(plane_param.normal);
    intercepts_list.push_back(plane_param.intercept);

    normals_mean += plane_param.normal;
    intercepts_mean += plane_param.intercept;
  }

  normals_mean = normals_mean / pcds_g_cloud_.size();
  intercepts_mean = intercepts_mean / pcds_g_cloud_.size();
  double normal_error = 0;
  double intercept_error = 0;
  double min = 0, max = 0;
  for (int i = 0; i < normals_list.size(); i++) {
    double alpha = std::acos(normals_list[i].dot(normals_mean));
    normal_error += std::abs(alpha);
    intercept_error += std::abs(intercepts_list[i] - intercepts_mean);
  }
  return normal_error;
}