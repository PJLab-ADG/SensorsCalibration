/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>

struct NDTParameter {
  double ndt_maxIterations = 50;
  double ndt_transformation_epsilon = 0.05;
  double ndt_step_size = 0.1; // need to adjust
  double resolution = 10;     // nedd to adjust
};

struct PlaneParam {
  PlaneParam() {}
  PlaneParam(const Eigen::Vector3d &n, double i) : normal(n), intercept(i) {}
  Eigen::Vector3d normal;
  double intercept;
};

struct PlaneParamError {
  PlaneParamError() {}
  PlaneParamError(double n, double i) : normal_error(n), intercept_error(i) {}
  double normal_error;
  double intercept_error;
};

struct PointCloudBbox {
  int min_x = 0;
  int min_y = 0;
  int min_z = 0;

  int max_x = 0;
  int max_y = 0;
  int max_z = 0;
};

class Registrator {
public:
  Registrator();
  ~Registrator();

  bool
  GroundPlaneExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud,
                        PlaneParam &plane);
  bool
  GroundPlaneExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
                        PlaneParam &g_param);

  void
  PointCloudDownSampling(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
                         double voxel_size,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud);

  void
  PointCloudFilterByROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
                        const PointCloudBbox &roi,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud);

  size_t ComputeVoxelOccupancy(float var[6]);

  double ComputeGroundRegistrationError(float var[6]);

  // registration method
  bool RegistrationByGroundPlane(Eigen::Matrix4d &transform);
  bool RegistrationByVoxelOccupancy(Eigen::Matrix4d &transform);

  // load data
  void LoadOdometerData(const std::string odometer_file,
                        const Eigen::Matrix4d &initial_extrinsic);
  void LoadLidarPCDs(const std::string &pcds_dir);

  void SaveStitching(const std::string &stitching_path);

private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr all_octree_;

  std::vector<pcl::PointCloud<pcl::PointXYZI>> pcds_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> pcds_g_cloud_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> pcds_ng_cloud_;
  std::vector<std::string> timestamp_;
  std::vector<Eigen::Matrix4d> lidar_poses_;

  float curr_best_extrinsic_[6] = {0};
  Eigen::Matrix4d lidar2imu_initial_ = Eigen::Matrix4d::Identity();

  int intensity_threshold_ = 35;
};