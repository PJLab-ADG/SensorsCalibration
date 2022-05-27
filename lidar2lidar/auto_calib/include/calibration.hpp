/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#pragma once

#include <Eigen/Dense>
#include <array>
#include <map>
#include <memory>
#include <pcl/io/pcd_io.h>
#include <string>
#include <vector>

#include "logging.hpp"
#include "registration_icp.hpp"

struct InitialExtrinsic {
  Eigen::Vector3d euler_angles;
  Eigen::Vector3d t_matrix;
};
struct PlaneParam {
  PlaneParam() {}
  PlaneParam(const Eigen::Vector3d &n, double i) : normal(n), intercept(i) {}
  Eigen::Vector3d normal;
  double intercept;
};

class Calibrator {
public:
  Calibrator();
  void LoadCalibrationData(
      const std::map<int32_t, pcl::PointCloud<pcl::PointXYZI>> lidar_points,
      const std::map<int32_t, InitialExtrinsic> extrinsics);
  Eigen::Matrix3d GetRotation(double roll, double pitch, double yaw);
  Eigen::Matrix4d GetMatrix(const Eigen::Vector3d &translation,
                            const Eigen::Matrix3d &rotation);
  void Calibrate();
  bool
  GroundPlaneExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud,
                        PlaneParam &plane);
  std::map<int32_t, Eigen::Matrix4d> GetFinalTransformation();

private:
  std::map<int32_t, pcl::PointCloud<pcl::PointXYZI>> pcs_;
  std::map<int32_t, Eigen::Matrix4d> init_extrinsics_;
  std::map<int32_t, Eigen::Matrix4d> refined_extrinsics_;

  std::unique_ptr<ICPRegistrator> registrator_;
};
