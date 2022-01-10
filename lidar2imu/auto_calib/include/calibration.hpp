/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/Lidar_parser_base.h"

class Calibrator {
public:
  Calibrator();
  ~Calibrator();

  // load data
  void LoadTimeAndPoes(const std::string &filename, const Eigen::Matrix4d &Tl2i,
                       std::vector<std::string> &lidarTimes,
                       std::vector<Eigen::Matrix4d> &lidarPoses);

  Eigen::Matrix4d GetDeltaTrans(double R[3], double t[3]);

  void Calibration(const std::string lidar_path, const std::string odom_path,
                   const Eigen::Matrix4d init_Tl2i);
  void SaveStitching(const Eigen::Matrix4d transform,
                     const std::string pcd_name);

private:
  int turn_ = 35;
  int window_ = 10;
  std::vector<std::string> lidar_files_;
  std::vector<Eigen::Matrix4d> lidar_poses_;
  // std::vector<pcl::PointCloud<LidarPointXYZIRT>> pcd_seq_;
  double degree_2_radian = 0.017453293;
  std::string lidar_path_;
};