/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#ifndef LIDAR_FEATURE_HPP_
#define LIDAR_FEATURE_HPP_

#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <opencv2/core/eigen.hpp>
#include <vector>

class LidarFeature {
private:
  /* data */
  std::vector<Eigen::Matrix4Xf> point_mats;
  std::vector<std::vector<float>> lidar_ranges;
  std::vector<std::vector<float>> lidar_weights;
  std::string pcd_path_;
  Eigen::Matrix3d intrinsic_;
  Eigen::Matrix4d extrinsic_;

  pcl::PointCloud<pcl::PointXYZI> original_pcd_;

  // bbox:feature extraction range
  double x_min_threshold_ = 0;
  double x_max_threshold_ = 100;
  double y_min_threshold_ = -20;
  double y_max_threshold_ = 20;
  double intensity_threshold = 80; // Lane extraction threshold

public:
  LidarFeature(const std::string &cloud_path, const Eigen::Matrix3d &intrinsic,
               const Eigen::Matrix4d &extrinsic);
  ~LidarFeature();

  void extract_feature(const cv::Mat *distance_images,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud);
  void extract_objects(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_pole,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_lane,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_car,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud_lane);

  void plane_model(const pcl::PointCloud<pcl::PointXYZI>::Ptr &full_cloud,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr &ground,
                   float plane[4]);
};

#endif //  LIDAR_FEATURE_HPP_