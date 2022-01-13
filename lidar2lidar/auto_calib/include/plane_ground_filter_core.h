/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#pragma once

#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

class PlaneGroundFilter {

private:
  int sensor_model_;
  double sensor_height_, clip_height_, min_distance_, max_distance_;
  int num_seg_ = 1;
  int num_iter_, num_lpr_;
  double th_seeds_, th_dist_;
  // Model parameter for ground plane fitting
  // The ground plane model is: ax+by+cz+d=0
  // Here normal:=[a,b,c], d=d
  // th_dist_d_ = threshold_dist - d
  float d_, th_dist_d_;
  Eigen::MatrixXf normal_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr g_seeds_pc;
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_ground_pc;
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_not_ground_pc;
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_all_pc;

  void estimate_plane();
  void extract_initial_seeds(const pcl::PointCloud<pcl::PointXYZ> &p_sorted);
  void post_process(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr out);
  void point_cb(const pcl::PointCloud<pcl::PointXYZ> &in_cloud);
  void clip_above(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr out);
  void remove_close_far_pt(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                           const pcl::PointCloud<pcl::PointXYZ>::Ptr out);

public:
  PlaneGroundFilter();
  ~PlaneGroundFilter(){};
};