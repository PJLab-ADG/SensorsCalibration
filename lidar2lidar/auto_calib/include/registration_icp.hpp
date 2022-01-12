#pragma once

#include "logging.hpp"
#include "transform_util.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
class ICPRegistrator {
public:
  ICPRegistrator(){};
  void SetTargetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &gcloud,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &ngcloud);
  void SetSourceCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &gcloud,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &ngcloud);

  void RegistrationByICP(const Eigen::Matrix4d &init_guess,
                         double *refined_yaw);
  Eigen::Matrix4d GetFinalTransformation();
  double CalculateICPError(const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,
                           const Eigen::Matrix4d &init_guess, float cur_yaw);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_gcloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_ngcloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_gcloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_ngcloud_;
  Eigen::Matrix4d final_transformation_;
};