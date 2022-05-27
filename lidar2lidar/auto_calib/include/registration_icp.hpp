#pragma once

#include "logging.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

class ICPRegistrator {
public:
  ICPRegistrator();
  void SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &gcloud,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &ngcloud,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
  void SetSourceCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &gcloud,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &ngcloud,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

  bool RegistrationByICP(const Eigen::Matrix4d &init_guess,
                         Eigen::Matrix4d &transform);
  bool RegistrationByICP2(const Eigen::Matrix4d &init_guess,
                          Eigen::Matrix4d &refined_extrinsic);
  Eigen::Matrix4d GetFinalTransformation();
  double CalculateICPError(const pcl::KdTreeFLANN<pcl::PointXYZI> &kdtree,
                           const Eigen::Matrix4d &init_guess, float cur_yaw);

  void computeNormals(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_pts,
                      pcl::PointCloud<pcl::PointXYZINormal>::Ptr out_pts);
  bool RegistrationByVoxelOccupancy(const Eigen::Matrix4d &init_guess,
                                    Eigen::Matrix4d &refined_extrinsic);
  size_t ComputeVoxelOccupancy(const Eigen::Matrix4d &init_guess);

private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr tgt_gcloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr tgt_ngcloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr tgt_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr src_gcloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr src_ngcloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr all_octree_;

  Eigen::Matrix4d final_transformation_;
};