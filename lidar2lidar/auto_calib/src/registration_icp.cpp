#include "registration_icp.hpp"
#include <limits>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>

ICPRegistrator::ICPRegistrator() {
  all_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  all_octree_.reset(
      new pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>(0.05));
  all_octree_->setInputCloud(all_cloud_);
}
void ICPRegistrator::SetTargetCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &gcloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &ngcloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
  tgt_gcloud_ = gcloud;
  tgt_ngcloud_ = ngcloud;
  tgt_cloud_ = cloud;
}

void ICPRegistrator::SetSourceCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &gcloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &ngcloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
  src_gcloud_ = gcloud;
  src_ngcloud_ = ngcloud;
  src_cloud_ = cloud;
}

Eigen::Matrix4d ICPRegistrator::GetFinalTransformation() {
  return final_transformation_;
}

Eigen::Matrix4d GetDeltaT(const float yaw) {
  Eigen::Matrix3d deltaR = Eigen::Matrix3d(
      Eigen::AngleAxisd(yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));

  Eigen::Matrix4d deltaT = Eigen::Matrix4d::Identity();
  deltaT.block<3, 3>(0, 0) = deltaR;
  return deltaT;
}

bool ICPRegistrator::RegistrationByICP(const Eigen::Matrix4d &init_guess,
                                       Eigen::Matrix4d &transform) {
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(tgt_ngcloud_);
  double cur_yaw = 0;
  double min_error = CalculateICPError(kdtree, init_guess, cur_yaw);
  double best_yaw = cur_yaw;
  float degree_2_radian = 0.017453293;
  int iter_cnt = 0;
  double step = 5; // Resolution is 5°
  int search_range = 10;
  while (iter_cnt < 5) {
    for (int delta = -search_range; delta < search_range; delta++) {
      double yaw = cur_yaw + delta * step * degree_2_radian;
      double error = CalculateICPError(kdtree, init_guess, yaw);
      if (error < min_error) {
        min_error = error;
        best_yaw = yaw;
      }
    }
    search_range = static_cast<int>(search_range / 2 + 0.5);
    step /= 2;
    cur_yaw = best_yaw;
    iter_cnt++;
  }
  Eigen::Matrix4d T = GetDeltaT(best_yaw);
  T = T * init_guess;
  transform = T;
  return true;
}

double ICPRegistrator::CalculateICPError(
    const pcl::KdTreeFLANN<pcl::PointXYZI> &kdtree,
    const Eigen::Matrix4d &init_guess, float cur_yaw) {
  Eigen::Matrix4d T = GetDeltaT(cur_yaw) * init_guess;
  pcl::PointCloud<pcl::PointXYZI> trans_cloud;

  pcl::transformPointCloud(*src_ngcloud_, trans_cloud, T);
  double dist_sum = 0;
  for (size_t j = 0; j < trans_cloud.points.size(); j++) {
    std::vector<int> indices;
    std::vector<float> distances;
    int k = 1;
    pcl::PointXYZI point = trans_cloud.points[j];
    int size = kdtree.nearestKSearch(point, k, indices, distances);
    if (distances.size() > 0) {
      dist_sum += distances[0];
    } else {
      LOGI("no nearest neighbors found");
    }
  }
  return dist_sum;
}

bool ICPRegistrator::RegistrationByICP2(const Eigen::Matrix4d &init_guess,
                                        Eigen::Matrix4d &refined_extrinsic) {

  pcl::PointCloud<pcl::PointXYZI> trans_cloud;
  pcl::transformPointCloud(*src_cloud_, trans_cloud, init_guess);

  LOGI("compute normals of source points cloud.");
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_before_normal(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  computeNormals(trans_cloud.makeShared(), cloud_before_normal);

  LOGI("compute normals of target points cloud.");
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tgt_normal(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  computeNormals(tgt_cloud_, cloud_tgt_normal);

  pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal,
                                        pcl::PointXYZINormal>
      icp;
  icp.setInputSource(cloud_before_normal);
  icp.setInputTarget(cloud_tgt_normal);
  icp.setMaximumIterations(10);
  // icp.setMaxCorrespondenceDistance(1.0);  // 1.5m
  icp.setMaxCorrespondenceDistance(0.3); // 1.5m
  pcl::PointCloud<pcl::PointXYZINormal> cloud_out;
  icp.align(cloud_out);
  Eigen::Matrix4f transform = icp.getFinalTransformation();
  refined_extrinsic = transform.cast<double>() * init_guess;
  return true;
}

void ICPRegistrator::computeNormals(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_pts,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr out_pts) {
  pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> norm_est;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(40);
  // norm_est.setRadiusSearch(5);
  norm_est.setInputCloud(in_pts);
  norm_est.compute(*out_pts);

  LOGI("normal point cloud number: %d\n", out_pts->size());
  for (int i = 0; i < out_pts->size(); ++i) {
    (*out_pts)[i].x = (*in_pts)[i].x;
    (*out_pts)[i].y = (*in_pts)[i].y;
    (*out_pts)[i].z = (*in_pts)[i].z;
  }
}
