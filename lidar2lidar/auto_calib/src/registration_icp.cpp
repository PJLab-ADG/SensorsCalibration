#include "registration_icp.hpp"
#include <limits>

#include <pcl/common/transforms.h>

void ICPRegistrator::SetTargetCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &gcloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &ngcloud) {
  tgt_gcloud_ = gcloud;
  tgt_ngcloud_ = ngcloud;
}

void ICPRegistrator::SetSourceCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &gcloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &ngcloud) {
  src_gcloud_ = gcloud;
  src_ngcloud_ = ngcloud;
}

Eigen::Matrix4d ICPRegistrator::GetFinalTransformation() {
  return final_transformation_;
}

void ICPRegistrator::RegistrationByICP(const Eigen::Matrix4d &init_guess,
                                       double *refined_yaw) {
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(tgt_ngcloud_);

  double cur_yaw = TransformUtil::GetYaw(init_guess);
  // Resolution is 1°
  double min_error = CalculateICPError(kdtree, init_guess, cur_yaw);
  double best_yaw = cur_yaw;
  float degree_2_radian = 0.017453293;
  for (int delta = -50; delta < 50; delta++) {
    double yaw = cur_yaw + delta * degree_2_radian;
    double error = CalculateICPError(kdtree, init_guess, yaw);
    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
      std::cout << "distance decrease to: " << min_error << std::endl;
    }
  }
  *refined_yaw = best_yaw;
}

double
ICPRegistrator::CalculateICPError(const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,
                                  const Eigen::Matrix4d &init_guess,
                                  float cur_yaw) {
  // transform to tranformation
  double init_roll = TransformUtil::GetRoll(init_guess);
  double init_pitch = TransformUtil::GetPitch(init_guess);

  Eigen::Matrix4d T = TransformUtil::GetMatrix(
      TransformUtil::GetTranslation(init_guess),
      TransformUtil::GetRotation(init_roll, init_pitch, cur_yaw));
  pcl::PointCloud<pcl::PointXYZ> trans_cloud;

  pcl::transformPointCloud(*src_ngcloud_, trans_cloud, T);
  double dist_sum = 0;
  for (size_t j = 0; j < trans_cloud.points.size(); j++) {
    std::vector<int> indices;
    std::vector<float> distances;
    int k = 1;
    pcl::PointXYZ point = trans_cloud.points[j];
    int size = kdtree.nearestKSearch(point, k, indices, distances);
    if (distances.size() > 0) {
      dist_sum += distances[0];
    } else {
      LOGI("no nearest neighbors found");
    }
  }
  return dist_sum;
}