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
  LOGI(" cur_yaw: %f", cur_yaw);
  int iter_cnt = 0;
  int max_iter = 4;
  int cur_sample_half_num = 6;
  double cur_res = 0.14;
  while (iter_cnt < max_iter) {
    double opt_yaw = 0, error = 0;
    CalculateICPError(kdtree, init_guess, cur_yaw, cur_sample_half_num, cur_res,
                      &opt_yaw, &error);
    cur_yaw = opt_yaw;
    cur_sample_half_num = static_cast<int>(cur_sample_half_num / 2 + 0.5);
    cur_res /= 2;
    iter_cnt++;
  }

  *refined_yaw = cur_yaw;
}

void ICPRegistrator::CalculateICPError(
    const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,
    const Eigen::Matrix4d &init_guess, double yaw, int half_num, double res,
    double *refined_yaw, double *cost)

{
  std::vector<std::pair<double, double>> yaw_cost_pairs;
  yaw_cost_pairs.resize(2 * half_num + 1);
  // #pragma omp parallel for num_threads(8)
  double min_dis = DBL_MAX;

  for (int i = -half_num; i < half_num; i++) {
    double cur_yaw = yaw + res * i;
    // transform to tranformation
    double init_roll = TransformUtil::GetRoll(init_guess);
    double init_pitch = TransformUtil::GetPitch(init_guess);

    Eigen::Matrix4d T = TransformUtil::GetMatrix(
        TransformUtil::GetTranslation(init_guess),
        TransformUtil::GetRotation(init_roll, init_pitch, cur_yaw));
    // transform point cloud
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
        return;
      }
    }
    yaw_cost_pairs[i + half_num] = std::make_pair(cur_yaw, dist_sum);

    if (min_dis > dist_sum) {
      min_dis = dist_sum;
      *refined_yaw = cur_yaw;
    }
  }

  // sort by cost
  // std::sort(yaw_cost_pairs.begin(), yaw_cost_pairs.end(),
  //           [&](const std::pair<double, double>& p1,
  //               const std::pair<double, double>& p2) {
  //               return p1.second < p2.second;
  //           });
  // *refined_yaw = yaw_cost_pairs[0].first;
  // *cost = yaw_cost_pairs[0].second;
  *refined_yaw = 0.5;
}