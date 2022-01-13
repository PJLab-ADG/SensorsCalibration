#include "registration_icp.hpp"
#include <limits>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>

void ICPRegistrator::SetTargetCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &gcloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &ngcloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  tgt_gcloud_ = gcloud;
  tgt_ngcloud_ = ngcloud;
  tgt_cloud_ = cloud;
}

void ICPRegistrator::SetSourceCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &gcloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &ngcloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  src_gcloud_ = gcloud;
  src_ngcloud_ = ngcloud;
  src_cloud_ = cloud;
}

Eigen::Matrix4d ICPRegistrator::GetFinalTransformation() {
  return final_transformation_;
}

bool ICPRegistrator::RegistrationByICP(const Eigen::Matrix4d &init_guess,
                                       double *refined_yaw) {
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(tgt_ngcloud_);

  double cur_yaw = TransformUtil::GetYaw(init_guess);

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
        // std::cout << "distance decrease to: " << min_error << std::endl;
      }
    }
    search_range = static_cast<int>(search_range / 2 + 0.5);
    step /= 2;
    cur_yaw = best_yaw;
    iter_cnt++;
  }

  // for (int delta = -50; delta < 50; delta++) {
  //   double yaw = cur_yaw + delta * degree_2_radian;
  //   double error = CalculateICPError(kdtree, init_guess, yaw);
  //   if (error < min_error) {
  //     min_error = error;
  //     best_yaw = yaw;
  //     std::cout << "distance decrease to: " << min_error << std::endl;
  //   }
  // }
  *refined_yaw = best_yaw;
  return true;
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

bool ICPRegistrator::RegistrationByNDT(const Eigen::Matrix4d &init_guess,
                                       Eigen::Matrix4d &refined_extrinsic) {
  pcl::PointCloud<pcl::PointXYZ> trans_cloud;
  pcl::transformPointCloud(*src_cloud_, trans_cloud, init_guess);

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setInputSource(trans_cloud.makeShared());
  ndt.setInputTarget(tgt_cloud_);
  ndt.setMaximumIterations(50);
  ndt.setTransformationEpsilon(0.1);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize(0.1);
  ndt.setResolution(1);
  ndt.align(trans_cloud);
  bool is_suceed = ndt.hasConverged();
  double match_score = ndt.getFitnessScore();

  Eigen::Matrix4f transform = ndt.getFinalTransformation();
  refined_extrinsic =
      TransformUtil::Matrix4FloatToDouble(transform) * init_guess;
  return true;
}

bool ICPRegistrator::RegistrationByGICP(const Eigen::Matrix4d &init_guess,
                                        Eigen::Matrix4d &refined_extrinsic) {
  pcl::PointCloud<pcl::PointXYZ> trans_cloud;
  pcl::transformPointCloud(*src_cloud_, trans_cloud, init_guess);

  int max_iter = 50;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setInputSource(trans_cloud.makeShared());
  gicp.setInputTarget(tgt_cloud_);
  gicp.setMaximumIterations(max_iter);
  gicp.align(trans_cloud);
  bool is_suceed = gicp.hasConverged();
  double match_score = gicp.getFitnessScore();
  Eigen::Matrix4f transform = gicp.getFinalTransformation();
  refined_extrinsic =
      TransformUtil::Matrix4FloatToDouble(transform) * init_guess;
  return is_suceed;
}

bool ICPRegistrator::RegistrationByPointToPlane(
    const Eigen::Matrix4d &init_guess, Eigen::Matrix4d &refined_extrinsic) {
  pcl::PointCloud<pcl::PointXYZ> trans_cloud;
  pcl::transformPointCloud(*src_cloud_, trans_cloud, init_guess);

  int max_iter = 10;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setRadiusSearch(0.5);
  pcl::PointCloud<pcl::Normal>::Ptr tgt_norm(new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(tgt_cloud_);
  ne.compute(*tgt_norm);
  pcl::PointCloud<pcl::PointNormal>::Ptr tgt_concat(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*tgt_cloud_, *tgt_norm, *tgt_concat);
  pcl::PointCloud<pcl::Normal>::Ptr src_norm(new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(trans_cloud.makeShared());
  pcl::PointCloud<pcl::PointNormal>::Ptr src_concat(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(trans_cloud, *src_norm, *src_concat);
  pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
  using PointToPlane =
      pcl::registration::TransformationEstimationPointToPlaneLLS<
          pcl::PointNormal, pcl::PointNormal>;
  boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
  icp.setTransformationEstimation(point_to_plane);
  icp.setInputSource(src_concat);
  icp.setInputTarget(tgt_concat);
  icp.setMaximumIterations(max_iter);
  icp.align(*src_concat);
  bool is_suceed = icp.hasConverged();
  double match_score = icp.getFitnessScore();
  Eigen::Matrix4f transform = icp.getFinalTransformation();
  refined_extrinsic =
      TransformUtil::Matrix4FloatToDouble(transform) * init_guess;
  return is_suceed;
}