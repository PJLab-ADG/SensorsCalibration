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

bool ICPRegistrator::RegistrationByICP(const Eigen::Matrix4d &init_guess,
                                       double *refined_yaw) {
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
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

double ICPRegistrator::CalculateICPError(
    const pcl::KdTreeFLANN<pcl::PointXYZI> &kdtree,
    const Eigen::Matrix4d &init_guess, float cur_yaw) {
  // transform to tranformation
  double init_roll = TransformUtil::GetRoll(init_guess);
  double init_pitch = TransformUtil::GetPitch(init_guess);

  Eigen::Matrix4d T = TransformUtil::GetMatrix(
      TransformUtil::GetTranslation(init_guess),
      TransformUtil::GetRotation(init_roll, init_pitch, cur_yaw));
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
  refined_extrinsic =
      TransformUtil::Matrix4FloatToDouble(transform) * init_guess;
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

// Fine tune in translation
bool ICPRegistrator::RegistrationByVoxelOccupancy(
    const Eigen::Matrix4d &init_guess, Eigen::Matrix4d &refined_extrinsic) {
  double init_roll = TransformUtil::GetRoll(init_guess);
  double init_pitch = TransformUtil::GetPitch(init_guess);
  double init_yaw = TransformUtil::GetYaw(init_guess);
  double init_x = TransformUtil::GetX(init_guess);
  double init_y = TransformUtil::GetY(init_guess);
  double init_z = TransformUtil::GetZ(init_guess);

  double delta_tx = 0;
  double delta_ty = 0;
  double best_tx = 0;
  double best_ty = 0;
  int max_iter = 10;
  double resolution = 0.02;
  int direction[2] = {1, -1};

  double min_voxel_occupancy = ComputeVoxelOccupancy(init_guess);
  float var[6] = {0}, bestVal[6] = {0};
  std::string varName[6] = {"roll", "pitch", "yaw", "tx", "ty", "tz"};
  // tx
  for (int j = 0; j <= 1; j++) {
    for (int iter = 1; iter < max_iter; iter++) {

      var[3] = iter * direction[j] * resolution;

      Eigen::Matrix4d T = TransformUtil::GetDeltaT(var) * init_guess;
      // delta_tx = iter * direction[j] * resolution + init_x;
      // Eigen::Matrix4d T = TransformUtil::GetMatrix(delta_tx, init_y, init_z,
      // init_roll, init_pitch, init_yaw);

      size_t cnt = ComputeVoxelOccupancy(T);
      if (cnt < min_voxel_occupancy * (1 - 1e-4)) {
        min_voxel_occupancy = cnt;
        // best_tx = delta_tx;
        bestVal[3] = var[3];
        std::cout << "Voxel Occupancy decrease to: " << cnt << std::endl;
      } else {
        std::cout << "Voxel Occupancy increase to: " << cnt << std::endl;
        break;
      }
    }
  }
  var[3] = bestVal[3];
  // ty
  for (int j = 0; j <= 1; j++) {
    for (int iter = 1; iter < max_iter; iter++) {

      var[4] = iter * direction[j] * resolution;
      Eigen::Matrix4d T = TransformUtil::GetDeltaT(var) * init_guess;
      // delta_ty = iter * direction[j] * resolution + init_y;
      // Eigen::Matrix4d T = TransformUtil::GetMatrix(best_tx, delta_ty, init_z,
      // init_roll, init_pitch, init_yaw);
      size_t cnt = ComputeVoxelOccupancy(T);
      if (cnt < min_voxel_occupancy * (1 - 1e-4)) {
        min_voxel_occupancy = cnt;
        // best_tx = delta_tx;
        bestVal[4] = var[4];
        std::cout << "Voxel Occupancy decrease to: " << cnt << std::endl;
      } else {
        std::cout << "Voxel Occupancy increase to: " << cnt << std::endl;
        break;
      }
    }
  }

  refined_extrinsic = TransformUtil::GetDeltaT(bestVal) * init_guess;
}

size_t
ICPRegistrator::ComputeVoxelOccupancy(const Eigen::Matrix4d &init_guess) {
  pcl::PointCloud<pcl::PointXYZI> trans_cloud;
  pcl::transformPointCloud(*src_ngcloud_, trans_cloud, init_guess);
  for (const auto src_pt : tgt_ngcloud_->points) {
    if (!all_octree_->isVoxelOccupiedAtPoint(src_pt)) {
      all_octree_->addPointToCloud(src_pt, all_cloud_);
    }
  }
  for (const auto src_pt : trans_cloud.points) {
    if (!all_octree_->isVoxelOccupiedAtPoint(src_pt)) {
      all_octree_->addPointToCloud(src_pt, all_cloud_);
    }
  }
  size_t cnt = all_cloud_->size();
  all_cloud_->clear();
  all_octree_->deleteTree();
  return cnt;
}