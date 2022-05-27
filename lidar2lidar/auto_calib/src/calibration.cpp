/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include "calibration.hpp"

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

const double eps = 1.0e-6;

Calibrator::Calibrator() { registrator_.reset(new ICPRegistrator); }

void Calibrator::LoadCalibrationData(
    const std::map<int32_t, pcl::PointCloud<pcl::PointXYZI>> lidar_points,
    const std::map<int32_t, InitialExtrinsic> extrinsics) {
  pcs_ = lidar_points;
  for (auto src : extrinsics) {
    int32_t device_id = src.first;
    InitialExtrinsic extrinsic = src.second;
    Eigen::Matrix3d rotation;
    Eigen::AngleAxisd Rx(
        Eigen::AngleAxisd(extrinsic.euler_angles[0], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd Ry(
        Eigen::AngleAxisd(extrinsic.euler_angles[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd Rz(
        Eigen::AngleAxisd(extrinsic.euler_angles[2], Eigen::Vector3d::UnitZ()));
    rotation = Rz * Ry * Rx;
    Eigen::Matrix3d rot = rotation;
    Eigen::Matrix4d init_ext = Eigen::Matrix4d::Identity();
    init_ext.block<3, 1>(0, 3) = extrinsic.t_matrix;
    init_ext.block<3, 3>(0, 0) = rot;
    init_extrinsics_.insert(std::make_pair(device_id, init_ext));
  }
}

void Calibrator::Calibrate() {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d curr_transform = Eigen::Matrix4d::Identity();

  int32_t master_id = 0;
  auto master_iter = pcs_.find(master_id);
  pcl::PointCloud<pcl::PointXYZI> master_pc = master_iter->second;
  pcl::PointCloud<pcl::PointXYZI>::Ptr master_pc_ptr = master_pc.makeShared();
  PlaneParam master_gplane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr master_gcloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr master_ngcloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  bool ret = GroundPlaneExtraction(master_pc_ptr, master_gcloud, master_ngcloud,
                                   master_gplane);
  if (!ret) {
    LOGE("ground plane extraction failed.\n");
    return;
  }
  registrator_->SetTargetCloud(master_gcloud, master_ngcloud, master_pc_ptr);
  Eigen::Vector3d t_mp(0, 0,
                       -master_gplane.intercept / master_gplane.normal(2));

  for (auto iter = pcs_.begin(); iter != pcs_.end(); iter++) {
    int32_t slave_id = iter->first;
    if (slave_id == master_id)
      continue;
    pcl::PointCloud<pcl::PointXYZI> slave_pc = iter->second;
    pcl::PointCloud<pcl::PointXYZI> slave_original_pc = slave_pc;
    if (init_extrinsics_.find(slave_id) == init_extrinsics_.end()) {
      LOGE("cannot find the init extrinsic, id: %d\n", slave_id);
      return;
    }
    Eigen::Matrix4d init_ext = init_extrinsics_[slave_id];
    pcl::PointCloud<pcl::PointXYZI>::Ptr slave_pc_ptr = slave_pc.makeShared();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_Condition(
        new pcl::PointCloud<pcl::PointXYZI>);
    PlaneParam slave_gplane;
    pcl::PointCloud<pcl::PointXYZI>::Ptr slave_gcloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr slave_ngcloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    // earse the points close to LiDAR
    if (slave_id) {

      pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_condition(
          new pcl::ConditionAnd<pcl::PointXYZI>());
      range_condition->addComparison(
          pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
              new pcl::FieldComparison<pcl::PointXYZI>(
                  "x", pcl::ComparisonOps::GT, -1)));
      range_condition->addComparison(
          pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
              new pcl::FieldComparison<pcl::PointXYZI>(
                  "x", pcl::ComparisonOps::LT, 1))); //
      range_condition->addComparison(
          pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
              new pcl::FieldComparison<pcl::PointXYZI>(
                  "y", pcl::ComparisonOps::GT, -1.0)));
      range_condition->addComparison(
          pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
              new pcl::FieldComparison<pcl::PointXYZI>(
                  "y", pcl::ComparisonOps::LT, 1)));
      range_condition->addComparison(
          pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
              new pcl::FieldComparison<pcl::PointXYZI>(
                  "z", pcl::ComparisonOps::GT, -1)));
      range_condition->addComparison(
          pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
              new pcl::FieldComparison<pcl::PointXYZI>(
                  "z", pcl::ComparisonOps::LT, 1)));

      pcl::ConditionalRemoval<pcl::PointXYZI> condition;
      condition.setCondition(range_condition);
      condition.setInputCloud(slave_pc_ptr);
      condition.setKeepOrganized(false);
      condition.filter(*cloud_after_Condition);

      pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
      pcl::PointXYZI searchPoint;
      int K = 1;
      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);
      std::vector<pcl::PointXYZI> DeleteData;
      int num = 0;
      for (auto iter = cloud_after_Condition->begin();
           iter != cloud_after_Condition->end(); iter++) {
        searchPoint.x = iter->x;
        searchPoint.y = iter->y;
        searchPoint.z = iter->z;
        kdtree.setInputCloud(slave_pc_ptr);
        num = kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                                    pointNKNSquaredDistance);
        if (num > 0) {
          if (sqrt(pointNKNSquaredDistance[0]) < eps) {
            auto iterB = slave_pc_ptr->begin() + pointIdxNKNSearch[0];
            slave_pc_ptr->erase(iterB);
            DeleteData.push_back(searchPoint);
            if (slave_pc_ptr->size() == 0) {
              break;
            }
            searchPoint.x = 0;
            searchPoint.y = 0;
            searchPoint.z = 0;
            num = 0;
            pointIdxNKNSearch.clear();
            pointNKNSquaredDistance.clear();
          }
        }
      }
    }

    ret = GroundPlaneExtraction(slave_pc_ptr, slave_gcloud, slave_ngcloud,
                                slave_gplane);
    if (!ret) {
      LOGE("ground plane extraction failed.\n");
      continue;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr slave_original_pc_ptr =
        slave_original_pc.makeShared();
    PlaneParam slave_original_gplane;
    pcl::PointCloud<pcl::PointXYZI>::Ptr slave_original_ngcloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr slave_original_gcloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    ret = GroundPlaneExtraction(slave_original_pc_ptr, slave_original_gcloud,
                                slave_original_ngcloud, slave_original_gplane);
    registrator_->SetSourceCloud(slave_original_gcloud, slave_original_ngcloud,
                                 slave_original_pc_ptr);

    // ground normal direction
    Eigen::Vector3f ground_point(
        0, 0, (slave_gplane.intercept) / (-slave_gplane.normal(2)));
    Eigen::Vector3f point2plane_vector;
    int Ontheground = 0;
    int Undertheground = 0;
    for (auto iter = slave_ngcloud->begin(); iter < slave_ngcloud->end() - 100;
         iter += 100) {
      Eigen::Vector3f samplePoint(iter->x, iter->y, iter->z);
      point2plane_vector = samplePoint - ground_point;
      if ((point2plane_vector(0) * slave_gplane.normal(0) +
           point2plane_vector(1) * slave_gplane.normal(1) +
           point2plane_vector(2) * slave_gplane.normal(2)) >= 0) {
        Ontheground++;
      } else {
        Undertheground++;
      }
    }
    // ground plane align
    Eigen::Vector3d rot_axis2 = slave_gplane.normal.cross(master_gplane.normal);
    rot_axis2.normalize();
    double alpha2 = std::acos(slave_gplane.normal.dot(master_gplane.normal));
    Eigen::Matrix3d R_ms;
    R_ms = Eigen::AngleAxisd(alpha2, rot_axis2);
    Eigen::Vector3d slave_intcpt_local(
        0, 0, -slave_gplane.intercept / slave_gplane.normal(2));
    Eigen::Vector3d slave_intcpt_master = R_ms * slave_intcpt_local;
    Eigen::Vector3d t_ms(0, 0, t_mp(2) - slave_intcpt_master(2));

    Eigen::Matrix4d T_ms = Eigen::Matrix4d::Identity();
    T_ms.block<3, 1>(0, 3) = t_ms;
    T_ms.block<3, 3>(0, 0) = R_ms;

    double z_error = std::fabs(t_ms(2) - init_ext(2, 3));
    if (z_error > 0.5) {
      slave_gplane.normal = -slave_gplane.normal;
      slave_gplane.intercept = -slave_gplane.intercept;
      rot_axis2 = slave_gplane.normal.cross(master_gplane.normal);
      rot_axis2.normalize();
      alpha2 = std::acos(slave_gplane.normal.dot(master_gplane.normal));
      R_ms = Eigen::AngleAxisd(alpha2, rot_axis2);
      slave_intcpt_local = Eigen::Vector3d(
          0, 0, -slave_gplane.intercept / slave_gplane.normal(2));
      slave_intcpt_master = R_ms * slave_intcpt_local;
      t_ms = Eigen::Vector3d(0, 0, t_mp(2) - slave_intcpt_master(2));
      T_ms.block<3, 1>(0, 3) = t_ms;
      T_ms.block<3, 3>(0, 0) = R_ms;
    }
    curr_transform = init_ext * T_ms;
    registrator_->RegistrationByICP(curr_transform, transform);
    Eigen::Matrix4d final_opt_result;
    registrator_->RegistrationByICP2(transform, final_opt_result);
    refined_extrinsics_.insert(std::make_pair(slave_id, final_opt_result));
  }
}

bool Calibrator::GroundPlaneExtraction(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud, PlaneParam &plane) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.2);
  seg.setInputCloud(in_cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return false;
  }
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(inliers);
  extract.filter(*g_cloud);
  extract.setNegative(true);
  extract.filter(*ng_cloud);
  plane.normal(0) = coefficients->values[0];
  plane.normal(1) = coefficients->values[1];
  plane.normal(2) = coefficients->values[2];
  plane.intercept = coefficients->values[3];
  return true;
}

std::map<int32_t, Eigen::Matrix4d> Calibrator::GetFinalTransformation() {
  return refined_extrinsics_;
}