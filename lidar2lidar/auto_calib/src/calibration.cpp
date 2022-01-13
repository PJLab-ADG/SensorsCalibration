/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include "calibration.hpp"

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "plane_ground_filter_core.h"

Calibrator::Calibrator() { registrator_.reset(new ICPRegistrator); }

void Calibrator::LoadCalibrationData(
    const std::map<int32_t, pcl::PointCloud<pcl::PointXYZ>> lidar_points,
    const std::map<int32_t, InitialExtrinsic> extrinsics) {
  pcs_ = lidar_points;
  for (auto src : extrinsics) {
    int32_t device_id = src.first;
    InitialExtrinsic extrinsic = src.second;

    Eigen::Matrix3d rot = TransformUtil::GetRotation(extrinsic.euler_angles[0],
                                                     extrinsic.euler_angles[1],
                                                     extrinsic.euler_angles[2]);
    Eigen::Matrix4d init_ext =
        TransformUtil::GetMatrix(extrinsic.t_matrix, rot);
    init_extrinsics_.insert(std::make_pair(device_id, init_ext));
  }
}

void Calibrator::Calibrate() {
  float degree_2_radian = 0.017453293;
  LOGI("calibrate");
  int32_t master_id = 0;
  auto master_iter = pcs_.find(master_id);
  pcl::PointCloud<pcl::PointXYZ> master_pc = master_iter->second;
  pcl::PointCloud<pcl::PointXYZ>::Ptr master_pc_ptr = master_pc.makeShared();
  PlaneParam master_gplane;
  pcl::PointCloud<pcl::PointXYZ>::Ptr master_gcloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr master_ngcloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  bool ret = GroundPlaneExtraction(master_pc_ptr, master_gcloud, master_ngcloud,
                                   master_gplane);
  if (!ret) {
    LOGE("master lidar ground fitting failed.\n");
    return;
  }
  registrator_->SetTargetCloud(master_gcloud, master_ngcloud, master_pc_ptr);
  Eigen::Vector3d t_mp(0, 0,
                       -master_gplane.intercept / master_gplane.normal(2));

  for (auto iter = pcs_.begin(); iter != pcs_.end(); iter++) {
    int32_t slave_id = iter->first;
    LOGI("slave %d begin", slave_id);
    if (slave_id == master_id)
      continue;
    LOGI("start calibrating slave lidar, id: %d\n", slave_id);
    pcl::PointCloud<pcl::PointXYZ> slave_pc = iter->second;
    if (init_extrinsics_.find(slave_id) == init_extrinsics_.end()) {
      LOGE("cannot find the init extrinsic, slave id: %d\n", slave_id);
      return;
    }
    Eigen::Matrix4d init_ext = init_extrinsics_[slave_id];
    LOGI("init extrinsic T_ms is: roll = %f, pitch = %f, yaw =  %f, x = %f, "
         "y = %f, z = %f\n",
         TransformUtil::GetRoll(init_ext) / degree_2_radian,
         TransformUtil::GetPitch(init_ext) / degree_2_radian,
         TransformUtil::GetYaw(init_ext) / degree_2_radian, init_ext(0, 3),
         init_ext(1, 3), init_ext(2, 3));
    pcl::PointCloud<pcl::PointXYZ>::Ptr slave_pc_ptr = slave_pc.makeShared();
    PlaneParam slave_gplane;
    pcl::PointCloud<pcl::PointXYZ>::Ptr slave_gcloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr slave_ngcloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    ret = GroundPlaneExtraction(slave_pc_ptr, slave_gcloud, slave_ngcloud,
                                slave_gplane);
    if (!ret) {
      LOGE("slave %d lidar ground fitting failed.\n", slave_id);
      continue;
    }
    registrator_->SetSourceCloud(slave_gcloud, slave_ngcloud, slave_pc_ptr);
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
    Eigen::Matrix4d T_ms = TransformUtil::GetMatrix(t_ms, R_ms);
    double z_error = std::fabs(t_ms(2) - init_ext(2, 3));
    if (z_error > 0.5) {
      // maybe the direction is diffetent
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
      T_ms = TransformUtil::GetMatrix(t_ms, R_ms);
      z_error = std::fabs(t_ms(2) - init_ext(2, 3));
      if (z_error > 0.5) {
        LOGE(
            "slave %d ground fitting failed, error: z-diff error is too big.\n",
            slave_id);
        continue;
      }
    }

    double roll = TransformUtil::GetRoll(T_ms);
    double pitch = TransformUtil::GetPitch(T_ms);
    double z = TransformUtil::GetZ(T_ms);
    double init_x = TransformUtil::GetX(init_ext);
    double init_y = TransformUtil::GetY(init_ext);
    double init_yaw = TransformUtil::GetYaw(init_ext);
    Eigen::Matrix4d init_guess =
        TransformUtil::GetMatrix(init_x, init_y, z, roll, pitch, init_yaw);
    LOGI("ground plane param align, roll = %f, pitch = %f, z = %f\n", roll,
         pitch, z);
    // registration
    double refined_yaw = 0;
    // YawAligh(master_ngcloud, slave_ngcloud, init_guess, &refined_yaw);
    registrator_->RegistrationByICP(init_guess, &refined_yaw);
    // Eigen::Matrix4d refine_trans = registrator_->GetFinalTransformation();
    double init_roll = TransformUtil::GetRoll(init_guess);
    double init_pitch = TransformUtil::GetPitch(init_guess);
    Eigen::Matrix4d yaw_opt_resust = TransformUtil::GetMatrix(
        TransformUtil::GetTranslation(init_guess),
        TransformUtil::GetRotation(init_roll, init_pitch, refined_yaw));
    Eigen::Matrix4d final_opt_result;
    // registrator_->RegistrationByNDT(yaw_opt_resust, final_opt_result);
    // registrator_->RegistrationByGICP(yaw_opt_resust, final_opt_result);
    // registrator_->RegistrationByPointToPlane(yaw_opt_resust,
    // final_opt_result);
    refined_extrinsics_.insert(std::make_pair(slave_id, yaw_opt_resust));
  }
}

bool Calibrator::GroundPlaneExtraction(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ng_cloud, PlaneParam &plane) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
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
  pcl::ExtractIndices<pcl::PointXYZ> extract;
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