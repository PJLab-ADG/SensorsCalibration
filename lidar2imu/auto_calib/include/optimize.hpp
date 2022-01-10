/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#pragma once

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct BalmVoxelEnergy1 {
  BalmVoxelEnergy1(const Eigen::Vector3d &current_laser_point,
                   const Eigen::Vector3d &point_average,
                   const Eigen::Vector3d &normal,
                   const Eigen::Matrix4d &imu_tran)
      : current_laser_point_(current_laser_point),
        point_average_(point_average), normal_(normal), imu_tran_(imu_tran) {}

  ~BalmVoxelEnergy1() {}

  template <typename T>
  bool operator()(const T *const q, const T *const t, T *residual) const {
    T cur_p[3];
    cur_p[0] = T(current_laser_point_(0));
    cur_p[1] = T(current_laser_point_(1));
    cur_p[2] = T(current_laser_point_(2));
    T world_p[3];
    ceres::QuaternionRotatePoint(q, cur_p, world_p);
    // ceres::AngleAxisRotatePoint(q, cur_p, world_p);
    world_p[0] += t[0];
    world_p[1] += t[1];
    // world_p[2] += t[2];
    T tran_p[3];

    // n * (p - p_aver)
    tran_p[0] = imu_tran_(0, 0) * world_p[0] + imu_tran_(0, 1) * world_p[1] +
                imu_tran_(0, 2) * world_p[2] + imu_tran_(0, 3) -
                point_average_[0];
    tran_p[1] = imu_tran_(1, 0) * world_p[0] + imu_tran_(1, 1) * world_p[1] +
                imu_tran_(1, 2) * world_p[2] + imu_tran_(1, 3) -
                point_average_[1];
    tran_p[2] = imu_tran_(2, 0) * world_p[0] + imu_tran_(2, 1) * world_p[1] +
                imu_tran_(2, 2) * world_p[2] + imu_tran_(2, 3) -
                point_average_[2];

    residual[0] = normal_[0] * tran_p[0] + normal_[1] * tran_p[1] +
                  normal_[2] * tran_p[2];

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &current_laser_point,
                                     const Eigen::Vector3d &point_average,
                                     const Eigen::Vector3d &normal,
                                     const Eigen::Matrix4d &imu_tran) {
    auto cost_function =
        new ceres::AutoDiffCostFunction<BalmVoxelEnergy1, 1, 4, 3>(
            new BalmVoxelEnergy1(current_laser_point, point_average, normal,
                                 imu_tran));
    return cost_function;
  }

  const Eigen::Vector3d current_laser_point_;
  const Eigen::Vector3d point_average_;
  const Eigen::Vector3d normal_;
  const Eigen::Matrix4d imu_tran_;
};

struct BalmVoxelEnergy2 {
  BalmVoxelEnergy2(const Eigen::Vector3d &current_laser_point,
                   const Eigen::Vector3d &point_average,
                   const Eigen::Vector3d &normal,
                   const Eigen::Matrix4d &imu_tran,
                   const Eigen::Matrix4d &imu_tran_orig)
      : current_laser_point_(current_laser_point),
        point_average_(point_average), normal_(normal), imu_tran_(imu_tran),
        imu_tran_orig_(imu_tran_orig) {}

  ~BalmVoxelEnergy2() {}

  template <typename T>
  bool operator()(const T *const q, const T *const t, T *residual) const {
    T cur_p[3];
    cur_p[0] = T(current_laser_point_(0));
    cur_p[1] = T(current_laser_point_(1));
    cur_p[2] = T(current_laser_point_(2));
    T world_p[3];
    ceres::QuaternionRotatePoint(q, cur_p, world_p);
    // ceres::AngleAxisRotatePoint(q, cur_p, world_p);
    world_p[0] += t[0];
    world_p[1] += t[1];
    // world_p[2] += t[2];
    T tran_p[3];

    // transform point average and surface normal
    T tran_point_aver[3];
    T tran_normal[3];
    T orig_point_aver[3];
    T orig_normal[3];
    orig_point_aver[0] = T(point_average_(0));
    orig_point_aver[1] = T(point_average_(1));
    orig_point_aver[2] = T(point_average_(2));
    orig_normal[0] = T(normal_(0));
    orig_normal[1] = T(normal_(1));
    orig_normal[2] = T(normal_(2));
    ceres::QuaternionRotatePoint(q, orig_point_aver, tran_point_aver);
    tran_point_aver[0] += t[0];
    tran_point_aver[1] += t[1];
    // tran_point_aver[2] += t[2];
    ceres::QuaternionRotatePoint(q, orig_normal, tran_normal);
    // imu pose
    T imu_point_aver[3];
    T imu_normal[3];
    imu_point_aver[0] = imu_tran_orig_(0, 0) * tran_point_aver[0] +
                        imu_tran_orig_(0, 1) * tran_point_aver[1] +
                        imu_tran_orig_(0, 2) * tran_point_aver[2] +
                        imu_tran_orig_(0, 3);
    imu_point_aver[1] = imu_tran_orig_(1, 0) * tran_point_aver[0] +
                        imu_tran_orig_(1, 1) * tran_point_aver[1] +
                        imu_tran_orig_(1, 2) * tran_point_aver[2] +
                        imu_tran_orig_(1, 3);
    imu_point_aver[2] = imu_tran_orig_(2, 0) * tran_point_aver[0] +
                        imu_tran_orig_(2, 1) * tran_point_aver[1] +
                        imu_tran_orig_(2, 2) * tran_point_aver[2] +
                        imu_tran_orig_(2, 3);

    imu_normal[0] = imu_tran_orig_(0, 0) * tran_normal[0] +
                    imu_tran_orig_(0, 1) * tran_normal[1] +
                    imu_tran_orig_(0, 2) * tran_normal[2];
    imu_normal[1] = imu_tran_orig_(1, 0) * tran_normal[0] +
                    imu_tran_orig_(1, 1) * tran_normal[1] +
                    imu_tran_orig_(1, 2) * tran_normal[2];
    imu_normal[2] = imu_tran_orig_(2, 0) * tran_normal[0] +
                    imu_tran_orig_(2, 1) * tran_normal[1] +
                    imu_tran_orig_(2, 2) * tran_normal[2];

    // n * (p - p_aver)
    tran_p[0] = imu_tran_(0, 0) * world_p[0] + imu_tran_(0, 1) * world_p[1] +
                imu_tran_(0, 2) * world_p[2] + imu_tran_(0, 3) -
                imu_point_aver[0];
    tran_p[1] = imu_tran_(1, 0) * world_p[0] + imu_tran_(1, 1) * world_p[1] +
                imu_tran_(1, 2) * world_p[2] + imu_tran_(1, 3) -
                imu_point_aver[1];
    tran_p[2] = imu_tran_(2, 0) * world_p[0] + imu_tran_(2, 1) * world_p[1] +
                imu_tran_(2, 2) * world_p[2] + imu_tran_(2, 3) -
                imu_point_aver[2];

    residual[0] = imu_normal[0] * tran_p[0] + imu_normal[1] * tran_p[1] +
                  imu_normal[2] * tran_p[2];

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &current_laser_point,
                                     const Eigen::Vector3d &point_average,
                                     const Eigen::Vector3d &normal,
                                     const Eigen::Matrix4d &imu_tran,
                                     const Eigen::Matrix4d &imu_tran_orig) {
    auto cost_function =
        new ceres::AutoDiffCostFunction<BalmVoxelEnergy2, 1, 4, 3>(
            new BalmVoxelEnergy2(current_laser_point, point_average, normal,
                                 imu_tran, imu_tran_orig));
    return cost_function;
  }

  const Eigen::Vector3d current_laser_point_;
  const Eigen::Vector3d point_average_;
  const Eigen::Vector3d normal_;
  const Eigen::Matrix4d imu_tran_;
  const Eigen::Matrix4d imu_tran_orig_;
};

struct BalmVoxelEnergy2_NOT {
  BalmVoxelEnergy2_NOT(const Eigen::Vector3d &current_laser_point,
                       const Eigen::Vector3d &point_average,
                       const Eigen::Vector3d &normal,
                       const Eigen::Matrix4d &imu_tran,
                       const Eigen::Matrix4d &imu_tran_orig)
      : current_laser_point_(current_laser_point),
        point_average_(point_average), normal_(normal), imu_tran_(imu_tran),
        imu_tran_orig_(imu_tran_orig) {}

  ~BalmVoxelEnergy2_NOT() {}

  template <typename T> bool operator()(const T *const q, T *residual) const {
    T cur_p[3];
    cur_p[0] = T(current_laser_point_(0));
    cur_p[1] = T(current_laser_point_(1));
    cur_p[2] = T(current_laser_point_(2));
    T world_p[3];
    ceres::QuaternionRotatePoint(q, cur_p, world_p);
    // ceres::AngleAxisRotatePoint(q, cur_p, world_p);
    T tran_p[3];

    // transform point average and surface normal
    T tran_point_aver[3];
    T tran_normal[3];
    T orig_point_aver[3];
    T orig_normal[3];
    orig_point_aver[0] = T(point_average_(0));
    orig_point_aver[1] = T(point_average_(1));
    orig_point_aver[2] = T(point_average_(2));
    orig_normal[0] = T(normal_(0));
    orig_normal[1] = T(normal_(1));
    orig_normal[2] = T(normal_(2));
    ceres::QuaternionRotatePoint(q, orig_point_aver, tran_point_aver);
    ceres::QuaternionRotatePoint(q, orig_normal, tran_normal);
    // imu pose
    T imu_point_aver[3];
    T imu_normal[3];
    imu_point_aver[0] = imu_tran_orig_(0, 0) * tran_point_aver[0] +
                        imu_tran_orig_(0, 1) * tran_point_aver[1] +
                        imu_tran_orig_(0, 2) * tran_point_aver[2] +
                        imu_tran_orig_(0, 3);
    imu_point_aver[1] = imu_tran_orig_(1, 0) * tran_point_aver[0] +
                        imu_tran_orig_(1, 1) * tran_point_aver[1] +
                        imu_tran_orig_(1, 2) * tran_point_aver[2] +
                        imu_tran_orig_(1, 3);
    imu_point_aver[2] = imu_tran_orig_(2, 0) * tran_point_aver[0] +
                        imu_tran_orig_(2, 1) * tran_point_aver[1] +
                        imu_tran_orig_(2, 2) * tran_point_aver[2] +
                        imu_tran_orig_(2, 3);

    imu_normal[0] = imu_tran_orig_(0, 0) * tran_normal[0] +
                    imu_tran_orig_(0, 1) * tran_normal[1] +
                    imu_tran_orig_(0, 2) * tran_normal[2];
    imu_normal[1] = imu_tran_orig_(1, 0) * tran_normal[0] +
                    imu_tran_orig_(1, 1) * tran_normal[1] +
                    imu_tran_orig_(1, 2) * tran_normal[2];
    imu_normal[2] = imu_tran_orig_(2, 0) * tran_normal[0] +
                    imu_tran_orig_(2, 1) * tran_normal[1] +
                    imu_tran_orig_(2, 2) * tran_normal[2];

    // n * (p - p_aver)
    tran_p[0] = imu_tran_(0, 0) * world_p[0] + imu_tran_(0, 1) * world_p[1] +
                imu_tran_(0, 2) * world_p[2] + imu_tran_(0, 3) -
                imu_point_aver[0];
    tran_p[1] = imu_tran_(1, 0) * world_p[0] + imu_tran_(1, 1) * world_p[1] +
                imu_tran_(1, 2) * world_p[2] + imu_tran_(1, 3) -
                imu_point_aver[1];
    tran_p[2] = imu_tran_(2, 0) * world_p[0] + imu_tran_(2, 1) * world_p[1] +
                imu_tran_(2, 2) * world_p[2] + imu_tran_(2, 3) -
                imu_point_aver[2];

    residual[0] = imu_normal[0] * tran_p[0] + imu_normal[1] * tran_p[1] +
                  imu_normal[2] * tran_p[2];

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &current_laser_point,
                                     const Eigen::Vector3d &point_average,
                                     const Eigen::Vector3d &normal,
                                     const Eigen::Matrix4d &imu_tran,
                                     const Eigen::Matrix4d &imu_tran_orig) {
    auto cost_function =
        new ceres::AutoDiffCostFunction<BalmVoxelEnergy2_NOT, 1, 4>(
            new BalmVoxelEnergy2_NOT(current_laser_point, point_average, normal,
                                     imu_tran, imu_tran_orig));
    return cost_function;
  }

  const Eigen::Vector3d current_laser_point_;
  const Eigen::Vector3d point_average_;
  const Eigen::Vector3d normal_;
  const Eigen::Matrix4d imu_tran_;
  const Eigen::Matrix4d imu_tran_orig_;
};

struct BalmVoxelEnergy3 {
  BalmVoxelEnergy3(
      const Eigen::Vector3d &cur_laser_point,
      const Eigen::Matrix4d &cur_imu_tran,
      const std::vector<std::vector<Eigen::Vector3d> *> &orig_laser_points,
      const std::vector<pcl::PointXYZINormal> &ap_centor_origns,
      const std::vector<Eigen::Matrix4d> &imu_trans)
      : cur_laser_point_(cur_laser_point), cur_imu_tran_(cur_imu_tran),
        orig_laser_points_(orig_laser_points),
        ap_centor_origns_(ap_centor_origns), imu_trans_(imu_trans) {}

  ~BalmVoxelEnergy3() {}

  template <typename T>
  bool operator()(const T *const q, const T *const t, T *residual) const {
    T real_window_size = T(0);
    T overall_point_aver[3] = {T(0), T(0), T(0)};
    T overall_normal[3] = {T(0), T(0), T(0)};
    // transform point average and surface normal
    for (int i = 0; i < orig_laser_points_.size(); i++) {
      if (orig_laser_points_[i]->size() <= 0)
        continue;
      real_window_size += T(1);
      // int asize = current_laser_points_[i] -> size();
      T tran_point_aver[3];
      T tran_normal[3];
      T orig_point_aver[3];
      T orig_normal[3];
      orig_point_aver[0] = T(ap_centor_origns_[i].x);
      orig_point_aver[1] = T(ap_centor_origns_[i].y);
      orig_point_aver[2] = T(ap_centor_origns_[i].z);
      orig_normal[0] = T(ap_centor_origns_[i].normal_x);
      orig_normal[1] = T(ap_centor_origns_[i].normal_y);
      orig_normal[2] = T(ap_centor_origns_[i].normal_z);
      ceres::QuaternionRotatePoint(q, orig_point_aver, tran_point_aver);
      tran_point_aver[0] += t[0];
      tran_point_aver[1] += t[1];
      // tran_point_aver[2] += t[2];
      ceres::QuaternionRotatePoint(q, orig_normal, tran_normal);
      Eigen::Matrix4d imu_tran_orign = imu_trans_[i];

      // imu pose
      T imu_point_aver[3];
      T imu_normal[3];
      imu_point_aver[0] = imu_tran_orign(0, 0) * tran_point_aver[0] +
                          imu_tran_orign(0, 1) * tran_point_aver[1] +
                          imu_tran_orign(0, 2) * tran_point_aver[2] +
                          imu_tran_orign(0, 3);
      imu_point_aver[1] = imu_tran_orign(1, 0) * tran_point_aver[0] +
                          imu_tran_orign(1, 1) * tran_point_aver[1] +
                          imu_tran_orign(1, 2) * tran_point_aver[2] +
                          imu_tran_orign(1, 3);
      imu_point_aver[2] = imu_tran_orign(2, 0) * tran_point_aver[0] +
                          imu_tran_orign(2, 1) * tran_point_aver[1] +
                          imu_tran_orign(2, 2) * tran_point_aver[2] +
                          imu_tran_orign(2, 3);

      imu_normal[0] = imu_tran_orign(0, 0) * tran_normal[0] +
                      imu_tran_orign(0, 1) * tran_normal[1] +
                      imu_tran_orign(0, 2) * tran_normal[2];
      imu_normal[1] = imu_tran_orign(1, 0) * tran_normal[0] +
                      imu_tran_orign(1, 1) * tran_normal[1] +
                      imu_tran_orign(1, 2) * tran_normal[2];
      imu_normal[2] = imu_tran_orign(2, 0) * tran_normal[0] +
                      imu_tran_orign(2, 1) * tran_normal[1] +
                      imu_tran_orign(2, 2) * tran_normal[2];
      // add to overall point average
      overall_point_aver[0] += imu_point_aver[0];
      overall_point_aver[1] += imu_point_aver[1];
      overall_point_aver[2] += imu_point_aver[2];
      overall_normal[0] += imu_normal[0];
      overall_normal[1] += imu_normal[1];
      overall_normal[2] += imu_normal[2];
    }
    overall_point_aver[0] /= real_window_size;
    overall_point_aver[1] /= real_window_size;
    overall_point_aver[2] /= real_window_size;
    overall_normal[0] /= real_window_size;
    overall_normal[1] /= real_window_size;
    overall_normal[2] /= real_window_size;

    // add residuals
    T cur_p[3];
    cur_p[0] = T(cur_laser_point_(0));
    cur_p[1] = T(cur_laser_point_(1));
    cur_p[2] = T(cur_laser_point_(2));
    T world_p[3];
    ceres::QuaternionRotatePoint(q, cur_p, world_p);
    // ceres::AngleAxisRotatePoint(q, cur_p, world_p);
    world_p[0] += t[0];
    world_p[1] += t[1];
    // world_p[2] += t[2];
    T tran_p[3];

    // n * (p - p_aver)
    tran_p[0] = cur_imu_tran_(0, 0) * world_p[0] +
                cur_imu_tran_(0, 1) * world_p[1] +
                cur_imu_tran_(0, 2) * world_p[2] + cur_imu_tran_(0, 3) -
                overall_point_aver[0];
    tran_p[1] = cur_imu_tran_(1, 0) * world_p[0] +
                cur_imu_tran_(1, 1) * world_p[1] +
                cur_imu_tran_(1, 2) * world_p[2] + cur_imu_tran_(1, 3) -
                overall_point_aver[1];
    tran_p[2] = cur_imu_tran_(2, 0) * world_p[0] +
                cur_imu_tran_(2, 1) * world_p[1] +
                cur_imu_tran_(2, 2) * world_p[2] + cur_imu_tran_(2, 3) -
                overall_point_aver[2];

    residual[0] = overall_normal[0] * tran_p[0] +
                  overall_normal[1] * tran_p[1] + overall_normal[2] * tran_p[2];
    return true;
  }

  static ceres::CostFunction *
  Create(const Eigen::Vector3d &cur_laser_point,
         const Eigen::Matrix4d &cur_imu_tran,
         const std::vector<std::vector<Eigen::Vector3d> *> &orig_laser_points,
         const std::vector<pcl::PointXYZINormal> &ap_centor_origns,
         const std::vector<Eigen::Matrix4d> &imu_trans) {
    auto cost_function =
        new ceres::AutoDiffCostFunction<BalmVoxelEnergy3, 1, 4, 3>(
            new BalmVoxelEnergy3(cur_laser_point, cur_imu_tran,
                                 orig_laser_points, ap_centor_origns,
                                 imu_trans));
    // cost_function.SetNumResiduals(point_size);
    return cost_function;
  }

  const Eigen::Vector3d cur_laser_point_;
  const Eigen::Matrix4d cur_imu_tran_;
  const std::vector<std::vector<Eigen::Vector3d> *> orig_laser_points_;
  const std::vector<pcl::PointXYZINormal> ap_centor_origns_;
  const std::vector<Eigen::Matrix4d> imu_trans_;
};

struct BalmVoxelTransEnergy {
  BalmVoxelTransEnergy(const Eigen::Vector3d &current_laser_point,
                       const Eigen::Vector3d &point_average,
                       const Eigen::Vector3d &normal,
                       const Eigen::Matrix4d &imu_tran,
                       const Eigen::Matrix4d &imu_tran_orig,
                       const Eigen::Matrix4d &refined_calib_mat)
      : current_laser_point_(current_laser_point),
        point_average_(point_average), normal_(normal), imu_tran_(imu_tran),
        imu_tran_orig_(imu_tran_orig), refined_calib_mat_(refined_calib_mat) {}

  ~BalmVoxelTransEnergy() {}

  template <typename T> bool operator()(const T *const t, T *residual) const {
    T cur_p[3];
    cur_p[0] = T(current_laser_point_(0));
    cur_p[1] = T(current_laser_point_(1));
    cur_p[2] = T(current_laser_point_(2));
    T world_p[3];
    world_p[0] = refined_calib_mat_(0, 0) * cur_p[0] +
                 refined_calib_mat_(0, 1) * cur_p[1] +
                 refined_calib_mat_(0, 2) * cur_p[2] + refined_calib_mat_(0, 3);
    world_p[1] = refined_calib_mat_(1, 0) * cur_p[0] +
                 refined_calib_mat_(1, 1) * cur_p[1] +
                 refined_calib_mat_(1, 2) * cur_p[2] + refined_calib_mat_(1, 3);
    world_p[2] = refined_calib_mat_(2, 0) * cur_p[0] +
                 refined_calib_mat_(2, 1) * cur_p[1] +
                 refined_calib_mat_(2, 2) * cur_p[2] + t[2];
    T tran_p[3];

    // transform point average and surface normal
    T tran_point_aver[3];
    T tran_normal[3];
    T orig_point_aver[3];
    T orig_normal[3];
    orig_point_aver[0] = T(point_average_(0));
    orig_point_aver[1] = T(point_average_(1));
    orig_point_aver[2] = T(point_average_(2));
    orig_normal[0] = T(normal_(0));
    orig_normal[1] = T(normal_(1));
    orig_normal[2] = T(normal_(2));
    tran_point_aver[0] = refined_calib_mat_(0, 0) * orig_point_aver[0] +
                         refined_calib_mat_(0, 1) * orig_point_aver[1] +
                         refined_calib_mat_(0, 2) * orig_point_aver[2] +
                         refined_calib_mat_(0, 3);
    tran_point_aver[1] = refined_calib_mat_(1, 0) * orig_point_aver[0] +
                         refined_calib_mat_(1, 1) * orig_point_aver[1] +
                         refined_calib_mat_(1, 2) * orig_point_aver[2] +
                         refined_calib_mat_(1, 3);
    tran_point_aver[2] = refined_calib_mat_(2, 0) * orig_point_aver[0] +
                         refined_calib_mat_(2, 1) * orig_point_aver[1] +
                         refined_calib_mat_(2, 2) * orig_point_aver[2] + t[2];
    tran_normal[0] = refined_calib_mat_(0, 0) * orig_normal[0] +
                     refined_calib_mat_(0, 1) * orig_normal[1] +
                     refined_calib_mat_(0, 2) * orig_normal[2];
    tran_normal[1] = refined_calib_mat_(1, 0) * orig_normal[0] +
                     refined_calib_mat_(1, 1) * orig_normal[1] +
                     refined_calib_mat_(1, 2) * orig_normal[2];
    tran_normal[2] = refined_calib_mat_(2, 0) * orig_normal[0] +
                     refined_calib_mat_(2, 1) * orig_normal[1] +
                     refined_calib_mat_(2, 2) * orig_normal[2];
    // imu pose
    T imu_point_aver[3];
    T imu_normal[3];
    imu_point_aver[0] = imu_tran_orig_(0, 0) * tran_point_aver[0] +
                        imu_tran_orig_(0, 1) * tran_point_aver[1] +
                        imu_tran_orig_(0, 2) * tran_point_aver[2] +
                        imu_tran_orig_(0, 3);
    imu_point_aver[1] = imu_tran_orig_(1, 0) * tran_point_aver[0] +
                        imu_tran_orig_(1, 1) * tran_point_aver[1] +
                        imu_tran_orig_(1, 2) * tran_point_aver[2] +
                        imu_tran_orig_(1, 3);
    imu_point_aver[2] = imu_tran_orig_(2, 0) * tran_point_aver[0] +
                        imu_tran_orig_(2, 1) * tran_point_aver[1] +
                        imu_tran_orig_(2, 2) * tran_point_aver[2] +
                        imu_tran_orig_(2, 3);

    imu_normal[0] = imu_tran_orig_(0, 0) * tran_normal[0] +
                    imu_tran_orig_(0, 1) * tran_normal[1] +
                    imu_tran_orig_(0, 2) * tran_normal[2];
    imu_normal[1] = imu_tran_orig_(1, 0) * tran_normal[0] +
                    imu_tran_orig_(1, 1) * tran_normal[1] +
                    imu_tran_orig_(1, 2) * tran_normal[2];
    imu_normal[2] = imu_tran_orig_(2, 0) * tran_normal[0] +
                    imu_tran_orig_(2, 1) * tran_normal[1] +
                    imu_tran_orig_(2, 2) * tran_normal[2];

    // n * (p - p_aver)
    tran_p[0] = imu_tran_(0, 0) * world_p[0] + imu_tran_(0, 1) * world_p[1] +
                imu_tran_(0, 2) * world_p[2] + imu_tran_(0, 3) -
                imu_point_aver[0];
    tran_p[1] = imu_tran_(1, 0) * world_p[0] + imu_tran_(1, 1) * world_p[1] +
                imu_tran_(1, 2) * world_p[2] + imu_tran_(1, 3) -
                imu_point_aver[1];
    tran_p[2] = imu_tran_(2, 0) * world_p[0] + imu_tran_(2, 1) * world_p[1] +
                imu_tran_(2, 2) * world_p[2] + imu_tran_(2, 3) -
                imu_point_aver[2];

    residual[0] = imu_normal[0] * tran_p[0] + imu_normal[1] * tran_p[1] +
                  imu_normal[2] * tran_p[2];

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &current_laser_point,
                                     const Eigen::Vector3d &point_average,
                                     const Eigen::Vector3d &normal,
                                     const Eigen::Matrix4d &imu_tran,
                                     const Eigen::Matrix4d &imu_tran_orig,
                                     const Eigen::Matrix4d &refined_calib_mat) {
    auto cost_function =
        new ceres::AutoDiffCostFunction<BalmVoxelTransEnergy, 1, 3>(
            new BalmVoxelTransEnergy(current_laser_point, point_average, normal,
                                     imu_tran, imu_tran_orig,
                                     refined_calib_mat));
    return cost_function;
  }

  const Eigen::Vector3d current_laser_point_;
  const Eigen::Vector3d point_average_;
  const Eigen::Vector3d normal_;
  const Eigen::Matrix4d imu_tran_;
  const Eigen::Matrix4d imu_tran_orig_;
  const Eigen::Matrix4d refined_calib_mat_;
};
