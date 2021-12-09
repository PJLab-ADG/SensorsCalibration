/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */

#pragma once

#include "Eigen/Core"
#include "pnp_solver.hpp"
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace cameracalib {

// 0:getCam2GroundFromH, 1:getCam2GroundFromPNP
#define EXTRINSIC_TRANSFORM_METHOD 1

static inline void getCam2GroundFromH(const Eigen::Matrix3d &homography,
                                      const Eigen::Matrix3d &intrinsic,
                                      Eigen::Matrix3d *cam2ground) {
  Eigen::Matrix3d h_g2cam = homography.inverse();
  h_g2cam /= h_g2cam(2, 2);
  Eigen::Matrix3d K_inv = intrinsic.inverse();
  h_g2cam = K_inv * h_g2cam;
  double lamb0 = 1.0 / h_g2cam.col(0).norm();
  double lamb1 = 1.0 / h_g2cam.col(1).norm();
  (*cam2ground).col(0) = lamb0 * h_g2cam.col(0);
  (*cam2ground).col(1) = lamb1 * h_g2cam.col(1);
  if (h_g2cam(2, 2) > 0) {
    (*cam2ground).col(0) *= -1;
    (*cam2ground).col(1) *= -1;
  }
  (*cam2ground).col(2) = (*cam2ground).col(0).cross((*cam2ground).col(1));

  Eigen::Quaterniond g2c_q((*cam2ground));
  g2c_q.normalize();
  (*cam2ground) = g2c_q.toRotationMatrix();
  (*cam2ground) = (*cam2ground).inverse().eval();
}

static inline void getCam2GroundFromPNP(const Eigen::Matrix3d &homography,
                                        const Eigen::Matrix3d &intrinsic,
                                        Eigen::Matrix3d *cam2ground) {
  std::vector<double> ground_pts_x = {2, 20, 10};
  std::vector<double> ground_pts_y = {-1, 0, 1};
  std::vector<Eigen::Vector2d> img_pts;
  std::vector<Eigen::Vector3d> ground_pts;
  Eigen::Matrix3d h_g2cam = homography.inverse();

  for (size_t i = 0; i < ground_pts_x.size(); i++) {
    for (size_t j = 0; j < ground_pts_y.size(); j++) {
      ground_pts.emplace_back(
          Eigen::Vector3d(ground_pts_x[i], ground_pts_y[j], 0));
      Eigen::Vector3d pixel =
          h_g2cam * Eigen::Vector3d(ground_pts_x[i], ground_pts_y[j], 1);
      pixel /= pixel(2);
      img_pts.emplace_back(Eigen::Vector2d(pixel(0), pixel(1)));
    }
  }
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  solvePnPbyIterative(intrinsic, ground_pts, img_pts, &R, &t);
  *cam2ground = R.inverse();
}

} // cameracalib