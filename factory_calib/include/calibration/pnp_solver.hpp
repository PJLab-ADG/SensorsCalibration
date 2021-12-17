/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */

#pragma once

#include "Eigen/Dense"
#include "Eigen/QR"
#include "Eigen/SVD"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <chrono>
#include <vector>

/************** Functions for DLT solver ******************/
bool solvePnPbyDLT(const Eigen::Matrix3d &K,
                   const std::vector<Eigen::Vector3d> &pts3d,
                   const std::vector<Eigen::Vector2d> &pts2d,
                   Eigen::Matrix3d *R, Eigen::Vector3d *t);

/************** Functions for EPnP solver ******************/
bool solvePnPbyEPnP(const Eigen::Matrix3d &K,
                    const std::vector<Eigen::Vector3d> &pts3d,
                    const std::vector<Eigen::Vector2d> &pts2d,
                    Eigen::Matrix3d *R, Eigen::Vector3d *t);

/***** Functions for Iterative solver(same with opencv) ****/
bool solvePnPbyIterative(const Eigen::Matrix3d &K,
                         const std::vector<Eigen::Vector3d> &pts3d,
                         const std::vector<Eigen::Vector2d> &pts2d,
                         Eigen::Matrix3d *R, Eigen::Vector3d *t);

struct CameraProjectFactor {
  CameraProjectFactor(const Eigen::Vector3d &object_pt,
                      const Eigen::Vector2d &img_pt, const Eigen::Matrix3d &K)
      : object_pt_(object_pt), img_pt_(img_pt), K_(K) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Quaternion<T> q_w2cam{q[0], q[1], q[2], q[3]};
    Eigen::Matrix<T, 3, 1> t_w2cam{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> pt_w{T(object_pt_(0)), T(object_pt_(1)),
                                T(object_pt_(2))};
    Eigen::Matrix<T, 2, 1> pt_img{T(img_pt_(0)), T(img_pt_(1))};
    T fx = T(K_(0, 0));
    T fy = T(K_(1, 1));
    T cx = T(K_(0, 2));
    T cy = T(K_(1, 2));
    Eigen::Matrix<T, 3, 1> pt_cam_2;
    pt_cam_2 = q_w2cam * pt_w + t_w2cam;
    if (pt_cam_2(2) != T(0)) {
      pt_cam_2 /= pt_cam_2(2);
    }

    residual[0] = pt_cam_2(0, 0) * fx + cx - pt_img(0, 0);
    residual[1] = pt_cam_2(1, 0) * fy + cy - pt_img(1, 0);
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &object_pt,
                                     const Eigen::Vector2d &img_pt,
                                     const Eigen::Matrix3d &K) {
    return (new ceres::AutoDiffCostFunction<CameraProjectFactor, 2, 4, 3>(
        new CameraProjectFactor(object_pt, img_pt, K)));
  }

  Eigen::Vector3d object_pt_;
  Eigen::Vector2d img_pt_;
  Eigen::Matrix3d K_;
};

template <typename T>
Eigen::MatrixXd covertPointsHomogenous(const std::vector<T> &pts);

void selectControlPoints(const std::vector<Eigen::Vector3d> &pts3d,
                         std::vector<Eigen::Vector3d> *control_points);
void computeHomogeneousBarycentricCoordinates(
    const std::vector<Eigen::Vector3d> &pts3d,
    const std::vector<Eigen::Vector3d> &control_points,
    std::vector<Eigen::Vector4d> *hb_coordinates);
void constructM(const Eigen::Matrix3d &K,
                const std::vector<Eigen::Vector4d> &hb_coordinates,
                const std::vector<Eigen::Vector2d> &pts2d, Eigen::MatrixXd *M);
void getFourEigenVectors(const Eigen::MatrixXd &M,
                         Eigen::Matrix<double, 12, 4> *eigen_vectors);
void computeL(const Eigen::Matrix<double, 12, 4> &eigen_vectors,
              Eigen::Matrix<double, 6, 10> *L);
void computeRho(const std::vector<Eigen::Vector3d> &control_points,
                Eigen::Matrix<double, 6, 1> *rho);
void solveBetaN2(const Eigen::Matrix<double, 12, 4> &eigen_vectors,
                 const Eigen::Matrix<double, 6, 10> &L,
                 const Eigen::Matrix<double, 6, 1> &rho,
                 Eigen::Vector4d *betas);
void solveBetaN3(const Eigen::Matrix<double, 12, 4> &eigen_vectors,
                 const Eigen::Matrix<double, 6, 10> &L,
                 const Eigen::Matrix<double, 6, 1> &rho,
                 Eigen::Vector4d *betas);
void solveBetaN4(const Eigen::Matrix<double, 12, 4> &eigen_vectors,
                 const Eigen::Matrix<double, 6, 10> &L,
                 const Eigen::Matrix<double, 6, 1> &rho,
                 Eigen::Vector4d *betas);
void optimizeBeta(const Eigen::Matrix<double, 6, 10> &L,
                  const Eigen::Matrix<double, 6, 1> &rho,
                  Eigen::Vector4d *betas);
void computeCameraControlPoints(
    const Eigen::Matrix<double, 12, 4> &eigen_vectors,
    const Eigen::Vector4d &betas,
    std::vector<Eigen::Vector3d> *camera_control_points);
bool isGoodBetas(const std::vector<Eigen::Vector3d> &camera_control_points);
void rebuiltPts3dCamera(
    const std::vector<Eigen::Vector3d> &camera_control_points,
    const std::vector<Eigen::Vector4d> &hb_coordinates,
    std::vector<Eigen::Vector3d> *pts3d_camera);
void computeRt(const std::vector<Eigen::Vector3d> &pts3d_camera,
               const std::vector<Eigen::Vector3d> &pts3d_world,
               Eigen::Matrix3d *R, Eigen::Vector3d *t);
double reprojectionError(const Eigen::Matrix3d &K,
                         const std::vector<Eigen::Vector3d> &pts3d_world,
                         const std::vector<Eigen::Vector2d> &pts2d,
                         const Eigen::Matrix3d &R, const Eigen::Vector3d &t);

bool solveLidarPnP(std::vector< std::vector<float> >& objpoints,  std::vector< std::vector<float> >& lidarpoints, 
	  std::vector<float>& rvec, std::vector<float>& tvec);

bool solveCamPnP(std::vector< std::vector<float> >& objpoints,  std::vector< std::vector<float> >& imgpoints, 
	 std::vector< std::vector<double> >& camera_intrinsic,  std::vector<double>& dist_coeffs, std::vector<float>& rvec, std::vector<float>& tvec);