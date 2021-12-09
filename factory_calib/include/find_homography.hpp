/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */

#pragma once

#include <time.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <stack>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/SVD"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

typedef Eigen::Matrix<double, 3, 3> Mat3d;
typedef Eigen::Matrix<double, 2, 1> Vec2d;
typedef Eigen::MatrixXd MatXd;
typedef Eigen::VectorXd VecXd;
typedef Eigen::Matrix<double, Eigen::Dynamic, 8> MatX8d;
typedef Eigen::NumTraits<double> EigenDouble;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Matrix<double, 4, 4> Mat4d;

struct Point2float {
  Point2float(float a, float b) : x(a), y(b) {}
  float x;
  float y;
};

struct Point3f {
  Point3f(float a, float b, float c) : x(a), y(b), z(x) {}
  float x;
  float y;
  float z;
};

template <typename T>
void SymmetricGeometricDistanceTerms(const Eigen::Matrix<T, 3, 3> &h_mat,
                                     const Eigen::Matrix<T, 2, 1> &x1,
                                     const Eigen::Matrix<T, 2, 1> &x2,
                                     T forward_error[2], T backward_error[2]);

inline double SymmetricGeometricDistance(const Mat3d &h_mat, const Vec2d &x1,
                                         const Vec2d &x2);

template <typename T = double> class Homography2DNormalizedParameterization {
public:
  typedef Eigen::Matrix<T, 8, 1> Parameters;    // a, b, ... g, h
  typedef Eigen::Matrix<T, 3, 3> Parameterized; // H
  // Convert from the 8 parameters to a H matrix.
  static void To(const Parameters &p, Parameterized *h) {
    *h << p(0), p(1), p(2), p(3), p(4), p(5), p(6), p(7), 1.0;
  }
  // Convert from a H matrix to the 8 parameters.
  static void From(const Parameterized &h, Parameters *p) {
    *p << h(0, 0), h(0, 1), h(0, 2), h(1, 0), h(1, 1), h(1, 2), h(2, 0),
        h(2, 1);
  }
};

inline bool Homography2DFromCorrespondencesLinearEuc(const MatXd &x1,
                                                     const MatXd &x2,
                                                     Mat3d *h_mat,
                                                     double expected_precision);

class HomographySymmetricGeometricCostFunctor {
public:
  HomographySymmetricGeometricCostFunctor(const Vec2d &x, const Vec2d &y)
      : x_(x), y_(y) {}
  template <typename T>
  bool operator()(const T *homography_parameters, T *residuals) const;
  const Vec2d x_;
  const Vec2d y_;
};

struct EstimateHomographyOptions {
  EstimateHomographyOptions()
      : max_num_iterations(50), expected_average_symmetric_distance(1e-16) {}
  int max_num_iterations;
  double expected_average_symmetric_distance;
};

class TerminationCheckingCallback : public ceres::IterationCallback {
public:
  TerminationCheckingCallback(const MatXd &x1, const MatXd &x2,
                              const EstimateHomographyOptions &options,
                              Mat3d *H)
      : options_(options), x1_(x1), x2_(x2), H_(H) {}

  virtual ceres::CallbackReturnType
  operator()(const ceres::IterationSummary &summary);

private:
  const EstimateHomographyOptions &options_;
  const MatXd &x1_;
  const MatXd &x2_;
  Mat3d *H_;
};

inline bool EstimateHomography2DFromCorrespondences(
    const MatXd &x1, const MatXd &x2, const EstimateHomographyOptions &options,
    Mat3d *h_mat);

bool CeresSolveHomography(const std::vector<Point2float> &src_quad,
                          const std::vector<Point2float> &dst_quad,
                          const double expected_symmetric_distance,
                          Mat3d *h_mat);
