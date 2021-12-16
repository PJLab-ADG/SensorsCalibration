/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */

#include "calibration/find_homography.hpp"

// Calculate symmetric geometric cost terms:
// forward_error = D(H * x1, x2)
// backward_error = D(H^-1 * x2, x1)

// Templated to be used with autodifferentiation.
template <typename T>
void SymmetricGeometricDistanceTerms(const Eigen::Matrix<T, 3, 3> &h_mat,
                                     const Eigen::Matrix<T, 2, 1> &x1,
                                     const Eigen::Matrix<T, 2, 1> &x2,
                                     T forward_error[2], T backward_error[2]) {
  typedef Eigen::Matrix<T, 3, 1> Vec3;
  Vec3 x(x1(0), x1(1), T(1.0));
  Vec3 y(x2(0), x2(1), T(1.0));
  Vec3 H_x = h_mat * x;
  Vec3 Hinv_y = h_mat.inverse() * y;
  H_x /= H_x(2);
  Hinv_y /= Hinv_y(2);
  forward_error[0] = H_x(0) - y(0);
  forward_error[1] = H_x(1) - y(1);
  backward_error[0] = Hinv_y(0) - x(0);
  backward_error[1] = Hinv_y(1) - x(1);
}

// Calculate symmetric geometric cost:
//
// D(H * x1, x2)^2 + D(H^-1 * x2, x1)^2
inline double SymmetricGeometricDistance(const Mat3d &h_mat, const Vec2d &x1,
                                         const Vec2d &x2) {
  Vec2d forward_error, backward_error;
  SymmetricGeometricDistanceTerms<double>(h_mat, x1, x2, forward_error.data(),
                                          backward_error.data());
  return forward_error.squaredNorm() + backward_error.squaredNorm();
}

// 2D Homography transformation estimation in the case that points are in
// euclidean coordinates.
//
//   x = H y
//
// x and y vector must have the same direction, we could write
//
//   crossproduct(|x|, * H * |y| ) = |0|
//
//   | 0 -1  x2|   |a b c|   |y1|    |0|
//   | 1  0 -x1| * |d e f| * |y2| =  |0|
//   |-x2  x1 0|   |g h 1|   |1 |    |0|
//
// That gives:
//
//   (-d+x2*g)*y1    + (-e+x2*h)*y2 + -f+x2          |0|
//   (a-x1*g)*y1     + (b-x1*h)*y2  + c-x1         = |0|
//   (-x2*a+x1*d)*y1 + (-x2*b+x1*e)*y2 + -x2*c+x1*f  |0|
//
inline bool Homography2DFromCorrespondencesLinearEuc(
    const MatXd &x1, const MatXd &x2, Mat3d *h_mat, double expected_precision) {
  if (2 != x1.rows() || 4 > x1.cols() || x1.rows() != x2.rows() ||
      x1.cols() != x2.cols()) {
    // std::cout<< "Size not consistent!"<<std::endl;
    return false;
  }
  int n = x1.cols();
  MatX8d L = MatXd::Zero(n * 3, 8);
  MatXd b = MatXd::Zero(n * 3, 1);
  for (int i = 0; i < n; ++i) {
    int j = 3 * i;
    L(j, 0) = x1(0, i);             // a
    L(j, 1) = x1(1, i);             // b
    L(j, 2) = 1.0;                  // c
    L(j, 6) = -x2(0, i) * x1(0, i); // g
    L(j, 7) = -x2(0, i) * x1(1, i); // h
    b(j, 0) = x2(0, i);             // i
    ++j;
    L(j, 3) = x1(0, i);             // d
    L(j, 4) = x1(1, i);             // e
    L(j, 5) = 1.0;                  // f
    L(j, 6) = -x2(1, i) * x1(0, i); // g
    L(j, 7) = -x2(1, i) * x1(1, i); // h
    b(j, 0) = x2(1, i);             // i
    // This ensures better stability
    // make a lite version without this 3rd set
    ++j;
    L(j, 0) = x2(1, i) * x1(0, i);  // a
    L(j, 1) = x2(1, i) * x1(1, i);  // b
    L(j, 2) = x2(1, i);             // c
    L(j, 3) = -x2(0, i) * x1(0, i); // d
    L(j, 4) = -x2(0, i) * x1(1, i); // e
    L(j, 5) = -x2(0, i);            // f
  }
  // Solve Lx=B
  const VecXd h_vec = L.fullPivLu().solve(b);
  Homography2DNormalizedParameterization<double>::To(h_vec, h_mat);
  return (L * h_vec).isApprox(b, expected_precision);
}

// Cost functor which computes symmetric geometric distance
// used for homography matrix refinement.
//
template <typename T>
bool HomographySymmetricGeometricCostFunctor::
operator()(const T *homography_parameters, T *residuals) const {
  typedef Eigen::Matrix<T, 3, 3> Mat3;
  typedef Eigen::Matrix<T, 2, 1> Vec2;
  Mat3 H(homography_parameters);
  Vec2 x(T(x_(0)), T(x_(1)));
  Vec2 y(T(y_(0)), T(y_(1)));
  SymmetricGeometricDistanceTerms<T>(H, x, y, &residuals[0], &residuals[2]);
  return true;
}

ceres::CallbackReturnType TerminationCheckingCallback::
operator()(const ceres::IterationSummary &summary) {
  // If the step wasn't successful, there's nothing to do.
  if (!summary.step_is_successful) {
    return ceres::SOLVER_CONTINUE;
  }
  // Calculate average of symmetric geometric distance.
  double average_distance = 0.0;
  for (int i = 0; i < x1_.cols(); i++) {
    average_distance += SymmetricGeometricDistance(*H_, x1_.col(i), x2_.col(i));
  }
  average_distance /= x1_.cols();
  if (average_distance <= options_.expected_average_symmetric_distance) {
    return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
  }
  return ceres::SOLVER_CONTINUE;
}

inline bool EstimateHomography2DFromCorrespondences(
    const MatXd &x1, const MatXd &x2, const EstimateHomographyOptions &options,
    Mat3d *h_mat) {
  if (2 != x1.rows() || 4 > x1.cols() || x1.rows() != x2.rows() ||
      x1.cols() != x2.cols()) {
    // std::cout<< "Size not consistent!"<<std::endl;
    return false;
  }
  // Step 1: Algebraic homography estimation.
  // Assume algebraic estimation always succeeds.
  Homography2DFromCorrespondencesLinearEuc(x1, x2, h_mat,
                                           EigenDouble::dummy_precision());
  // std::cout<< "Estimated matrix after algebraic estimation:\n" <<
  // *h_mat<<std::endl;
  // Step 2: Refine matrix using Ceres minimizer.
  ceres::Problem problem;
  for (int i = 0; i < x1.cols(); i++) {
    HomographySymmetricGeometricCostFunctor
        *homography_symmetric_geometric_cost_function =
            new HomographySymmetricGeometricCostFunctor(x1.col(i), x2.col(i));
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<HomographySymmetricGeometricCostFunctor,
                                        4, // num_residuals
                                        9>(
            homography_symmetric_geometric_cost_function),
        NULL, h_mat->data());
  }
  // Configure the solve.
  ceres::Solver::Options solver_options;
  solver_options.linear_solver_type = ceres::DENSE_SCHUR; // DENSE_QR;
  solver_options.max_num_iterations = options.max_num_iterations;
  solver_options.update_state_every_iteration = true;
  solver_options.minimizer_progress_to_stdout = false;
  solver_options.logging_type = ceres::SILENT;
  // Terminate if the average symmetric distance is good enough.
  TerminationCheckingCallback callback(x1, x2, options, h_mat);
  solver_options.callbacks.push_back(&callback);
  // Run the solve.
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  // std::cout<< "Summary:\n" << summary.FullReport()<<std::endl;
  // std::cout<< "Final refined matrix:\n" << *h_mat<<std::endl;
  return summary.IsSolutionUsable();
}

bool CeresSolveHomography(const std::vector<Point2float> &src_quad,
                          const std::vector<Point2float> &dst_quad,
                          const double expected_symmetric_distance,
                          Mat3d *h_mat) {
  if (nullptr == h_mat) {
    // std::cout<< "Null pointer!"<<std::endl;
    return false;
  }

  MatXd x1(2, src_quad.size());
  MatXd x2(2, dst_quad.size());

  for (size_t i = 0; i < src_quad.size(); ++i) {
    auto src_point = src_quad.at(i);
    auto dst_point = dst_quad.at(i);
    x1(0, i) = static_cast<double>(src_point.x);
    x1(1, i) = static_cast<double>(src_point.y);
    x2(0, i) = static_cast<double>(dst_point.x);
    x2(1, i) = static_cast<double>(dst_point.y);
  }

  Mat3d estimated_matrix;
  EstimateHomographyOptions options;
  options.expected_average_symmetric_distance = expected_symmetric_distance;
  EstimateHomography2DFromCorrespondences(x1, x2, options, &estimated_matrix);
  // Normalize the matrix for easier comparison.
  estimated_matrix /= estimated_matrix(2, 2);
  // std::cout << "Estimated matrix:\n" << estimated_matrix << std::endl;
  *h_mat = estimated_matrix;
  return true;
}
