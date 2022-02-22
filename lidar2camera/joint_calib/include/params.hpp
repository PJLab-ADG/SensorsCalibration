#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>

struct Params {
  Eigen::Matrix3d camera_matrix;
  Eigen::VectorXd k;
  std::vector<Eigen::MatrixXd> vec_rt;
};

struct LidarParams {
  Eigen::Matrix3d camera_matrix;
  Eigen::VectorXd k;
  std::vector<Eigen::MatrixXd> vec_rt;
  Eigen::Matrix<double, 3, 4> extrinsic;
};

struct LidarPointPair {
  cv::Point3f lidar_3d_point[4];
  cv::Point2f lidar_2d_point[4];
  int img_index = 0;
};

struct EstimateHomographyOptions {
  // Default settings for homography estimation which should be suitable
  // for a wide range of use cases.
  EstimateHomographyOptions()
      : max_num_iterations(50), expected_average_symmetric_distance(1e-16) {}

  int max_num_iterations;
  double expected_average_symmetric_distance;
};

class Utils {

public:
  template <typename T>
  static void SymmetricGeometricDistanceTerms(const Eigen::Matrix<T, 3, 3> &H,
                                              const Eigen::Matrix<T, 2, 1> &x1,
                                              const Eigen::Matrix<T, 2, 1> &x2,
                                              T forward_error[2],
                                              T backward_error[2]) {
    typedef Eigen::Matrix<T, 3, 1> Vec3;
    Vec3 x(x1(0), x1(1), T(1.0));
    Vec3 y(x2(0), x2(1), T(1.0));

    Vec3 H_x = H * x;
    Vec3 Hinv_y = H.inverse() * y;

    H_x /= H_x(2);
    Hinv_y /= Hinv_y(2);

    forward_error[0] = H_x(0) - y(0);
    forward_error[1] = H_x(1) - y(1);
    backward_error[0] = Hinv_y(0) - x(0);
    backward_error[1] = Hinv_y(1) - x(1);
  }
  static double SymmetricGeometricDistance(const Eigen::Matrix3d &H,
                                           const Eigen::Vector2d &x1,
                                           const Eigen::Vector2d &x2) {
    Eigen::Vector2d forward_error, backward_error;
    SymmetricGeometricDistanceTerms<double>(H, x1, x2, forward_error.data(),
                                            backward_error.data());
    return forward_error.squaredNorm() + backward_error.squaredNorm();
  }
};

#endif // UTILS_HPP_
