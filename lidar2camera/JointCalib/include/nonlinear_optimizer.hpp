#ifndef NONLINEAR_OPTIMIZER_HPP_
#define NONLINEAR_OPTIMIZER_HPP_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>

#include "params.hpp"
#include "reprojection_error.hpp"
#include "termination_checking.hpp"

class NonlinearOptimizer {
public:
  NonlinearOptimizer() {}
  ~NonlinearOptimizer() {}

  void refine_H(const std::vector<cv::Point2f> &img_pts_,
                const std::vector<cv::Point2f> &board_pts_,
                const Eigen::Matrix3d &matrix_H_, Eigen::Matrix3d &refined_H_);

  void refine_all_camera_params(
      const Params &params_,
      const std::vector<std::vector<cv::Point2f>> &imgs_pts_,
      const std::vector<std::vector<cv::Point2f>> &bords_pts_,
      Params &refined_params_);

  void refine_lidar2camera_params(
      const LidarParams &params_,
      const std::vector<std::vector<cv::Point2f>> &imgs_pts_,
      const std::vector<std::vector<cv::Point2f>> &bords_pts_,
      const std::vector<LidarPointPair> lidar_point_pairs,
      LidarParams &refined_params_);

private:
  void formate_data(const Eigen::VectorXd &v_camera_matrix_,
                    const Eigen::VectorXd &v_dist_,
                    const std::vector<Eigen::VectorXd> &v_rt_, Params &params_);
  void formate_data(const Eigen::VectorXd &v_camera_matrix_,
                    const Eigen::VectorXd &v_dist_,
                    const std::vector<Eigen::VectorXd> &v_rt_,
                    const Eigen::VectorXd &RT, LidarParams &params_);
  // Cost functor which computes symmetric geometric distance
  // used for homography matrix refinement.
  struct HomographySymmetricGeometricCostFunctor {
    HomographySymmetricGeometricCostFunctor(const Eigen::Vector2d &x,
                                            const Eigen::Vector2d &y)
        : x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T *homography_parameters, T *residuals) const {
      typedef Eigen::Matrix<T, 3, 3> Mat3;
      typedef Eigen::Matrix<T, 2, 1> Vec2;

      Mat3 H(homography_parameters);
      Vec2 x(T(x_(0)), T(x_(1)));
      Vec2 y(T(y_(0)), T(y_(1)));

      Utils::SymmetricGeometricDistanceTerms<T>(H, x, y, &residuals[0],
                                                &residuals[2]);
      return true;
    }

    const Eigen::Vector2d x_;
    const Eigen::Vector2d y_;
  };

  struct ReprojectionError {
    ReprojectionError(const Eigen::Vector2d &img_pts_,
                      const Eigen::Vector2d &board_pts_)
        : _img_pts(img_pts_), _board_pts(board_pts_) {}

    template <typename T>
    bool operator()(const T *const instrinsics_, const T *const k_,
                    const T *const rt_, // 6 : angle axis and translation
                    T *residuls) const {
      //	Eigen::Vector3d hom_w(_board_pts(0), _board_pts(1), T(1.));
      T hom_w_t[3];
      hom_w_t[0] = T(_board_pts(0));
      hom_w_t[1] = T(_board_pts(1));
      hom_w_t[2] = T(1.);
      T hom_w_trans[3];
      ceres::AngleAxisRotatePoint(rt_, hom_w_t, hom_w_trans);
      hom_w_trans[0] += rt_[3];
      hom_w_trans[1] += rt_[4];
      hom_w_trans[2] += rt_[5];

      T c_x = hom_w_trans[0] / hom_w_trans[2];
      T c_y = hom_w_trans[1] / hom_w_trans[2];

      // distortion
      T r2 = c_x * c_x + c_y * c_y;
      T r4 = r2 * r2;
      T r_coeff = (T(1) + k_[0] * r2 + k_[1] * r4);
      T xd = c_x * r_coeff;
      T yd = c_y * r_coeff;

      // camera coord => image coord
      T predict_x = instrinsics_[0] * xd + instrinsics_[2];
      T predict_y = instrinsics_[3] * yd + instrinsics_[4];

      // residus

      residuls[0] = _img_pts(0) - predict_x;
      residuls[1] = _img_pts(1) - predict_y;

      return true;
    }
    const Eigen::Vector2d _img_pts;
    const Eigen::Vector2d _board_pts;
  };

  struct LidarReprojectionError {
    LidarReprojectionError(const Eigen::Vector2d &img_pts_,
                           const Eigen::Vector3d &lidar_pts_)
        : _img_pts(img_pts_), _lidar_pts(lidar_pts_) {}

    template <typename T>
    bool operator()(const T *const instrinsics_, const T *const k_,
                    const T *const rt_, // 6 : angle axis and translation
                    T *residuls) const {
      T hom_w_t[3];
      hom_w_t[0] = T(_lidar_pts(0));
      hom_w_t[1] = T(_lidar_pts(1));
      hom_w_t[2] = T(_lidar_pts(2));
      T hom_w_trans[3];
      ceres::AngleAxisRotatePoint(rt_, hom_w_t, hom_w_trans);
      hom_w_trans[0] += rt_[3];
      hom_w_trans[1] += rt_[4];
      hom_w_trans[2] += rt_[5];

      T c_x = hom_w_trans[0] / hom_w_trans[2];
      T c_y = hom_w_trans[1] / hom_w_trans[2];

      T xd = c_x;
      T yd = c_y;
      // distortion
      // T r2 = c_x * c_x + c_y * c_y;
      // T r4 = r2 * r2;
      // T r_coeff = (T(1) + k_[0] * r2 + k_[1] * r4);
      // T xd = c_x * r_coeff;
      // T yd = c_y * r_coeff;

      // // camera coord => image coord
      T predict_x = instrinsics_[0] * xd + instrinsics_[2];
      T predict_y = instrinsics_[3] * yd + instrinsics_[4];

      T scale_factor = (T)60.0;
      residuls[0] = (_img_pts(0) - predict_x) * scale_factor;
      residuls[1] = (_img_pts(1) - predict_y) * scale_factor;

      return true;
    }
    const Eigen::Vector2d _img_pts;
    const Eigen::Vector3d _lidar_pts;
  };
};

#endif // NONLINEAR_OPTIMIZER_HPP_
