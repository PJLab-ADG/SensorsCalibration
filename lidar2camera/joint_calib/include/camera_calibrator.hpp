#ifndef CAMERA_CALIBRATOR_HPP_
#define CAMERA_CALIBRATOR_HPP_

#include "nonlinear_optimizer.hpp"
#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include "logging.hpp"

class CameraCalibrator {
public:
  void set_input(const std::vector<std::string> images_name,
                 const std::vector<cv::Mat> &vec_mat_,
                 const cv::Size &chessboard_size_,
                 const std::vector<std::vector<std::string>> &lidar_3d_pts);

  void get_result(cv::Mat &camera_matrix, cv::Mat &k,
                  const cv::Size &image_size, std::vector<cv::Mat> &rvecsMat,
                  std::vector<cv::Mat> &tvecsMat);

private:
  void make_board_points(const cv::Size &chessboard_size_);

  void compute_mean_error(cv::Mat &img, cv::Mat &camera_matrix_, cv::Mat &k_,
                          std::vector<cv::Mat> &rvecsMat_,
                          std::vector<cv::Mat> &tvecsMat_);

  void refine_all(Eigen::Matrix3d &camera_matrix_, Eigen::VectorXd &k_,
                  std::vector<Eigen::MatrixXd> &vec_extrinsics_);
  void refine_lidar2camera(Eigen::Matrix3d &camera_matrix_, Eigen::VectorXd &k_,
                           std::vector<Eigen::MatrixXd> &vec_extrinsics_,
                           Eigen::Matrix<double, 3, 4> &initial_extrinsic,
                           std::vector<LidarPointPair> &lidar_point_pairs);

  void get_distortion(const Eigen::Matrix3d &camera_matrix_,
                      const std::vector<Eigen::MatrixXd> &vec_extrinsics_,
                      Eigen::VectorXd &k_);

  void get_extrinsics(const std::vector<Eigen::Matrix3d> &vec_h_,
                      const Eigen::Matrix3d &camera_matrix_,
                      std::vector<Eigen::MatrixXd> &vec_extrinsics_);

  //
  void create_v(const Eigen::Matrix3d &h_, const int p, const int q,
                Eigen::RowVectorXd &row_v_);

  void get_camera_instrinsics(const std::vector<Eigen::Matrix3d> &vec_h_,
                              Eigen::Matrix3d &camera_matrix_);

  void get_homography(std::vector<Eigen::Matrix3d> &vec_h_);

  // normalized DLT algorithm
  void estimate_H(const std::vector<cv::Point2f> &img_pts_,
                  const std::vector<cv::Point2f> &board_pts_,
                  Eigen::Matrix3d &matrix_H_);

  void get_normalization_matrix(const std::vector<cv::Point2f> &pts_,
                                Eigen::Matrix3d &matrix_trans_);

  void
  point_undistort(const std::vector<std::vector<cv::Point2f>> &board_imgs_pts,
                  std::vector<std::vector<cv::Point2f>> &undistort_pts,
                  const Eigen::Matrix3d camera_intrinsic,
                  const Eigen::VectorXd distort);
  void image_undistort(const cv::Mat &img, cv::Mat &undistort_img,
                       const Eigen::Matrix3d camera_intrinsic,
                       const Eigen::VectorXd distort);

  void lidar_projection(const Eigen::Matrix3d &camera_intrinsic,
                        const Eigen::Matrix<double, 3, 4> &extrinsic,
                        const cv::Point3f &pt, cv::Point2f &img_pt);
  void DrawCross(cv::Mat &img, cv::Point point);

private:
  bool _b_disp_corners = false;
  std::vector<std::vector<cv::Point2f>> _imgs_pts;
  std::vector<std::vector<cv::Point2f>> _boards_pts;
  std::vector<std::vector<cv::Point3f>> _boards_pts_3d;
  std::vector<std::vector<cv::Point3f>> _boards_pts_cir; // scl
  std::vector<std::vector<cv::Point3f>> lidar_3d_pts_;
  std::vector<std::vector<cv::Point2f>> _imgs_pts_cir2D_true; // scl
  std::vector<cv::Mat> available_imgs;

  NonlinearOptimizer optimier;
};

#endif // CAMERA_CALIBRATOR_HPP_
