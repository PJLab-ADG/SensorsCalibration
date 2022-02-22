/*
 * Copyright (C) 2022 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include "camera_calibrator.hpp"
#include <iomanip>

void CameraCalibrator::set_input(
    const std::vector<std::string> images_name,
    const std::vector<cv::Mat> &vec_mat_, const cv::Size &chessboard_size_,
    const std::vector<std::vector<std::string>> &lidar_3d_pts) {
  // lidar
  std::vector<std::vector<cv::Point3f>> pts;
  for (auto src : lidar_3d_pts) {
    std::vector<cv::Point3f> pt;
    cv::Point3f left_top(std::stod(src[0]), std::stod(src[1]),
                         std::stod(src[2]));
    cv::Point3f right_top(std::stod(src[3]), std::stod(src[4]),
                          std::stod(src[5]));
    cv::Point3f left_bottom(std::stod(src[6]), std::stod(src[7]),
                            std::stod(src[8]));
    cv::Point3f right_bottom(std::stod(src[9]), std::stod(src[10]),
                             std::stod(src[11]));
    pt.push_back(left_top);
    pt.push_back(right_top);
    pt.push_back(left_bottom);
    pt.push_back(right_bottom);
    pts.push_back(pt);
  }

  std::cout << "start" << std::endl;
  _boards_pts.clear();
  _imgs_pts.clear();
  int i = 0;
  for (const auto &img : vec_mat_) {
    CHECK(1 == img.channels()) << "images must be gray";
    std::vector<cv::Point2f> corner_pts;
    int found = cv::findChessboardCorners(img, chessboard_size_, corner_pts,
                                          cv::CALIB_CB_ADAPTIVE_THRESH |
                                              cv::CALIB_CB_FAST_CHECK |
                                              cv::CALIB_CB_NORMALIZE_IMAGE);
    if (!found) {
      continue;
    }
    cv::Mat img_copy = img.clone();
    available_imgs.push_back(img_copy);
    lidar_3d_pts_.push_back(pts[i]);
    std::cout << images_name[i] << std::endl;
    i++;
    cv::TermCriteria criteria(2, 30, 0.001);
    cv::cornerSubPix(img, corner_pts, chessboard_size_, cv::Size(-1, -1),
                     criteria);
    _imgs_pts.push_back(corner_pts);
    this->make_board_points(chessboard_size_);
  }
}

void CameraCalibrator::get_result(cv::Mat &camera_matrix, cv::Mat &k,
                                  const cv::Size &image_size,
                                  std::vector<cv::Mat> &rvecsMat,
                                  std::vector<cv::Mat> &tvecsMat) {
  double re_error =
      cv::calibrateCamera(_boards_pts_3d, _imgs_pts, image_size, camera_matrix,
                          k, rvecsMat, tvecsMat, CV_CALIB_FIX_PRINCIPAL_POINT);
  std::cout << "reprojection is " << re_error << std::endl;

  Eigen::Matrix3d camera_intrinsic;
  camera_intrinsic << camera_matrix.at<double>(0, 0),
      camera_matrix.at<double>(1, 0), camera_matrix.at<double>(2, 0),
      camera_matrix.at<double>(0, 1), camera_matrix.at<double>(1, 1),
      camera_matrix.at<double>(2, 1), camera_matrix.at<double>(0, 2),
      camera_matrix.at<double>(1, 2), camera_matrix.at<double>(2, 2);

  Eigen::VectorXd distort(2);
  distort << k.at<double>(0, 0), k.at<double>(0, 1);

  std::cout << camera_intrinsic << std::endl;
  std::cout << distort << std::endl;

  std::vector<Eigen::MatrixXd> vec_extrinsics;
  for (size_t i = 0; i < rvecsMat.size(); i++) {
    cv::Mat rvec = rvecsMat[i];
    cv::Mat tvec = tvecsMat[i];
    cv::Mat rot;
    cv::Rodrigues(rvec, rot);
    std::vector<cv::Point2f> imgpoints_cir;
    cv::projectPoints(_boards_pts_cir[i], rvecsMat[i], tvecsMat[i],
                      camera_matrix, k, imgpoints_cir);
    size_t y_min = imgpoints_cir[0].y;
    size_t y_max = imgpoints_cir[0].y;
    size_t x_min = imgpoints_cir[0].x;
    size_t x_max = imgpoints_cir[0].x;
    for (size_t i = 1; i < 4; i++) {
      if (imgpoints_cir[i].y > y_max)
        y_max = imgpoints_cir[i].y;
      if (imgpoints_cir[i].y < y_min)
        y_min = imgpoints_cir[i].y;
      if (imgpoints_cir[i].x > x_max)
        x_max = imgpoints_cir[i].x;
      if (imgpoints_cir[i].x < x_min)
        x_min = imgpoints_cir[i].x;
    }
    std::vector<cv::Point2f> imgpoints_cir1; // scl
    for (size_t i = 0; i < 4; i++) {
      if ((imgpoints_cir[i].x - x_min) <= 50 &&
          (imgpoints_cir[i].y - y_min) <= 50)
        imgpoints_cir1.push_back(imgpoints_cir[i]);
    }
    for (size_t i = 0; i < 4; i++) {
      if ((x_max - imgpoints_cir[i].x) <= 50 &&
          (imgpoints_cir[i].y - y_min) <= 50)
        imgpoints_cir1.push_back(imgpoints_cir[i]);
    }
    for (size_t i = 0; i < 4; i++) {
      if ((imgpoints_cir[i].x - x_min) <= 50 &&
          (y_max - imgpoints_cir[i].y) <= 50)
        imgpoints_cir1.push_back(imgpoints_cir[i]);
    }
    for (size_t i = 0; i < 4; i++) {
      if ((x_max - imgpoints_cir[i].x) <= 50 &&
          (y_max - imgpoints_cir[i].y) <= 50)
        imgpoints_cir1.push_back(imgpoints_cir[i]);
    }
    if (imgpoints_cir1.size() != 4) {
      std::cout << "imgpoints_cir1.size() must = 4" << std::endl;
      return;
    }
    _imgs_pts_cir2D_true.push_back(imgpoints_cir1);
    Eigen::Vector3d r0, r1, r2, t;
    r0 << rot.at<double>(0, 0), rot.at<double>(1, 0), rot.at<double>(2, 0);
    r1 << rot.at<double>(0, 1), rot.at<double>(1, 1), rot.at<double>(2, 1);
    r2 << rot.at<double>(0, 2), rot.at<double>(1, 2), rot.at<double>(2, 2);
    t << tvec.at<double>(0, 0), tvec.at<double>(0, 1), tvec.at<double>(0, 2);

    Eigen::MatrixXd RT(3, 4);
    RT.block<3, 1>(0, 0) = r0;
    RT.block<3, 1>(0, 1) = r1;
    RT.block<3, 1>(0, 2) = r2;
    RT.block<3, 1>(0, 3) = t;
    vec_extrinsics.push_back(RT);
  }

  this->refine_all(camera_intrinsic, distort, vec_extrinsics);
  std::vector<LidarPointPair> lidar_point_pairs;
  for (size_t i = 0; i < _imgs_pts_cir2D_true.size(); i++) {
    cv::Mat img = available_imgs[i].clone();
    cv::Mat undistort_img = img;
    std::vector<cv::Point2f> left_top_src, right_top_src, left_bottom_src,
        right_bottom_src;

    LidarPointPair lidar_point_pair;
    lidar_point_pair.img_index = i;
    // left top
    lidar_point_pair.lidar_2d_point[0] = _imgs_pts_cir2D_true[i][0];
    lidar_point_pair.lidar_3d_point[0] = lidar_3d_pts_[i][0];
    // right top
    lidar_point_pair.lidar_2d_point[1] = _imgs_pts_cir2D_true[i][1];
    lidar_point_pair.lidar_3d_point[1] = lidar_3d_pts_[i][1];

    // left bottom;
    lidar_point_pair.lidar_2d_point[2] = _imgs_pts_cir2D_true[i][2];
    lidar_point_pair.lidar_3d_point[2] = lidar_3d_pts_[i][3];

    // right bottom
    lidar_point_pair.lidar_2d_point[3] = _imgs_pts_cir2D_true[i][3];
    lidar_point_pair.lidar_3d_point[3] = lidar_3d_pts_[i][2];
    if (false) { // Show real LiDAR pixels
      DrawCross(undistort_img, lidar_point_pair.lidar_2d_point[0]);
      DrawCross(undistort_img, lidar_point_pair.lidar_2d_point[1]);
      DrawCross(undistort_img, lidar_point_pair.lidar_2d_point[2]);
      DrawCross(undistort_img, lidar_point_pair.lidar_2d_point[3]);
      std::string save_name = "original" + std::to_string(i) + ".png";
      cv::imwrite(save_name, undistort_img);
    }

    if (i > 20) // no corresponding circle center for more than 20
    {
      continue;
    }

    lidar_point_pairs.push_back(lidar_point_pair);
  }

  // an inaccurate initial Lidar-camera extrinsic
  Eigen::Matrix<double, 3, 4> initial_extrinsic;
  initial_extrinsic << -0.0000667338, -0.9999999780, 0.0001990654,
      -0.0010031200, -0.0000409491, 0.0001990681, 0.9999999793, 0.5912607639,
      -0.9999999969, 0.0000667257, -0.0000409624, 2.5079706347;
  std::cout << initial_extrinsic << std::endl;

  this->refine_lidar2camera(camera_intrinsic, distort, vec_extrinsics,
                            initial_extrinsic, lidar_point_pairs);

  double lidar_reprojection_error = 0;
  int number = 0;
  if (true) // Show the optimized projection effect
  {
    for (size_t i = 0; i < lidar_point_pairs.size(); i++) {
      int img_index = lidar_point_pairs[i].img_index;
      cv::Mat img = available_imgs[img_index].clone();
      cv::Mat undistort_img = img;
      // image_undistort(img, undistort_img, camera_intrinsic, distort);
      cv::Point2f img_pt;
      lidar_projection(camera_intrinsic, initial_extrinsic,
                       lidar_point_pairs[i].lidar_3d_point[0], img_pt);
      cv::circle(undistort_img, img_pt, 8, (0, 255, 0), 8);

      cv::Point2f pt = lidar_point_pairs[i].lidar_2d_point[0];
      double error1 = std::sqrt((img_pt.x - pt.x) * (img_pt.x - pt.x) +
                                (img_pt.y - pt.y) * (img_pt.y - pt.y));

      lidar_projection(camera_intrinsic, initial_extrinsic,
                       lidar_point_pairs[i].lidar_3d_point[1], img_pt);
      cv::circle(undistort_img, img_pt, 8, (0, 255, 0), 8);

      pt = lidar_point_pairs[i].lidar_2d_point[1];
      double error2 = std::sqrt((img_pt.x - pt.x) * (img_pt.x - pt.x) +
                                (img_pt.y - pt.y) * (img_pt.y - pt.y));

      lidar_projection(camera_intrinsic, initial_extrinsic,
                       lidar_point_pairs[i].lidar_3d_point[2], img_pt);
      cv::circle(undistort_img, img_pt, 8, (0, 255, 0), 8);

      pt = lidar_point_pairs[i].lidar_2d_point[2];
      double error3 = std::sqrt((img_pt.x - pt.x) * (img_pt.x - pt.x) +
                                (img_pt.y - pt.y) * (img_pt.y - pt.y));

      lidar_projection(camera_intrinsic, initial_extrinsic,
                       lidar_point_pairs[i].lidar_3d_point[3], img_pt);
      cv::circle(undistort_img, img_pt, 8, (0, 255, 0), 8);

      pt = lidar_point_pairs[i].lidar_2d_point[3];
      double error4 = std::sqrt((img_pt.x - pt.x) * (img_pt.x - pt.x) +
                                (img_pt.y - pt.y) * (img_pt.y - pt.y));
      //
      DrawCross(undistort_img, lidar_point_pairs[i].lidar_2d_point[0]);
      DrawCross(undistort_img, lidar_point_pairs[i].lidar_2d_point[1]);
      DrawCross(undistort_img, lidar_point_pairs[i].lidar_2d_point[2]);
      DrawCross(undistort_img, lidar_point_pairs[i].lidar_2d_point[3]);
      std::string save_name = "refine" + std::to_string(i) + ".png";
      cv::imwrite(save_name, undistort_img);
      double error = (error1 + error2 + error3 + error4) / 4;
      lidar_reprojection_error += error;
      number++;
    }
  }
  std::cout << "lidar reprojection error: " << lidar_reprojection_error / number
            << std::endl;
}

void CameraCalibrator::DrawCross(cv::Mat &img, cv::Point point) {
  double cx = point.x;
  double cy = point.y;
  double len = 10;
  cv::line(img, cv::Point(cx - len, cy), cv::Point(cx + len, cy), (0, 255, 255),
           3);
  cv::line(img, cv::Point(cx, cy - len), cv::Point(cx, cy + len), (0, 255, 255),
           3);
}

void CameraCalibrator::lidar_projection(
    const Eigen::Matrix3d &camera_intrinsic,
    const Eigen::Matrix<double, 3, 4> &extrinsic, const cv::Point3f &pt,
    cv::Point2f &img_pt) {
  Eigen::Matrix<double, 4, 1> lidar_point;
  lidar_point << pt.x, pt.y, pt.z, 1.0;
  // Eigen::Matrix<float, 3, 1> pro_pt;
  auto pro_pt = camera_intrinsic * extrinsic * lidar_point;
  img_pt.x = pro_pt(0) / pro_pt(2);
  img_pt.y = pro_pt(1) / pro_pt(2);
}

void CameraCalibrator::point_undistort(
    const std::vector<std::vector<cv::Point2f>> &board_imgs_pts,
    std::vector<std::vector<cv::Point2f>> &undistort_pts,
    const Eigen::Matrix3d camera_intrinsic, const Eigen::VectorXd distort) {
  cv::Mat K, D;
  float d[4], k[9];
  k[0] = camera_intrinsic(0, 0);
  k[1] = camera_intrinsic(0, 1);
  k[2] = camera_intrinsic(0, 2);
  k[3] = camera_intrinsic(1, 0);
  k[4] = camera_intrinsic(1, 1);
  k[5] = camera_intrinsic(1, 2);
  k[6] = camera_intrinsic(2, 0);
  k[7] = camera_intrinsic(2, 1);
  k[8] = camera_intrinsic(2, 2);
  d[0] = distort(0);
  d[1] = distort(1);
  d[2] = 0;
  d[3] = 0;
  D = cv::Mat(4, 1, CV_32FC1, d);
  K = cv::Mat(3, 3, CV_32FC1, k);
  double fx = camera_intrinsic(0, 0);
  double fy = camera_intrinsic(1, 1);
  double cx = camera_intrinsic(0, 2);
  double cy = camera_intrinsic(1, 2);
  for (size_t i = 0; i < board_imgs_pts.size(); i++) {
    std::vector<cv::Point2f> src = board_imgs_pts[i];
    std::vector<cv::Point2f> dst;
    cv::undistortPoints(src, dst, K, D);
    for (size_t i = 0; i < dst.size(); i++) {
      dst[i].x = dst[i].x * fx + cx;
      dst[i].y = dst[i].y * fy + cy;
    }

    undistort_pts.push_back(dst);
  }
}

void CameraCalibrator::image_undistort(const cv::Mat &img,
                                       cv::Mat &undistort_img,
                                       const Eigen::Matrix3d camera_intrinsic,
                                       const Eigen::VectorXd distort) {
  cv::Mat K, D;
  float d[4], k[9];
  k[0] = camera_intrinsic(0, 0);
  k[1] = camera_intrinsic(0, 1);
  k[2] = camera_intrinsic(0, 2);
  k[3] = camera_intrinsic(1, 0);
  k[4] = camera_intrinsic(1, 1);
  k[5] = camera_intrinsic(1, 2);
  k[6] = camera_intrinsic(2, 0);
  k[7] = camera_intrinsic(2, 1);
  k[8] = camera_intrinsic(2, 2);
  d[0] = distort(0);
  d[1] = distort(1);
  d[2] = 0;
  d[3] = 0;
  D = cv::Mat(4, 1, CV_32FC1, d);
  K = cv::Mat(3, 3, CV_32FC1, k);

  cv::Mat I = cv::Mat::eye(3, 3, CV_32FC1);
  cv::Mat mapX, mapY;
  cv::Mat outImg = cv::Mat(img.size(), CV_32FC3);
  cv::initUndistortRectifyMap(K, D, I, K, img.size(), CV_32FC1, mapX, mapY);
  cv::remap(img, outImg, mapX, mapY, cv::INTER_LINEAR);
  undistort_img = outImg;
}

void CameraCalibrator::make_board_points(const cv::Size &chessboard_size_) {
  std::vector<cv::Point2f> vec_points;
  std::vector<cv::Point3f> vec_points_3d;
  for (int r = 0; r < chessboard_size_.height; ++r) {
    for (int c = 0; c < chessboard_size_.width; ++c) {
      vec_points.emplace_back(c, r);
      vec_points_3d.emplace_back(c, r, 0);
    }
  }
  _boards_pts.push_back(vec_points);
  _boards_pts_3d.push_back(vec_points_3d);

  std::vector<cv::Point3f> vec_points_cir;
  vec_points_cir.emplace_back(1.82f, -3.12f, 0.0f);  // sim
  vec_points_cir.emplace_back(14.18f, -3.12f, 0.0f); // sim
  vec_points_cir.emplace_back(1.82f, 9.12f, 0.0f);   // sim
  vec_points_cir.emplace_back(14.18f, 9.12f, 0.0f);  // sim

  _boards_pts_cir.push_back(vec_points_cir); // scl
}

void CameraCalibrator::refine_all(
    Eigen::Matrix3d &camera_matrix_, Eigen::VectorXd &k_,
    std::vector<Eigen::MatrixXd> &vec_extrinsics_) {

  Params params, params_refined;
  params.camera_matrix = camera_matrix_;
  params.k = k_;
  params.vec_rt = vec_extrinsics_;
  LOGI("this is here");
  optimier.refine_all_camera_params(params, _imgs_pts, _boards_pts,
                                    params_refined);
  camera_matrix_ = params_refined.camera_matrix;
  k_ = params_refined.k;
  vec_extrinsics_ = params_refined.vec_rt;
}

void CameraCalibrator::refine_lidar2camera(
    Eigen::Matrix3d &camera_matrix_, Eigen::VectorXd &k_,
    std::vector<Eigen::MatrixXd> &vec_extrinsics_,
    Eigen::Matrix<double, 3, 4> &initial_extrinsic,
    std::vector<LidarPointPair> &lidar_point_pairs) {
  LidarParams params, params_refined;
  params.camera_matrix = camera_matrix_;
  params.k = k_;
  params.vec_rt = vec_extrinsics_;
  params.extrinsic = initial_extrinsic;

  optimier.refine_lidar2camera_params(params, _imgs_pts, _boards_pts,
                                      lidar_point_pairs, params_refined);

  //
  camera_matrix_ = params_refined.camera_matrix;
  k_ = params_refined.k;
  vec_extrinsics_ = params_refined.vec_rt;
  initial_extrinsic = params_refined.extrinsic;
}

void CameraCalibrator::get_distortion(
    const Eigen::Matrix3d &camera_matrix_,
    const std::vector<Eigen::MatrixXd> &vec_extrinsics_, Eigen::VectorXd &k_) {
  Eigen::MatrixXd D;
  Eigen::VectorXd d;
  double uc = camera_matrix_(0, 2);
  double vc = camera_matrix_(1, 2);
  for (int i = 0; i < _imgs_pts.size(); ++i) {
    for (int j = 0; j < _imgs_pts[i].size(); ++j) {
      Eigen::Vector4d houm_coor(_boards_pts[i][j].x, _boards_pts[i][j].y, 0, 1);
      Eigen::Vector3d uv = camera_matrix_ * vec_extrinsics_[i] * houm_coor;
      Eigen::Vector2d uv_estim(uv(0) / uv(2), uv(1) / uv(2));

      Eigen::Vector3d coor_norm = vec_extrinsics_[i] * houm_coor;
      coor_norm /= coor_norm(2);
      Eigen::Vector2d v_r(coor_norm(0), coor_norm(1));
      double r = v_r.norm();

      Eigen::RowVector2d vu((uv_estim(0) - uc) * r * r,
                            (uv_estim(0) - uc) * r * r * r * r);
      D.conservativeResize(D.rows() + 1, 2);
      D.row(D.rows() - 1) = vu;
      Eigen::RowVector2d vv((uv_estim(1) - vc) * r * r,
                            (uv_estim(1) - vc) * r * r * r * r);
      D.conservativeResize(D.rows() + 1, 2);
      D.row(D.rows() - 1) = vv;

      d.conservativeResize(d.size() + 1);
      d(d.size() - 1) = _imgs_pts[i][j].x - uv_estim(0);
      d.conservativeResize(d.size() + 1);
      d(d.size() - 1) = _imgs_pts[i][j].y - uv_estim(1);
    }
  }
  Eigen::MatrixXd DTD = D.transpose() * D;
  Eigen::MatrixXd temp = (DTD.inverse()) * D.transpose();
  k_ = temp * d;
}

void CameraCalibrator::get_extrinsics(
    const std::vector<Eigen::Matrix3d> &vec_h_,
    const Eigen::Matrix3d &camera_matrix_,
    std::vector<Eigen::MatrixXd> &vec_extrinsics_) {
  vec_extrinsics_.clear();
  Eigen::Matrix3d inv_camera_matrix = camera_matrix_.inverse();
  for (int i = 0; i < vec_h_.size(); ++i) {
    Eigen::Vector3d s = inv_camera_matrix * vec_h_[i].col(0);
    double scalar_factor = 1 / s.norm();

    Eigen::Vector3d r0 = scalar_factor * inv_camera_matrix * vec_h_[i].col(0);
    Eigen::Vector3d r1 = scalar_factor * inv_camera_matrix * vec_h_[i].col(1);
    Eigen::Vector3d t = scalar_factor * inv_camera_matrix * vec_h_[i].col(2);
    Eigen::Vector3d r2 = r0.cross(r1);

    Eigen::MatrixXd RT(3, 4);
    RT.block<3, 1>(0, 0) = r0;
    RT.block<3, 1>(0, 1) = r1;
    RT.block<3, 1>(0, 2) = r2;
    RT.block<3, 1>(0, 3) = t;
    vec_extrinsics_.push_back(RT);
  }
}

void CameraCalibrator::create_v(const Eigen::Matrix3d &h_, const int p,
                                const int q, Eigen::RowVectorXd &row_v_) {
  row_v_ << h_(0, p) * h_(0, q), h_(0, p) * h_(1, q) + h_(1, p) * h_(0, q),
      h_(1, p) * h_(1, q), h_(2, p) * h_(0, q) + h_(0, p) * h_(2, q),
      h_(2, p) * h_(1, q) + h_(1, p) * h_(2, q), h_(2, p) * h_(2, q);
}

void CameraCalibrator::get_camera_instrinsics(
    const std::vector<Eigen::Matrix3d> &vec_h_,
    Eigen::Matrix3d &camera_matrix_) {
  int N = vec_h_.size();
  Eigen::MatrixXd V(2 * N, 6);
  V.setZero();

  for (int n = 0; n < N; ++n) {
    Eigen::RowVectorXd v01(6), v00(6), v11(6);
    create_v(vec_h_[n], 0, 1, v01);
    V.row(2 * n) = v01;
    create_v(vec_h_[n], 0, 0, v00);
    create_v(vec_h_[n], 1, 1, v11);
    V.row(2 * n + 1) = v00 - v11;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(V, Eigen::ComputeFullV);
  Eigen::VectorXd b = svd.matrixV().col(5);
  //
  double w = b[0] * b[2] * b[5] - b[1] * b[1] * b[5] - b[0] * b[4] * b[4] +
             2 * b[1] * b[3] * b[4] - b[2] * b[3] * b[3];
  double d = b[0] * b[2] - b[1] * b[1];

  double alpha = std::sqrt(w / (d * b[0]));
  double beta = std::sqrt(w / (d * d) * b[0]);
  double gamma = std::sqrt(w / (d * d * b[0])) * b[1];
  double uc = (b[1] * b[4] - b[2] * b[3]) / d;
  double vc = (b[1] * b[3] - b[0] * b[4]) / d;

  camera_matrix_ << alpha, gamma, uc, 0, beta, vc, 0, 0, 1;
}

void CameraCalibrator::get_homography(std::vector<Eigen::Matrix3d> &vec_h_) {
  vec_h_.clear();
  for (int i = 0; i < _imgs_pts.size(); ++i) {
    Eigen::Matrix3d ini_H, refined_H;
    this->estimate_H(_imgs_pts[i], _boards_pts[i], ini_H);
    optimier.refine_H(_imgs_pts[i], _boards_pts[i], ini_H, refined_H);
    vec_h_.push_back(refined_H);
  }
}

void CameraCalibrator::estimate_H(const std::vector<cv::Point2f> &img_pts_,
                                  const std::vector<cv::Point2f> &board_pts_,
                                  Eigen::Matrix3d &matrix_H_) {
  Eigen::Matrix3d matrix_normalize_img_pts;
  Eigen::Matrix3d matrix_normalize_board_pts;
  int N = img_pts_.size();
  this->get_normalization_matrix(img_pts_, matrix_normalize_img_pts);
  this->get_normalization_matrix(board_pts_, matrix_normalize_board_pts);
  Eigen::MatrixXd M(2 * N, 9);
  M.setZero();

  for (int i = 0; i < N; ++i) {
    Eigen::Vector3d norm_img_p =
        matrix_normalize_img_pts *
        Eigen::Vector3d(img_pts_[i].x, img_pts_[i].y, 1);
    Eigen::Vector3d norm_board_p =
        matrix_normalize_board_pts *
        Eigen::Vector3d(board_pts_[i].x, board_pts_[i].y, 1);
    // M
    M(2 * i, 0) = -norm_board_p(0);
    M(2 * i, 1) = -norm_board_p(1);
    M(2 * i, 2) = -1;
    M(2 * i, 6) = norm_img_p(0) * norm_board_p(0);
    M(2 * i, 7) = norm_img_p(0) * norm_board_p(1);
    M(2 * i, 8) = norm_img_p(0);

    M(2 * i + 1, 3) = -norm_board_p(0);
    M(2 * i + 1, 4) = -norm_board_p(1);
    M(2 * i + 1, 5) = -1;
    M(2 * i + 1, 6) = norm_img_p(1) * norm_board_p(0);
    M(2 * i + 1, 7) = norm_img_p(1) * norm_board_p(1);
    M(2 * i + 1, 8) = norm_img_p(1);
  }
  // svd solve M*h=0
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
  Eigen::VectorXd V = svd.matrixV().col(8);
  matrix_H_ << V(0), V(1), V(2), V(3), V(4), V(5), V(6), V(7), V(8);
  matrix_H_ = matrix_normalize_img_pts.inverse() * matrix_H_ *
              matrix_normalize_board_pts;
  matrix_H_ /= matrix_H_(2, 2);
}

void CameraCalibrator::get_normalization_matrix(
    const std::vector<cv::Point2f> &pts_, Eigen::Matrix3d &matrix_trans_) {
  double sum_x = 0, sum_y = 0;
  std::for_each(std::begin(pts_), std::end(pts_), [&](const cv::Point2f &p) {
    sum_x += p.x;
    sum_y += p.y;
  });
  double mean_x = sum_x / pts_.size();
  double mean_y = sum_y / pts_.size();

  double accmx = 0, accmy = 0;
  std::for_each(std::begin(pts_), std::end(pts_), [&](const cv::Point2f &p) {
    accmx += (p.x - mean_x) * (p.x - mean_x);
    accmy += (p.y - mean_y) * (p.y - mean_y);
  });
  double stdx = std::sqrt(accmx / double(pts_.size() - 1));
  double stdy = std::sqrt(accmy / double(pts_.size() - 1));

  double sx = std::sqrt(2.) / stdx;
  double sy = std::sqrt(2.) / stdy;

  matrix_trans_ << sx, 0, -sx * mean_x, 0, sy, -sy * mean_y, 0, 0, 1;
}
