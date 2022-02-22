#include "nonlinear_optimizer.hpp"

void NonlinearOptimizer::refine_H(const std::vector<cv::Point2f> &img_pts_,
                                  const std::vector<cv::Point2f> &board_pts_,
                                  const Eigen::Matrix3d &matrix_H_,
                                  Eigen::Matrix3d &refined_H_) {
  Eigen::MatrixXd x1(2, board_pts_.size());
  Eigen::MatrixXd x2(2, img_pts_.size());

  for (int i = 0; i < board_pts_.size(); ++i) {
    x1(0, i) = board_pts_[i].x;
    x1(1, i) = board_pts_[i].y;
  }
  for (int i = 0; i < img_pts_.size(); ++i) {
    x2(0, i) = img_pts_[i].x;
    x2(1, i) = img_pts_[i].y;
  }

  Eigen::Matrix3d H = matrix_H_;
  // std::cout << "H:" << H<< std::endl;
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
        NULL, H.data());
  }
  EstimateHomographyOptions options;
  options.expected_average_symmetric_distance = 0.02;
  // Configure the solve.
  ceres::Solver::Options solver_options;
  solver_options.linear_solver_type = ceres::DENSE_QR;
  solver_options.max_num_iterations = options.max_num_iterations;
  solver_options.update_state_every_iteration = true;

  // Terminate if the average symmetric distance is good enough.
  TerminationCheckingCallback callback(x1, x2, options, &H);
  solver_options.callbacks.push_back(&callback);

  // Run the solve.
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);

  refined_H_ = H / H(2, 2);
}

void NonlinearOptimizer::refine_all_camera_params(
    const Params &params_,
    const std::vector<std::vector<cv::Point2f>> &imgs_pts_,
    const std::vector<std::vector<cv::Point2f>> &bords_pts_,
    Params &refined_params_) {
  ceres::Problem problem;
  Eigen::Matrix3d camera_matrix = params_.camera_matrix;
  Eigen::VectorXd k = params_.k;
  std::vector<Eigen::MatrixXd> vec_rt = params_.vec_rt;
  Eigen::VectorXd v_camera_matrix(5);
  v_camera_matrix << camera_matrix(0, 0), camera_matrix(0, 1),
      camera_matrix(0, 2), camera_matrix(1, 1), camera_matrix(1, 2);
  double *p_camera = v_camera_matrix.data();
  double *p_k = k.data();

  // package all rt
  std::vector<Eigen::VectorXd> packet_rt;
  for (int n = 0; n < params_.vec_rt.size(); ++n) {
    Eigen::AngleAxisd r(vec_rt[n].block<3, 3>(0, 0));
    Eigen::VectorXd rot_vec(r.axis() * r.angle());
    Eigen::VectorXd rt(6);
    rt << rot_vec(0), rot_vec(1), rot_vec(2), vec_rt[n](0, 3), vec_rt[n](1, 3),
        vec_rt[n](2, 3);
    packet_rt.push_back(rt);
  }
  for (int n = 0; n < params_.vec_rt.size(); ++n) {
    Eigen::MatrixXd x1(2, bords_pts_[n].size());
    Eigen::MatrixXd x2(2, imgs_pts_[n].size());

    for (int i = 0; i < bords_pts_[n].size(); ++i) {
      x1(0, i) = bords_pts_[n][i].x;
      x1(1, i) = bords_pts_[n][i].y;
    }
    for (int i = 0; i < imgs_pts_[n].size(); ++i) {
      x2(0, i) = imgs_pts_[n][i].x;
      x2(1, i) = imgs_pts_[n][i].y;
    }

    double *p_rt = &packet_rt[n](0);
    for (int i = 0; i < x1.cols(); i++) {
      ReprojectionError *cost_function =
          new ReprojectionError(x2.col(i), x1.col(i));

      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<ReprojectionError,
                                          2, // num_residuals
                                          5, 3, 6>(cost_function),
          NULL, p_camera, p_k, p_rt);
    }
  }
  // Configure the solver.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;

  // Solve!
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  DLOG(INFO) << "Final Brief Report:\n" << summary.BriefReport() << std::endl;
  this->formate_data(v_camera_matrix, k, packet_rt, refined_params_);
}

void NonlinearOptimizer::formate_data(const Eigen::VectorXd &v_camera_matrix_,
                                      const Eigen::VectorXd &v_dist_,
                                      const std::vector<Eigen::VectorXd> &v_rt_,
                                      Params &params_) {
  params_.camera_matrix << v_camera_matrix_(0), v_camera_matrix_(1),
      v_camera_matrix_(2), 0., v_camera_matrix_(3), v_camera_matrix_(4), 0, 0,
      1.;
  params_.k = v_dist_;
  params_.vec_rt.clear();
  for (const auto &rt : v_rt_) {
    Eigen::Vector3d rv(rt(0), rt(1), rt(2));
    Eigen::AngleAxisd r_v(rv.norm(), rv / rv.norm());
    Eigen::Matrix<double, 3, 4> rt1;
    rt1.block<3, 3>(0, 0) = r_v.toRotationMatrix();
    rt1.block<3, 1>(0, 3) = Eigen::Vector3d(rt(3), rt(4), rt(5));
    params_.vec_rt.push_back(rt1);
  }
}

void NonlinearOptimizer::refine_lidar2camera_params(
    const LidarParams &params_,
    const std::vector<std::vector<cv::Point2f>> &imgs_pts_,
    const std::vector<std::vector<cv::Point2f>> &bords_pts_,
    const std::vector<LidarPointPair> lidar_point_pairs,
    LidarParams &refined_params_) {
  ceres::Problem problem;
  Eigen::Matrix3d camera_matrix = params_.camera_matrix;
  Eigen::VectorXd k = params_.k;
  std::vector<Eigen::MatrixXd> vec_rt = params_.vec_rt;
  Eigen::VectorXd v_camera_matrix(5);
  v_camera_matrix << camera_matrix(0, 0), camera_matrix(0, 1),
      camera_matrix(0, 2), camera_matrix(1, 1), camera_matrix(1, 2);
  double *p_camera = v_camera_matrix.data();
  double *p_k = k.data();

  // package all rt
  std::vector<Eigen::VectorXd> packet_rt;
  for (int n = 0; n < params_.vec_rt.size(); ++n) {
    Eigen::AngleAxisd r(vec_rt[n].block<3, 3>(0, 0));
    Eigen::VectorXd rot_vec(r.axis() * r.angle());
    Eigen::VectorXd rt(6);
    rt << rot_vec(0), rot_vec(1), rot_vec(2), vec_rt[n](0, 3), vec_rt[n](1, 3),
        vec_rt[n](2, 3);
    packet_rt.push_back(rt);
  }
  for (int n = 0; n < params_.vec_rt.size(); ++n) {
    Eigen::MatrixXd x1(2, bords_pts_[n].size());
    Eigen::MatrixXd x2(2, imgs_pts_[n].size());

    for (int i = 0; i < bords_pts_[n].size(); ++i) {
      x1(0, i) = bords_pts_[n][i].x;
      x1(1, i) = bords_pts_[n][i].y;
    }
    for (int i = 0; i < imgs_pts_[n].size(); ++i) {
      x2(0, i) = imgs_pts_[n][i].x;
      x2(1, i) = imgs_pts_[n][i].y;
    }

    double *p_rt = &packet_rt[n](0);
    for (int i = 0; i < x1.cols(); i++) {
      ReprojectionError *cost_function =
          new ReprojectionError(x2.col(i), x1.col(i));

      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<ReprojectionError,
                                          2, // num_residuals
                                          5, 2, 6>(cost_function),
          NULL, p_camera, p_k, p_rt);
    }
  }
  // lidar2camera
  Eigen::Matrix<double, 3, 4> initial_rt = params_.extrinsic;
  Eigen::AngleAxisd r(initial_rt.block<3, 3>(0, 0));
  Eigen::VectorXd rot_vec(r.axis() * r.angle());
  Eigen::VectorXd rt(6);
  rt << rot_vec(0), rot_vec(1), rot_vec(2), initial_rt(0, 3), initial_rt(1, 3),
      initial_rt(2, 3);
  double *p_rt = &rt(0);
  for (size_t i = 0; i < lidar_point_pairs.size(); i++) {
    LidarPointPair lidar_pair = lidar_point_pairs[i];
    for (size_t j = 0; j < 4; j++) {
      Eigen::Vector3d lidar_3d_point(lidar_pair.lidar_3d_point[j].x,
                                     lidar_pair.lidar_3d_point[j].y,
                                     lidar_pair.lidar_3d_point[j].z);
      Eigen::Vector2d lidar_2d_point(lidar_pair.lidar_2d_point[j].x,
                                     lidar_pair.lidar_2d_point[j].y);

      LidarReprojectionError *cost_function =
          new LidarReprojectionError(lidar_2d_point, lidar_3d_point);

      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<LidarReprojectionError,
                                          2, // num_residuals
                                          5, 2, 6>(cost_function),
          NULL, p_camera, p_k, p_rt);
    }
  }

  // Configure the solver.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;

  // Solve!
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // DLOG(INFO) << "Final Brief Report:\n" << summary.BriefReport() <<
  // std::endl;
  this->formate_data(v_camera_matrix, k, packet_rt, rt, refined_params_);
}

void NonlinearOptimizer::formate_data(const Eigen::VectorXd &v_camera_matrix_,
                                      const Eigen::VectorXd &v_dist_,
                                      const std::vector<Eigen::VectorXd> &v_rt_,
                                      const Eigen::VectorXd &RT,
                                      LidarParams &params_) {
  params_.camera_matrix << v_camera_matrix_(0), v_camera_matrix_(1),
      v_camera_matrix_(2), 0., v_camera_matrix_(3), v_camera_matrix_(4), 0, 0,
      1.;
  params_.k = v_dist_;
  params_.vec_rt.clear();
  for (const auto &rt : v_rt_) {
    Eigen::Vector3d rv(rt(0), rt(1), rt(2));
    Eigen::AngleAxisd r_v(rv.norm(), rv / rv.norm());
    Eigen::Matrix<double, 3, 4> rt1;
    rt1.block<3, 3>(0, 0) = r_v.toRotationMatrix();
    rt1.block<3, 1>(0, 3) = Eigen::Vector3d(rt(3), rt(4), rt(5));
    params_.vec_rt.push_back(rt1);
  }

  Eigen::Vector3d rv(RT(0), RT(1), RT(2));
  Eigen::AngleAxisd r_v(rv.norm(), rv / rv.norm());
  Eigen::Matrix<double, 3, 4> rt1;
  rt1.block<3, 3>(0, 0) = r_v.toRotationMatrix();
  rt1.block<3, 1>(0, 3) = Eigen::Vector3d(RT(3), RT(4), RT(5));
  std::cout << rv << std::endl;
  params_.extrinsic = rt1;
}