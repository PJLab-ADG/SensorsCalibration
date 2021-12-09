/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include "optimizer.hpp"
#include <ctime>
#include <random>

#include "logging.hpp"
#include "transform_util.hpp"
Optimizer::Optimizer(const Eigen::Matrix3d *intrinsic,
                     const Eigen::Matrix4d *extrinsic) {
  original_intrinsic_ = *intrinsic;
  original_extrinsic_ = *extrinsic;

  curr_optim_intrinsic_ = *intrinsic;
  curr_optim_extrinsic_ = *extrinsic;

  current_fc_score_ = 0;
}

Optimizer::~Optimizer() {}

void Optimizer::Calibrate(
    const cv::Mat *distance_img,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud) {

  register_img_ = *distance_img;
  register_cloud_ = *register_cloud;

  // SaveProjectResult("before.png", distance_img, original_cloud);
  float var[6] = {0}, bestVal[6] = {0}, best_extrinsic_vec[6] = {0};
  std::string varName[6] = {"roll", "pitch", "yaw", "tx", "ty", "tz"};
  int direction[3] = {1, 0, -1};
  float maxPointCnt = CostFunction(var, distance_img, register_cloud);

  std::cout << "before loss: " << maxPointCnt << std::endl;

  int iteration_num = 0;
  Eigen::Matrix3d rotation_matrix = curr_optim_extrinsic_.block<3, 3>(0, 0);
  Eigen::Vector3d ea = rotation_matrix.eulerAngles(2, 1, 0);
  float target_extrinsic_vec[6] = {0}, curr_optim_extrinsic_vec[6] = {0};
  bool is_violence_search = false;
  if (is_violence_search) {
    float rpy_resolution = 1;
    float xyz_resolution = 0.1;
    for (int i1 = -5; i1 < 5; i1++) {
      for (int i2 = -5; i2 < 5; i2++) {
        for (int i3 = -5; i3 < 5; i3++) {
          for (int i4 = -5; i4 < 5; i4++) {
            for (int i5 = -5; i5 < 5; i5++) {
              for (int i6 = -5; i6 < 5; i6++) {
                var[0] = i1 * rpy_resolution;
                var[1] = i2 * rpy_resolution;
                var[2] = i3 * rpy_resolution;
                var[3] = i4 * xyz_resolution;
                var[4] = i5 * xyz_resolution;
                var[5] = i6 * xyz_resolution;
                float cnt = CostFunction(var, distance_img, register_cloud);
                if (cnt > maxPointCnt) {
                  maxPointCnt = cnt;
                  for (size_t k = 0; k < 6; k++) {
                    bestVal[k] = var[k];
                  }

                  std::cout << "match point increase to: " << maxPointCnt
                            << std::endl;
                }
              }
            }
          }
        }
      }
    }
    curr_optim_extrinsic_ =
        curr_optim_extrinsic_ * TransformUtil::GetDeltaT(bestVal);
    current_cost_fun_ = maxPointCnt;
  } else {

    bool f1 = false, f2 = false;

    for (size_t k = 0; k < 100; k++) {
      if (k % 2 == 0) {

        random_search_params(10000, 0.05, 0.05, 0.05, 1, 1, 1);
        f1 = current_fc_score_ == 1;
      } else {

        random_search_params(10000, 0.005, 0.005, 0.005, 0.5, 0.5, 0.5);
        f2 = current_fc_score_ == 1;
      }
      if (f1 && f2)
        break;
    }

    for (size_t k = 0; k < 100; k++) {

      random_search_params(1000, 0.005, 0.005, 0.005, 0.5, 0.5, 0.5);
      if (current_fc_score_ == 1)
        break;
    }

    for (size_t k = 0; k < 100; k++) {

      random_search_params(1000, 0.001, 0.001, 0.001, 0.1, 0.1, 0.1);
      if (current_fc_score_ == 1)
        break;
    }

    for (size_t k = 0; k < 100; k++) {

      random_search_params(1000, 0.001, 0.001, 0.001, 0.05, 0.05, 0.05);
      if (current_fc_score_ == 1)
        break;
    }
  }

  std::cout << "after loss: " << current_cost_fun_ << std::endl;
  SaveProjectResult("feature_projection.png", distance_img, register_cloud);
}

float Optimizer::MaskRegistrationLoss(
    const float var[6], const cv::Mat *distance_img,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud) {

  size_t PointCnt = 0;
  // cv::Mat image = distance_img->clone();
  Eigen::Matrix4d deltaT = TransformUtil::GetDeltaT(var);
  Eigen::Matrix4d T = curr_optim_extrinsic_;
  T *= deltaT;
  for (const auto &src_pt : register_cloud->points) {
    if (!pcl_isfinite(src_pt.x) || !pcl_isfinite(src_pt.y) ||
        !pcl_isfinite(src_pt.z))
      continue;
    Eigen::Vector4d vec;
    vec << src_pt.x, src_pt.y, src_pt.z, 1;

    Eigen::Vector4d cam_point = T * vec;
    Eigen::Vector3d cam_vec;
    cam_vec << cam_point(0), cam_point(1), cam_point(2);
    Eigen::Vector3d vec_2d = curr_optim_intrinsic_ * cam_vec;
    if (vec_2d(2) > 0) {

      int x = (int)cvRound(vec_2d(0) / vec_2d(2));
      int y = (int)cvRound(vec_2d(1) / vec_2d(2));

      if (x >= 0 && x < distance_img->cols && y >= 0 &&
          y < distance_img->rows) {
        float f1 = (*distance_img).at<cv::Vec3b>(y, x)[0];
        float f2 = (*distance_img).at<cv::Vec3b>(y, x)[1];
        float f3 = (*distance_img).at<cv::Vec3b>(y, x)[2];
        double pixel_gray = (f3 + f2 + f1) / 3;

        if (pixel_gray > 55) {
          PointCnt++;
        }
      }
    }
  }
  // cv::imwrite("before_refine.png",image);
  return (float)PointCnt / register_cloud->points.size();
}

float Optimizer::CostFunction(
    const float var[6], const cv::Mat *distance_img,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud) {

  float registration_cost =
      MaskRegistrationLoss(var, distance_img, register_cloud);
  return registration_cost;
}

void Optimizer::SaveProjectResult(
    const std::string img_name, const cv::Mat *distance_img,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud) {

  Eigen::Matrix3d rotation_matrix = curr_optim_extrinsic_.block(0, 0, 3, 3);
  Eigen::Vector3d translation;
  translation(0) = curr_optim_extrinsic_(0, 3);
  translation(1) = curr_optim_extrinsic_(1, 3);
  translation(2) = curr_optim_extrinsic_(2, 3);
  Eigen::Vector3d eulerAngle =
      TransformUtil::Rotation2Eul(rotation_matrix); //.eulerAngles(2, 1, 0);

  std::cout << rotation_matrix << std::endl;
  std::cout << "refine extrinsic: " << std::endl;
  std::cout << "rotation: " << eulerAngle(0) << " " << eulerAngle(1) << " "
            << eulerAngle(2) << std::endl;
  std::cout << "translation: " << translation(0) << " " << translation(1) << " "
            << translation(2) << std::endl;
  cv::Mat image = distance_img->clone();
  for (const auto &src_pt : register_cloud->points) {
    if (!pcl_isfinite(src_pt.x) || !pcl_isfinite(src_pt.y) ||
        !pcl_isfinite(src_pt.z))
      continue;
    Eigen::Vector4d vec;
    vec << src_pt.x, src_pt.y, src_pt.z, 1;

    Eigen::Vector4d cam_point = curr_optim_extrinsic_ * vec;
    Eigen::Vector3d cam_vec;
    cam_vec << cam_point(0), cam_point(1), cam_point(2);
    Eigen::Vector3d vec_2d = curr_optim_intrinsic_ * cam_vec;
    if (vec_2d(2) > 0) {

      int x = (int)cvRound(vec_2d(0) / vec_2d(2));
      int y = (int)cvRound(vec_2d(1) / vec_2d(2));

      if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
        cv::Point centerCircle2(x, y);
        cv::Scalar color = cv::Scalar(0, 255, 0);
        cv::circle(image, centerCircle2, 3, color, -1, 0);
      }
    }
  }
  cv::imwrite(img_name, image);
}

void Optimizer::random_search_params(int search_count, float delta_x,
                                     float delta_y, float delta_z,
                                     float delta_roll, float delta_pitch,
                                     float delta_yaw) {

  float better_cnt = 0.0;
  float var[6] = {0}, bestVal[6] = {0};

  std::default_random_engine generator((clock() - time(0)) /
                                       (double)CLOCKS_PER_SEC);
  std::uniform_real_distribution<double> distribution_x(-delta_x, delta_x);
  std::uniform_real_distribution<double> distribution_y(-delta_y, delta_y);
  std::uniform_real_distribution<double> distribution_z(-delta_z, delta_z);
  std::uniform_real_distribution<double> distribution_roll(-delta_roll,
                                                           delta_roll);
  std::uniform_real_distribution<double> distribution_pitch(-delta_pitch,
                                                            delta_pitch);
  std::uniform_real_distribution<double> distribution_yaw(-delta_yaw,
                                                          delta_yaw);
  float maxPointCnt =
      CostFunction(var, &register_img_, register_cloud_.makeShared());
  for (size_t i = 0; i < search_count; i++) {
    var[0] = distribution_x(generator);
    var[1] = distribution_y(generator);
    var[2] = distribution_z(generator);
    var[3] = distribution_roll(generator);
    var[4] = distribution_pitch(generator);
    var[5] = distribution_yaw(generator);
    float cnt = CostFunction(var, &register_img_, register_cloud_.makeShared());
    if (cnt > maxPointCnt) {
      better_cnt++;
      maxPointCnt = cnt;
      for (size_t k = 0; k < 6; k++) {
        bestVal[k] = var[k];
      }
    }
  }

  curr_optim_extrinsic_ =
      curr_optim_extrinsic_ * TransformUtil::GetDeltaT(bestVal);

  current_cost_fun_ = maxPointCnt;
  current_fc_score_ = 1 - better_cnt / search_count;
}
