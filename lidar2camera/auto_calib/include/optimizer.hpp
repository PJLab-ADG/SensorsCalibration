/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#ifndef OPTIMIZER_HPP_
#define OPTIMIZER_HPP_

#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

class Optimizer {
private:
  Eigen::Matrix3d original_intrinsic_;
  Eigen::Matrix4d original_extrinsic_;

  Eigen::Matrix3d curr_optim_intrinsic_;
  Eigen::Matrix4d curr_optim_extrinsic_;

  float current_cost_fun_;
  float current_fc_score_;

  cv::Mat register_img_;
  pcl::PointCloud<pcl::PointXYZI> register_cloud_;

public:
  Optimizer(const Eigen::Matrix3d *intrinsic_,
            const Eigen::Matrix4d *extrinsic);
  ~Optimizer();

  void Calibrate(const cv::Mat *distance_img,
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud);
  float CostFunction(const float var[6], const cv::Mat *distance_img,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud);
  float MaskRegistrationLoss(
      const float var[6], const cv::Mat *distance_img,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud);
  void
  SaveProjectResult(const std::string img_name, const cv::Mat *distance_img,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud);
  void random_search_params(int search_count, float delta_x, float delta_y,
                            float delta_z, float delta_roll, float delta_pitch,
                            float delta_yaw);
};

#endif //  OPTIMIZER_HPP_