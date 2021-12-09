/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#include "lidar_feature.hpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "logging.hpp"

LidarFeature::LidarFeature(const std::string &cloud_path,
                           const Eigen::Matrix3d &intrinsic,
                           const Eigen::Matrix4d &extrinsic) {
  pcd_path_ = cloud_path;
  intrinsic_ = intrinsic;
  extrinsic_ = extrinsic;
}

LidarFeature::~LidarFeature() {}

void LidarFeature::extract_feature(
    const cv::Mat *distance_images,
    pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud) {

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pole(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lane(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_car(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud_lane(
      new pcl::PointCloud<pcl::PointXYZI>);
  int cloud_size = 0;
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path_, *cloud) ==
      -1) //* load the file
  {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return;
  }
  original_pcd_ = *cloud;
  // pre-filtering by extrinsic
  for (const auto &point : *cloud) {
    Eigen::Vector4d vec;
    vec << point.x, point.y, point.z, 1;

    Eigen::Vector4d cam_point = extrinsic_ * vec;
    Eigen::Vector3d cam_vec;
    cam_vec << cam_point(0), cam_point(1), cam_point(2);
    Eigen::Vector3d vec_2d = intrinsic_ * cam_vec;

    if (vec_2d(2) > 0) {
      int x = (int)cvRound(vec_2d(0) / vec_2d(2));
      int y = (int)cvRound(vec_2d(1) / vec_2d(2));
      if (x >= 0 && x < distance_images->cols && y >= 0 &&
          y < distance_images->rows) {
        if (vec(0) > x_min_threshold_ && vec(0) < x_max_threshold_ &&
            vec(1) > y_min_threshold_ && vec(1) < y_max_threshold_) {
          pcl::PointXYZI pc;
          pc.x = vec(0);
          pc.y = vec(1);
          pc.z = vec(2);
          pc.intensity = point.intensity;
          cloud_filtered->points.push_back(pc);
          cloud_size++;
        }
      }
    } else {
    }
  }
  cloud_filtered->height = 1;
  cloud_filtered->width = cloud_size;

  // extract lane, car, pole,
  extract_objects(cloud_filtered, cloud_pole, cloud_lane, cloud_car,
                  trans_cloud_lane);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lane_pole(
      new pcl::PointCloud<pcl::PointXYZI>);
  *cloud_lane_pole = *cloud_pole + *cloud_lane;

  *register_cloud = *cloud_lane_pole;
}

void LidarFeature::extract_objects(
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_pole,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_lane,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_car,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud_lane) {

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.2);
  seg.setMaxIterations(3000);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZI>),
      cloud_p(new pcl::PointCloud<pcl::PointXYZI>),
      cloud_f(new pcl::PointCloud<pcl::PointXYZI>),
      cloud_lane_mark(new pcl::PointCloud<pcl::PointXYZI>);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.empty()) {
    LOGW("Could not estimate a planar model for the given dataset.");
    return;
  }
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_f);
  extract.setNegative(false);
  extract.filter(*cloud_p);
  cloud_filtered->clear();
  cloud_lane->clear();

  // get lane
  for (unsigned long i = 0; i < cloud_p->size(); i++) {
    if (cloud_p->points[i].intensity >= intensity_threshold &&
        cloud_p->points[i].y > -10 && cloud_p->points[i].y < 10) {
      cloud_lane->push_back(cloud_p->points[i]);
      cloud_lane_mark->push_back(cloud_p->points[i]);
    }
  }
  // tans
  float plane[4];
  plane[0] = coefficients->values[0];
  plane[1] = coefficients->values[1];
  plane[2] = coefficients->values[2];
  plane[3] = coefficients->values[3];

  Eigen::Vector3d plane_normal(plane[0], plane[1], plane[2]);
  Eigen::Vector3d z_axis(0, 0, 1);

  double alpha =
      -acos(z_axis.dot(plane_normal) / (z_axis.norm() * plane_normal.norm()));
  Eigen::Vector3d rot_axis = z_axis.cross(plane_normal);
  rot_axis.normalize();

  Eigen::Matrix3d rot_tmp;
  rot_tmp = Eigen::AngleAxisd(alpha, rot_axis);
  Eigen::Matrix4d left_rot_tmp = Eigen::Matrix4d::Identity();
  left_rot_tmp.block(0, 0, 3, 3) = rot_tmp;
  left_rot_tmp(0, 3) = 0;
  left_rot_tmp(1, 3) = 0;
  left_rot_tmp(2, 3) = 0;

  pcl::PointCloud<pcl::PointXYZI>::Ptr trans_ground_point(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cloud_lane, *trans_ground_point, left_rot_tmp);
  *trans_cloud_lane = *trans_ground_point;

  // lidar height
  pcl::PointCloud<pcl::PointXYZI>::Ptr trans_target_lidar(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cloud_p, *trans_target_lidar, left_rot_tmp);
  float left_plane_tmp_plane[4] = {0};
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_ground_cloud_tmp(
      new pcl::PointCloud<pcl::PointXYZI>);
  plane_model(trans_target_lidar, lidar_ground_cloud_tmp, left_plane_tmp_plane);
  double left_offset_z = left_plane_tmp_plane[3];
  left_rot_tmp(2, 3) = left_offset_z;

  // get pole
  double delta_x = 0.7;
  double delta_y = 0.7;
  struct PointGrid {
    PointGrid() = default;
    float max = -100;
    float min = 100;
    float avg = 0;
    std::vector<pcl::PointXYZI> points;
  };
  std::vector<std::vector<PointGrid>> pointGrid;
  for (int i = 0; i < ((x_max_threshold_ - x_min_threshold_) / delta_x + 1);
       i++) {
    pointGrid.emplace_back((y_max_threshold_ - y_min_threshold_) / delta_y + 1);
  }

  for (auto &point : *cloud_f) {
    if (point.x >= x_min_threshold_ && point.x < x_max_threshold_ &&
        (point.y >= y_min_threshold_ && point.y < y_max_threshold_)) {
      int xx = floor((point.x - x_min_threshold_) / delta_x);
      int yy = floor((point.y - y_min_threshold_) / delta_y);
      if (pointGrid[xx][yy].max < point.z)
        pointGrid[xx][yy].max = point.z;
      if (pointGrid[xx][yy].min > point.z)
        pointGrid[xx][yy].min = point.z;
      pointGrid[xx][yy].avg += point.z;
      pointGrid[xx][yy].points.push_back(point);
    }
  }

  double elevation_min = -2;
  double elevation_max = 4;

  for (unsigned long i = 0; i < pointGrid.size(); i++) {
    for (unsigned long j = 0; j < pointGrid[0].size(); j++) {
      if (pointGrid[i][j].max > elevation_max) {
        for (auto &point : pointGrid[i][j].points) {
          if (point.z > elevation_min) {
            cloud_pole->push_back(point);
          }
        }
      }
    }
  }
}

void LidarFeature::plane_model(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &full_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &ground, float plane[4]) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  const static float INLIER_DIST_THRD = 0.05f;

  // Optional
  seg.setOptimizeCoefficients(true); // true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(40);
  seg.setDistanceThreshold(INLIER_DIST_THRD);
  seg.setInputCloud(full_cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    PCL_ERROR("could not estimate a planner model for the given dataset");
  }
  plane[0] = coefficients->values[0];
  plane[1] = coefficients->values[1];
  plane[2] = coefficients->values[2];
  plane[3] = coefficients->values[3];
  std::cerr << "Model coefficients" << coefficients->values[0] << " "
            << coefficients->values[1] << " " << coefficients->values[2] << " "
            << coefficients->values[3] << std::endl;
  std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

  float distance;
  for (size_t i = 0; i < full_cloud->points.size(); i++) {
    distance = fabs(coefficients->values[0] * full_cloud->points[i].x +
                    coefficients->values[1] * full_cloud->points[i].y +
                    coefficients->values[2] * full_cloud->points[i].z +
                    coefficients->values[3]);
    if (distance <= INLIER_DIST_THRD) {
      ground_cloud->push_back(full_cloud->points[i]);
    }
  }
  *ground = *ground_cloud;
}
