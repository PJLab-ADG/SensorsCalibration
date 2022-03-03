/*
 * Copyright (C) 2022 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include "round_hole_board/lidar_pattern.h"

#include <dirent.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <stdio.h>

#include <pcl/io/pcd_io.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "logging.hpp"
namespace lidarcalib {

void LidarDetector::LidarDetection(std::string pcds_dir) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloude(
      new pcl::PointCloud<pcl::PointXYZ>);

  std::string lidar_dir = pcds_dir;
  if (lidar_dir.rfind('/') != lidar_dir.size() - 1) {
    lidar_dir = lidar_dir + "/";
  }

  DIR *dir;
  if ((dir = opendir(lidar_dir.c_str())) == NULL) {
    std::cout << "Open dir error !" << std::endl;
    exit(1);
  }
  struct dirent *ptr;
  while ((ptr = readdir(dir)) != NULL) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
      std::string pcd_path = pcds_dir + ptr->d_name;
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *temp_cloude) == -1) {
        std::cout << "Couldn't read file rabbit.pcd\n" << std::endl;
        exit(1);
      }
      *cloud += *temp_cloude;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr velocloud(
      new pcl::PointCloud<pcl::PointXYZ>),
      velo_filtered(new pcl::PointCloud<pcl::PointXYZ>),
      velo_filtered2(new pcl::PointCloud<pcl::PointXYZ>),
      plane_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      edges_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  velocloud = cloud;

  pcl::PassThrough<pcl::PointXYZ> pass1;
  pass1.setInputCloud(velocloud);
  pass1.setFilterFieldName("x");
  pass1.setFilterLimits(5, 7);
  pass1.filter(*velo_filtered);

  pcl::PassThrough<pcl::PointXYZ> pass2;
  pass2.setInputCloud(velo_filtered);
  pass2.setFilterFieldName("y");
  pass2.setFilterLimits(-2.5, 0);
  pass2.filter(*velo_filtered2);

  pcl::io::savePCDFileBinary("test_filtered.pcd", *velo_filtered2);
  // Plane segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  Eigen::Vector3f axis(0, 1, 0);
  pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation;
  plane_segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  plane_segmentation.setDistanceThreshold(0.05);
  plane_segmentation.setMethodType(pcl::SAC_RANSAC);
  plane_segmentation.setAxis(axis);
  plane_segmentation.setEpsAngle(0.05);
  plane_segmentation.setOptimizeCoefficients(true);
  plane_segmentation.setMaxIterations(10);
  plane_segmentation.setInputCloud(velo_filtered2);
  plane_segmentation.segment(*inliers, *coefficients);

  float a_final = coefficients->values[0] / coefficients->values[3];
  float b_final = coefficients->values[1] / coefficients->values[3];
  float c_final = coefficients->values[2] / coefficients->values[3];

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(velo_filtered2);
  extract.setIndices(inliers);
  extract.filter(*plane_cloud);
  // pcl::io::savePCDFileBinary("test_plane.pcd", *plane_cloud);
  edges_cloud = plane_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_edges_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  float theta_z = -atan(b_final / a_final);
  float theta_y = atan(c_final / a_final);
  Eigen::MatrixXf R_y(3, 3), R_z(3, 3);
  R_y << cos(theta_y), 0, sin(theta_y), 0, 1, 0, -sin(theta_y), 0, cos(theta_y);
  R_z << cos(theta_z), -sin(theta_z), 0, sin(theta_z), cos(theta_z), 0, 0, 0, 1;
  Eigen::MatrixXf R = R_y * R_z;
  float average_x = 0.0;
  int cnt = 0;
  float min_pt_x = 99, max_pt_x = -99, min_pt_y = 99, max_pt_y = -99;
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt =
           edges_cloud->points.begin();
       pt < edges_cloud->points.end(); ++pt) {
    Eigen::MatrixXf tmp(3, 1);
    tmp << pt->x, pt->y, pt->z;
    Eigen::MatrixXf changed = R * tmp;
    pt->x = changed(1, 0);
    pt->y = changed(2, 0);
    pt->z = 0;
    if (pt->x < min_pt_x)
      min_pt_x = pt->x;
    if (pt->x > max_pt_x)
      max_pt_x = pt->x;
    if (pt->y < min_pt_y)
      min_pt_y = pt->y;
    if (pt->y > max_pt_y)
      max_pt_y = pt->y;
    average_x += changed(0, 0);
    cnt++;

    plane_edges_cloud->points.push_back(*pt);
  }
  average_x /= cnt;
  plane_edges_cloud->height = 1;
  plane_edges_cloud->width = plane_edges_cloud->points.size();
  // pcl::io::savePCDFileASCII("plane_edges_cloud.pcd", *plane_edges_cloud);
  std::vector<std::vector<bool>> pro_map(
      int(max_pt_y * 200) - int(min_pt_y * 200) + 1,
      std::vector<bool>(int(max_pt_x * 200) - int(min_pt_x * 200) + 1, false));
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt =
           plane_edges_cloud->points.begin();
       pt < plane_edges_cloud->points.end(); ++pt) {
    pro_map[int(pt->y * 200) - int(min_pt_y * 200)]
           [int(pt->x * 200) - int(min_pt_x * 200)] = true;
  }

  int max_cnt = 0;
  int max_i, max_j;

  for (int i = 0; i + 240 < pro_map.size(); i += 2) {
    for (int j = 0; j + 240 < pro_map[0].size(); j += 2) {
      int cnt = 0;
      for (int ii = i; ii - i <= 240; ii += 2) {
        for (int jj = j; jj - j <= 240; jj += 2) {
          if (pro_map[ii][jj])
            cnt++;
        }
      }
      if (cnt > max_cnt) {
        max_cnt = cnt;
        max_i = i;
        max_j = j;
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr initial_centers(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ center;
  center.y = float(max_j) / 200 + min_pt_x + 0.3;
  center.z = float(max_i) / 200 + min_pt_y + 0.3;
  std::cout << center.y << ',' << center.z << std::endl;
  initial_centers->push_back(center);
  center.y = float(max_j) / 200 + min_pt_x + 0.3;
  center.z = float(max_i) / 200 + min_pt_y + 0.9;
  std::cout << center.y << ',' << center.z << std::endl;
  initial_centers->push_back(center);
  center.y = float(max_j) / 200 + min_pt_x + 0.9;
  center.z = float(max_i) / 200 + min_pt_y + 0.3;
  std::cout << center.y << ',' << center.z << std::endl;
  initial_centers->push_back(center);
  center.y = float(max_j) / 200 + min_pt_x + 0.9;
  center.z = float(max_i) / 200 + min_pt_y + 0.9;
  std::cout << center.y << ',' << center.z << std::endl;
  initial_centers->push_back(center);
  // circle detection
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(
      new pcl::PointCloud<pcl::PointXYZ>),
      centroid_candidates(new pcl::PointCloud<pcl::PointXYZ>);
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt =
           initial_centers->points.begin();
       pt < initial_centers->points.end(); ++pt) {
    int initial_x = int(pt->y * 200) - int(min_pt_x * 200);
    int initial_y = int(pt->z * 200) - int(min_pt_y * 200);
    std::cout << "initial: " << initial_x << ',' << initial_y << std::endl;
    int min_cnt = 9999;
    int cnt_cnt = 0;
    pcl::PointXYZ refined_center;
    int final_i, final_j;
    for (int i = initial_y - 20; i <= initial_y + 20; i++) {
      for (int j = initial_x - 20; j <= initial_x + 20; j++) {
        int cnt = 0;
        for (int ii = i - 21; ii <= i + 21; ii++) {
          for (int jj = j - 21; jj <= j + 21; jj++) {
            if (pro_map[ii][jj]) {
              if ((ii - i) * (ii - i) + (jj - j) * (jj - j) < 21 * 21)
                cnt++;
            }
          }
        }
        if (cnt < min_cnt) {
          cnt_cnt = 1;
          final_i = i;
          final_j = j;
          min_cnt = cnt;
          refined_center.x = average_x;
          refined_center.y = float(j) / 200 + min_pt_x;
          refined_center.z = float(i) / 200 + min_pt_y;
        } else if (cnt == min_cnt) {
          refined_center.y =
              (refined_center.y * cnt_cnt + float(j) / 200 + min_pt_x) /
              (cnt_cnt + 1);
          refined_center.z =
              (refined_center.z * cnt_cnt + float(i) / 200 + min_pt_y) /
              (cnt_cnt + 1);
          cnt_cnt++;
        }
      }
    }
    std::cout << "pro_map: " << final_i << ',' << final_j << std::endl;
    std::cout << "min_cnt: " << min_cnt << std::endl;
    std::cout << "pro_ct: " << refined_center.y << ',' << refined_center.z
              << std::endl;
    centroid_candidates->push_back(refined_center);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::MatrixXf R_inv = R.inverse();
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt =
           centroid_candidates->points.begin();
       pt < centroid_candidates->points.end(); ++pt) {
    Eigen::MatrixXf tmp(3, 1);
    tmp << pt->x, pt->y, pt->z;
    Eigen::MatrixXf changed = R_inv * tmp;
    pt->x = changed(0, 0);
    pt->y = changed(1, 0);
    pt->z = changed(2, 0);
    std::cout << pt->x << "    " << pt->y << "    " << pt->z << std::endl;
    circle_cloud->points.push_back(*pt);
  }

  circle_cloud->height = 1;
  circle_cloud->width = circle_cloud->points.size();
  //  pcl::io::savePCDFileASCII("final_cloud.pcd", *plane_cloud+*circle_cloud);
  pcl::io::savePCDFileASCII("circle_cloud.pcd", *circle_cloud);
}

} // lidarcalib
