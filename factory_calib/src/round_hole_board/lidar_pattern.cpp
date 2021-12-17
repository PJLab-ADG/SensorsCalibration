/*
  lidar_pattern: Find the circle centers in the lidar cloud
*/
#define PCL_NO_PRECOMPILE
#include "round_hole_board/lidar_pattern.h"
#include "round_hole_board/velo2cam_utils.h"

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

namespace lidarcalib {
LidarDetector::LidarDetector() {}

LidarDetector::~LidarDetector() {}

std::vector<Point3D> LidarDetector::GetLidarDetectPoints() {
  return lidar_points_;
}

bool LidarDetector::LidarCircleCenterDetection(std::string pcds_dir, bool first,
                                               bool front) {
  pcl::PointCloud<Velodyne::Point>::Ptr cloud(
      new pcl::PointCloud<Velodyne::Point>);
  pcl::PointCloud<Velodyne::Point>::Ptr temp_cloude(
      new pcl::PointCloud<Velodyne::Point>);
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
      if (pcl::io::loadPCDFile<Velodyne::Point>(pcd_path, *temp_cloude) == -1) {
        std::cout << "Couldn't read file rabbit.pcd\n" << std::endl;
        exit(1);
      }
      *cloud += *temp_cloude;
    }
  }
  Velodyne::addRange(*cloud, first);
  Velodyne::addTheta(*cloud);

  Velodyne::roifilter(*cloud, front);

  pcl::PointCloud<Velodyne::Point>::Ptr velocloud(
      new pcl::PointCloud<Velodyne::Point>),
      velo_filtered(new pcl::PointCloud<Velodyne::Point>),
      plane_cloud(new pcl::PointCloud<Velodyne::Point>),
      pattern_cloud(new pcl::PointCloud<Velodyne::Point>),
      edges_cloud(new pcl::PointCloud<Velodyne::Point>);

  // Range passthrough filter
  velocloud = cloud;
  velocloud->height = 1;
  velocloud->width = velocloud->points.size();
  pcl::PassThrough<Velodyne::Point> pass2;
  pass2.setInputCloud(velocloud);
  pass2.setFilterFieldName("intensity");
  if (first)
    pass2.setFilterLimits(passthrough_radius_left_min_,
                          passthrough_radius_left_max_);
  else
    pass2.setFilterLimits(passthrough_radius_right_min_,
                          passthrough_radius_right_max_);
  pass2.filter(*velo_filtered);
  // Plane segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<Velodyne::Point> plane_segmentation;
  // pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation;

  plane_segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  plane_segmentation.setDistanceThreshold(plane_threshold_);
  plane_segmentation.setMethodType(pcl::SAC_RANSAC);
  plane_segmentation.setAxis(Eigen::Vector3f(axis_[0], axis_[1], axis_[2]));
  plane_segmentation.setEpsAngle(angle_threshold_);
  plane_segmentation.setOptimizeCoefficients(true);
  plane_segmentation.setMaxIterations(1000);
  plane_segmentation.setInputCloud(velo_filtered);
  plane_segmentation.segment(*inliers, *coefficients);

  float a_final = coefficients->values[0] / coefficients->values[3];
  float b_final = coefficients->values[1] / coefficients->values[3];
  float c_final = coefficients->values[2] / coefficients->values[3];

  if (inliers->indices.size() == 0) {
    // Exit 1: plane not found

    std::cout
        << "[LiDAR] Could not estimate a planar model for the given dataset."
        << std::endl;
    ;
    return false;
  }

  pcl::ExtractIndices<Velodyne::Point> extract;
  extract.setInputCloud(velo_filtered);
  extract.setIndices(inliers);
  extract.filter(*plane_cloud);

  for (pcl::PointCloud<Velodyne::Point>::iterator pt =
           plane_cloud->points.begin();
       pt < plane_cloud->points.end(); ++pt) {
  }

  edges_cloud = plane_cloud;

  // pcl::io::savePCDFileASCII("edges_cloud.pcd", *edges_cloud);

  if (edges_cloud->points.size() == 0) {
    // Exit 2: pattern edges not found
    std::cout << "[LiDAR] Could not detect pattern edges." << std::endl;
    return false;
  }

  // pcl::io::savePCDFileASCII("test_edges.pcd", *edges_cloud);

  pcl::PointCloud<Velodyne::Point>::Ptr plane_edges_cloud(
      new pcl::PointCloud<Velodyne::Point>);

  float theta_z = -atan(b_final / a_final);
  float theta_y = atan(c_final / a_final);
  Eigen::MatrixXf R_y(3, 3), R_z(3, 3);
  R_y << cos(theta_y), 0, sin(theta_y), 0, 1, 0, -sin(theta_y), 0, cos(theta_y);
  R_z << cos(theta_z), -sin(theta_z), 0, sin(theta_z), cos(theta_z), 0, 0, 0, 1;
  Eigen::MatrixXf R = R_y * R_z;
  float average_x = 0.0;
  int cnt = 0;
  float min_pt_x = 99, max_pt_x = -99, min_pt_y = 99, max_pt_y = -99;
  for (pcl::PointCloud<Velodyne::Point>::iterator pt =
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
  // std::ofstream outFile3;
  // outFile3.open("pro_map.csv",std::ios::ate);
  for (pcl::PointCloud<Velodyne::Point>::iterator pt =
           plane_edges_cloud->points.begin();
       pt < plane_edges_cloud->points.end(); ++pt) {
    pro_map[int(pt->y * 200) - int(min_pt_y * 200)]
           [int(pt->x * 200) - int(min_pt_x * 200)] = true;
    // outFile3<<int(pt->y*200)-int(min_pt_y*200)<<','<<int(pt->x*200)-int(min_pt_x*200)<<std::endl;
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

  pcl::PointCloud<Velodyne::Point>::Ptr initial_centers(
      new pcl::PointCloud<Velodyne::Point>);
  Velodyne::Point center;
  center.y = float(max_j) / 200 + min_pt_x + 0.3;
  center.z = float(max_i) / 200 + min_pt_y + 0.3;

  initial_centers->push_back(center);
  center.y = float(max_j) / 200 + min_pt_x + 0.3;
  center.z = float(max_i) / 200 + min_pt_y + 0.9;

  initial_centers->push_back(center);
  center.y = float(max_j) / 200 + min_pt_x + 0.9;
  center.z = float(max_i) / 200 + min_pt_y + 0.3;

  initial_centers->push_back(center);
  center.y = float(max_j) / 200 + min_pt_x + 0.9;
  center.z = float(max_i) / 200 + min_pt_y + 0.9;

  initial_centers->push_back(center);

  // circle detection
  pcl::PointCloud<Velodyne::Point>::Ptr cloud_f(
      new pcl::PointCloud<Velodyne::Point>),
      centroid_candidates(new pcl::PointCloud<Velodyne::Point>);
  for (pcl::PointCloud<Velodyne::Point>::iterator pt =
           initial_centers->points.begin();
       pt < initial_centers->points.end(); ++pt) {
    int initial_x = int(pt->y * 200) - int(min_pt_x * 200);
    int initial_y = int(pt->z * 200) - int(min_pt_y * 200);
    int min_cnt = 9999;
    int cnt_cnt = 0;
    Velodyne::Point refined_center;
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
    centroid_candidates->push_back(refined_center);
  }
  std::cout << centroid_candidates->size();
  for (size_t i = 0; i < centroid_candidates->size(); i++) {
    Velodyne::Point point = centroid_candidates->points[i];
    lidar_points_.push_back(Point3D(point.x, point.y, point.z));
  }

  return true;
}

} // lidarcalib
