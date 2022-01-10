/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
// #include "BALM.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <time.h>
#include <unordered_set>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/cloud_viewer.h>

#include "common/Lidar_parser_base.h"

#define LIDAR_LASER_NUM 64
#define MAX_POINT_CLOUD_NUM 400000
#define SCAN_LINE_CUT 30
#define INTENSITY_THRESHOLD 35

float cloudCurvature[MAX_POINT_CLOUD_NUM];
int cloudSortInd[MAX_POINT_CLOUD_NUM];
int cloudNeighborPicked[MAX_POINT_CLOUD_NUM];
int cloudLabel[MAX_POINT_CLOUD_NUM];

bool comp(int i, int j) { return (cloudCurvature[i] < cloudCurvature[j]); }

typedef pcl::PointXYZI PointType;

bool genPcdFeature(pcl::PointCloud<LidarPointXYZIRT>::Ptr laserCloud,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_surf,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_surf_sharp,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_corn) {
  int cloud_size = laserCloud->points.size();
  if (cloud_size >= MAX_POINT_CLOUD_NUM) {
    std::cerr << "[ERROR]too many points in pcd.\n";
    return false;
  }
  // std::cout << "Point Size: " << cloud_size << std::endl;
  // pcl::PointCloud<PointType>::Ptr laserCloud(
  //     new pcl::PointCloud<PointType>());

  std::vector<int> scan_start_index(LIDAR_LASER_NUM, 0);
  std::vector<int> scan_end_index(LIDAR_LASER_NUM, 0);
  // get laser index
  std::vector<std::vector<int>> laser_index(LIDAR_LASER_NUM,
                                            std::vector<int>());
  for (int i = 0; i < cloud_size; i++) {
    int ring = laserCloud->points[i].ring;
    if (ring < 0 || ring >= LIDAR_LASER_NUM) {
      std::cerr << "[ERROR] Wrong Ring value " << ring << " \n";
      return false;
    }
    laser_index[ring].push_back(i);
  }

  int scan_idx = 0;
  for (int i = 0; i < LIDAR_LASER_NUM; i++) {
    int cur_point_num = laser_index[i].size();
    if (cur_point_num < 11) {
      // std::cerr << "[Warning] not enough point on laser scan " << i << ".\n";
      scan_idx += cur_point_num;
      continue;
    }
    scan_start_index[i] = scan_idx + 5;
    // for(int j = 5; j < cur_point_num - 5; j ++){
    for (int j = 0; j < cur_point_num; j++) {
      int real_pidx = laser_index[i][j];
      int cur_scan_idx = scan_idx + j;
      if (j >= 5 && j < cur_point_num - 5) {
        float diffX = laserCloud->points[laser_index[i][j - 5]].x +
                      laserCloud->points[laser_index[i][j - 4]].x +
                      laserCloud->points[laser_index[i][j - 3]].x +
                      laserCloud->points[laser_index[i][j - 2]].x +
                      laserCloud->points[laser_index[i][j - 1]].x -
                      10 * laserCloud->points[real_pidx].x +
                      laserCloud->points[laser_index[i][j + 1]].x +
                      laserCloud->points[laser_index[i][j + 2]].x +
                      laserCloud->points[laser_index[i][j + 3]].x +
                      laserCloud->points[laser_index[i][j + 4]].x +
                      laserCloud->points[laser_index[i][j + 5]].x;
        float diffY = laserCloud->points[laser_index[i][j - 5]].y +
                      laserCloud->points[laser_index[i][j - 4]].y +
                      laserCloud->points[laser_index[i][j - 3]].y +
                      laserCloud->points[laser_index[i][j - 2]].y +
                      laserCloud->points[laser_index[i][j - 1]].y -
                      10 * laserCloud->points[real_pidx].y +
                      laserCloud->points[laser_index[i][j + 1]].y +
                      laserCloud->points[laser_index[i][j + 2]].y +
                      laserCloud->points[laser_index[i][j + 3]].y +
                      laserCloud->points[laser_index[i][j + 4]].y +
                      laserCloud->points[laser_index[i][j + 5]].y;
        float diffZ = laserCloud->points[laser_index[i][j - 5]].z +
                      laserCloud->points[laser_index[i][j - 4]].z +
                      laserCloud->points[laser_index[i][j - 3]].z +
                      laserCloud->points[laser_index[i][j - 2]].z +
                      laserCloud->points[laser_index[i][j - 1]].z -
                      10 * laserCloud->points[real_pidx].z +
                      laserCloud->points[laser_index[i][j + 1]].z +
                      laserCloud->points[laser_index[i][j + 2]].z +
                      laserCloud->points[laser_index[i][j + 3]].z +
                      laserCloud->points[laser_index[i][j + 4]].z +
                      laserCloud->points[laser_index[i][j + 5]].z;

        cloudCurvature[real_pidx] =
            diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudNeighborPicked[real_pidx] = 0;
        cloudLabel[real_pidx] = 0;
      }
      cloudSortInd[cur_scan_idx] = real_pidx;
    }
    scan_idx += cur_point_num;
    scan_end_index[i] = scan_idx - 6;
  }

  pcl::PointCloud<PointType> cornerPointsSharp;
  pcl::PointCloud<PointType> cornerPointsLessSharp;
  pcl::PointCloud<PointType> surfPointsFlat;
  pcl::PointCloud<PointType> surfPointsLessFlat;

  for (int i = 0; i < LIDAR_LASER_NUM; i++) {
    int valid_scan_point_num = scan_end_index[i] - scan_start_index[i];
    if (valid_scan_point_num < 6)
      continue;
    // std::cout << "valid scan " << i << " : " << valid_scan_point_num <<
    // std::endl;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(
        new pcl::PointCloud<PointType>);
    for (int j = 0; j < SCAN_LINE_CUT; j++) {
      int sp = scan_start_index[i] + valid_scan_point_num * j / SCAN_LINE_CUT;
      int ep = scan_start_index[i] +
               valid_scan_point_num * (j + 1) / SCAN_LINE_CUT - 1;
      int sp_scan = valid_scan_point_num * j / SCAN_LINE_CUT + 5;
      int ep_scan = valid_scan_point_num * (j + 1) / SCAN_LINE_CUT - 1 + 5;

      // TicToc t_tmp;
      std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp);
      // t_q_sort += t_tmp.toc();

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSortInd[k];
        if (laserCloud->points[ind].intensity < INTENSITY_THRESHOLD)
          continue;

        std::vector<int>::iterator pos_loc =
            std::find(laser_index[i].begin() + sp_scan,
                      laser_index[i].begin() + ep_scan, ind);

        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 10) { // 0.1
          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            PointType pt;
            pt.x = laserCloud->points[ind].x;
            pt.y = laserCloud->points[ind].y;
            pt.z = laserCloud->points[ind].z;
            pt.intensity = laserCloud->points[ind].intensity;
            cornerPointsSharp.push_back(pt);
            cornerPointsLessSharp.push_back(pt);
          } else if (largestPickedNum <= 20) { // 20
            cloudLabel[ind] = 1;
            PointType pt;
            pt.x = laserCloud->points[ind].x;
            pt.y = laserCloud->points[ind].y;
            pt.z = laserCloud->points[ind].z;
            pt.intensity = laserCloud->points[ind].intensity;
            cornerPointsLessSharp.push_back(pt);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;

          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[*(pos_loc + l)].x -
                          laserCloud->points[*(pos_loc + l - 1)].x;
            float diffY = laserCloud->points[*(pos_loc + l)].y -
                          laserCloud->points[*(pos_loc + l - 1)].y;
            float diffZ = laserCloud->points[*(pos_loc + l)].z -
                          laserCloud->points[*(pos_loc + l - 1)].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[*(pos_loc + l)] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[*(pos_loc + l)].x -
                          laserCloud->points[*(pos_loc + l + 1)].x;
            float diffY = laserCloud->points[*(pos_loc + l)].y -
                          laserCloud->points[*(pos_loc + l + 1)].y;
            float diffZ = laserCloud->points[*(pos_loc + l)].z -
                          laserCloud->points[*(pos_loc + l + 1)].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[*(pos_loc + l)] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSortInd[k];
        if (laserCloud->points[ind].intensity < INTENSITY_THRESHOLD)
          continue;
        std::vector<int>::iterator pos_loc =
            std::find(laser_index[i].begin() + sp_scan,
                      laser_index[i].begin() + ep_scan, ind);
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 10) {
          cloudLabel[ind] = -1;
          PointType pt;
          pt.x = laserCloud->points[ind].x;
          pt.y = laserCloud->points[ind].y;
          pt.z = laserCloud->points[ind].z;
          pt.intensity = laserCloud->points[ind].intensity;
          surfPointsFlat.push_back(pt);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[*(pos_loc + l)].x -
                          laserCloud->points[*(pos_loc + l - 1)].x;
            float diffY = laserCloud->points[*(pos_loc + l)].y -
                          laserCloud->points[*(pos_loc + l - 1)].y;
            float diffZ = laserCloud->points[*(pos_loc + l)].z -
                          laserCloud->points[*(pos_loc + l - 1)].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[*(pos_loc + l)] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[*(pos_loc + l)].x -
                          laserCloud->points[*(pos_loc + l + 1)].x;
            float diffY = laserCloud->points[*(pos_loc + l)].y -
                          laserCloud->points[*(pos_loc + l + 1)].y;
            float diffZ = laserCloud->points[*(pos_loc + l)].z -
                          laserCloud->points[*(pos_loc + l + 1)].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[*(pos_loc + l)] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        int ind = cloudSortInd[k];
        if (laserCloud->points[ind].intensity < INTENSITY_THRESHOLD)
          continue;
        if (cloudLabel[ind] <= 0) {
          PointType pt;
          pt.x = laserCloud->points[ind].x;
          pt.y = laserCloud->points[ind].y;
          pt.z = laserCloud->points[ind].z;
          pt.intensity = laserCloud->points[ind].intensity;
          surfPointsLessFlatScan->push_back(pt);
        }
      }
    }

    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    surfPointsLessFlat += surfPointsLessFlatScanDS;
    // surfPointsLessFlat += *surfPointsLessFlatScan;
  }

  // std::cout << "surfPointsFlat point size: " << surfPointsFlat.points.size()
  // << std::endl;
  // std::cout << "surfPointsLessFlat point size: " <<
  // surfPointsLessFlat.points.size() << std::endl;
  // std::cout << "cornerPointsSharp point size: " <<
  // cornerPointsSharp.points.size() << std::endl;
  // std::cout << "cornerPointsLessSharp point size: " <<
  // cornerPointsLessSharp.points.size() << std::endl;

  // pcl::PCDWriter writer;
  // writer.write("surfPointsFlat.pcd", surfPointsFlat);
  // writer.write("cornerPointsSharp.pcd", cornerPointsSharp);
  // writer.write("cornerPointsLessSharp.pcd", cornerPointsLessSharp);
  // writer.write("surfPointsLessFlat.pcd", surfPointsLessFlat);

  *pcd_surf = surfPointsLessFlat;
  *pcd_surf_sharp = surfPointsFlat;
  *pcd_corn = cornerPointsSharp;

  surfPointsFlat.clear();
  surfPointsLessFlat.clear();
  cornerPointsSharp.clear();
  cornerPointsLessSharp.clear();
  return true;
}

// int main(int argc, char **argv) {
//     pcl::PointCloud<LidarPointXYZIRT>::Ptr cloud(
//         new pcl::PointCloud<LidarPointXYZIRT>);
//     std::string lidar_file_name = "haha.pcd";
//     if (pcl::io::loadPCDFile(lidar_file_name, *cloud) < 0) {
//         std::cout << "cannot open pcd_file: " << lidar_file_name << "\n";
//         exit(1);
//     }
//     pcl::PointCloud<pcl::PointXYZI>::Ptr pl_corn(new
//     pcl::PointCloud<pcl::PointXYZI>);
//     pcl::PointCloud<pcl::PointXYZI>::Ptr pl_surf(new
//     pcl::PointCloud<pcl::PointXYZI>);
//     genPcdFeature(cloud, pl_surf, pl_corn);
//     return 0;
// }