/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include "DataReader.hpp"
#include "camera_feature.hpp"
#include "lidar_feature.hpp"
#include "optimizer.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
int main(int argc, char **argv) {

  if (argc != 5) {
    cout << "Usage: ./run_lidar2camera <mask_path> <pcd_path> <intrinsic_json> "
            "<extrinsic_json>"
            "\nexample:\n\t"
            "./bin/run_lidar2camera data/mask.jpg data/calib.pcd "
            "data/center_camera-intrinsic.json "
            "data/top_center_lidar-to-center_camera-extrinsic.json"
         << endl;
    return 0;
  }

  string image_path = argv[1];
  string pcd_path = argv[2];
  string intrinsic_json_path = argv[3];
  string extrinsic_json_path = argv[4];

  // readParameters(config_file, car_config_path);
  Eigen::Matrix4d extrinsic;
  Eigen::Matrix3d intrinsic;
  cv::Mat distortion;

  LoadIntrinsic(intrinsic_json_path, intrinsic, distortion);
  LoadExtrinsic(extrinsic_json_path, extrinsic);

  LidarFeature lidar_feature(pcd_path, intrinsic, extrinsic);
  pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  cv::Mat distance_img =
      CameraFeature::undistortion(image_path, intrinsic, distortion);
  lidar_feature.extract_feature(&distance_img, register_cloud);

  // optimizing
  Optimizer optimizer(&intrinsic, &extrinsic);
  optimizer.Calibrate(&distance_img, register_cloud);
  return 0;
}