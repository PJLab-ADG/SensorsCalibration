/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Liu Zhuochun <liuzhuochun@senseauto.com>
 */

#ifndef CALIB_LIDAR2CAR_HPP_
#define CALIB_LIDAR2CAR_HPP_

#include <stdio.h>
#include <memory>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>

#include "common/logging.hpp"
#include "common/common.hpp"
#include "lidar_car_calibration.hpp"

namespace autoCalib {
namespace calibration {

struct LidarCalibParam {
    double start_sec = 20;
    double max_sec = 500;

    int frame_gap = 3;

    double min_velocity = 5;  // 3m/s
    double max_velocity = 25;  // 25m/s
    // ve_diff_range_ = 1 - cos(v_diff_angle_deg)
    // 0.015 = 1 - cos(10 degree)
    double v_diff_angle_deg = 3;

    bool applyVegoScale = true; 
};

class LidarToCar {
public:
    LidarToCar();
    
    // rely on gnss2carcenter!
    // use vehicle info??
    void calib(const std::string &lidar_dir,
               const std::string &novatel_pose,
               const std::string &config_dir,
               // const Eigen::Vector3d &measured_trans_lidar2car,
               const std::string &output_dir,
               const LidarCalibParam &param);

private:
    void LoadCarPose(const std::string& novatel_pos_path,
                     const Eigen::Matrix4d& gnss2car,
                     const double& start_time,
                     const double& end_time,
                     std::vector<double>& timestamps,
                     std::vector<Eigen::Matrix4d>& car_poses);
    
    void LoadPointCloud(const std::string &point_cloud_path,
                        PointCloud::Ptr pcd); 

    std::unique_ptr<LidarCarCalibrator> calibrator_;
};

} // namespace calibration
} // namespace autoCalib


#endif // CALIB_LIDAR2CAR_HPP_