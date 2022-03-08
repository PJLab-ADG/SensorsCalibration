/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Liu Zhuochun <liuzhuochun@senseauto.com>
 */

#ifndef LIDAR2CAR_LIDAR_CAR_CALIBRATION_HPP_
#define LIDAR2CAR_LIDAR_CAR_CALIBRATION_HPP_

#include <stdio.h>
#include <map>
#include <memory>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "common/logging.hpp"
#include "common/common.hpp"
// #include "utils/pc_util.hpp"
#include "registration.hpp"
#include "handeye_calibration.hpp"

namespace autoCalib {
namespace calibration {

class LidarCarCalibrator {
public:
    LidarCarCalibrator();

    void setCarPose(const std::vector<double>& timestamp,
                    const std::vector<Eigen::Matrix4d>& pos_vec) {
        car_pos_idx_ = 0;
        car_pos_idx_1 = 0;
        car_timestamp_ = timestamp;
        car_poses_ = pos_vec;
    }

    bool inputLidar(const double& timestamp,
                    PointCloud::Ptr in_cloud);
    bool inputLidarT(const double& timestamp,
                    PointCloud::Ptr cloud);
    void getTrandformation(Eigen::Matrix4d& lidar2car);
    // 1:calib position
    int calib_t = 1;

private:
    bool getCarRot(const double& timestamp, Eigen::Matrix3d& car_rot);
    bool getCarPosition(const double& timestamp,Eigen::Vector3d & car_pos);

    void maxRangeFilter(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out);

private:
    int frame_cnt_;
    // fixed parameters
    // ??????? ground check???
    const double lidar_ground_normal_check_ = 0.8;

    const double filter_min_range_ = 0.5;
    // const double filter_max_range_ = 100.0;
    const double filter_max_range_ = 50.0;

    const double gvoxel_resolution_ = 0.2;
    const double ngvoxel_resolution_ = 0.05;

    const double pitch_diff_thres_ = 5;

    const double min_move_angle_ = 0.3;
    const double min_intercept_ = 1.4;

    // prev frame data
    Eigen::Matrix3d prev_car_rot_;

    Eigen::Vector3d prev_car_pos_;

    // car pose
    int car_pos_idx_;
    int car_pos_idx_1;
    std::vector<double> car_timestamp_;
    std::vector<Eigen::Matrix4d> car_poses_;

    // calibration results
    double calib_conf_; 
    std::vector<Eigen::Matrix3d> handeye_results_;
    std::vector<Eigen::Vector3d> handeye_results_t_;
    std::vector<Eigen::Matrix4d> ground_results_;
    Eigen::Matrix3d handeye_final_r_;
    Eigen::Quaterniond ground_rot_q_;
    double lidar2car_height_;

    // calibration tools
    std::unique_ptr<HandEyeCalibration> handeye_calibrator_;
    std::unique_ptr<Registrator> registrator_;


};

}   // calibration 
}   // autoCalib

#endif // LIDAR2CAR_LIDAR_CAR_CALIBRATION_HPP_