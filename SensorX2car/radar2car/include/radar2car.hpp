/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Liu Zhuochun <liuzhuochun@senseauto.com>
 */

#ifndef CALIB_RADAR2CARCENTER_RADAR2CAR_HPP_
#define CALIB_RADAR2CARCENTER_RADAR2CAR_HPP_

#include <stdio.h>
#include <memory>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "logging.hpp"
#include "RadarDataLoader.hpp"

#define RAD2DEG 180 / M_PI
#define DEG2RAD M_PI / 180

namespace autoCalib {
namespace calibration {

struct RadarCalibParam {
    double start_sec = 20;
    double max_sec = 500;

    double initial_yaw = 0; // in rad
    
    double min_velocity = 5;  // 3m/s
    double max_velocity = 25;  // 25m/s
    // ve_diff_range_ = 1 - cos(v_diff_angle_deg)
    // 0.015 = 1 - cos(10 degree)
    double v_diff_angle_deg = 3;

    bool applyVegoScale = true; 
};

class RadarCalibrator {
public:
    RadarCalibrator() {
    }
    
    double calib(const std::string &radar_file_dir,
               const std::string &radar_type, // delphi or conti 
               const std::string &novatel_enu,
               const RadarCalibParam &param,
               bool guessRoughYaw = true,
               bool savefig = true);

    // verification

private:
    bool whetherStatic(const RadarObject &object);

    bool getSingleYaw(const RadarObject &object, int i0, int i1,
                      double &yaw_deg, double &confidence);
    
    double angle_thresh_;
    int min_gap_;
    double min_velocity_;
    double max_velocity_;
    int used_frm_;

private:
    double yaw_deg;
    Eigen::Matrix3d radar2carcenter_;
    // verification
    double offset_var;
    double max_offset_diff;
};

} // namespace calibration
} // namespace autoCalib


#endif /*CALIB_RADAR2CARCENTER_RADAR2CAR_HPP_*/