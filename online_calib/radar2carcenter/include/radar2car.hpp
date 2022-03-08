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

namespace autoCalib {
namespace calibration {

struct RadarFrame;

struct RadarCalibParam {
    int start_file_num = 200;
    int max_file_num = 1000;

    double initial_yaw = 0; // in rad
    
    double min_velocity = 5;  // 3m/s
    double max_velocity = 25;  // 25m/s
    // ve_diff_range_ = 1 - cos(v_diff_angle_deg)
    // 0.015 = 1 - cos(10 degree)
    double v_diff_angle_deg = 5;

    bool applyVegoScale = true; 
};

class RadarCalibrator {
public:
    RadarCalibrator() {
        rough_deg_gap_ = 5;
        ve_scale_ = 1.0;
    }
    
    void calib(const std::string &radar_file_dir,
               const std::string &radar_type, // delphi or conti 
               const std::string &novatel_inspva,
               const std::string &output_dir,
               const RadarCalibParam &param,
               bool guessRoughYaw = true,
               bool savefig = true);

    // verification

private:
    // build Vego histogram and estimate Vego for each frame 
    // assumption: vego is constant in one frame
    // if vehicle speed is provided, this function will remove moving objects
    bool estimateVego(const std::vector<double> &timestamp,
                      const std::vector<double> &vi,
                      const std::vector<double> &ve,
                      double &estimated_ve,
                      std::vector<int> &stationary_indexs);
    
    // guess rough yaw if no initial yaw is provided
    // if vi = ve, vi's angle is treated as possible yaw
    bool getRoughYaw(const std::vector<RadarFrame> &radar_datas,
                     const int &frame_gap,
                     double &rough_yaw);
    bool getRoughYawSingle(const double &thresh,
                           const std::vector<double> &ve,
                           const std::vector<double> &vi,
                           const std::vector<double> &track_angle,
                           int &rough_yaw_idx,
                           double &confidence);

    // get stationary based on vehicle speed
    bool getStationary(const double &yaw,
                       const std::vector<double> &ve,
                       const std::vector<double> &vi,
                       const std::vector<double> &track_angle,
                       std::vector<int> &stationary_indexs);

    // fit stationary objects angle-vi to cosine
    // vego is different according to time stamp
    bool fittingCosine(const double &yaw,
                       const std::vector<double> &ve,
                       const std::vector<double> &vi,
                       const std::vector<double> &track_angle,
                       const std::vector<int> &selected_indexs,
                       double &angle,
                       bool applyWeight = false);
    
    double min_velocity_;
    double max_velocity_;
    double ve_diff_range_;

    int rough_deg_gap_;
    int used_frm_;

    bool applyScale_;
    double ve_scale_;

private:
    void saveStationaryTxt(const std::vector<double> &track_angle,
                           const std::vector<double> &track_dist,
                           const std::vector<double> &timestamps,
                           const std::vector<int> &selected_indexs,
                           const double &yaw);

    void saveFitlineTxt(const std::string &output_file,
                        const std::vector<double> &ve,
                        const std::vector<double> &vi,
                        const std::vector<double> &track_angle,
                        const std::vector<int> &selected_indexs,
                        const double &A,
                        const double &yaw);

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