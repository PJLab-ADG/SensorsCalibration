/*
 * Copyright (C) 2021-2021 by SenseTime Group Limited. All rights reserved.
 * Liu Zhuochun <liuzhuochun@senseauto.com>
 */

#ifndef CALIB_RADAR2CARCENTER_DATALOADER_HPP_
#define CALIB_RADAR2CARCENTER_DATALOADER_HPP_

#include <stdio.h>
#include <dirent.h>
#include <memory>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <fstream> 
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include "time.h"
#include <Eigen/Core>
#include "logging.hpp"
// #include "utilities.hpp"

#define NOVATEL_INSPVA_FREQ 100 // 100hz
#define DELPHI_TIMEGAP 0.5 // 500 ms/frame
#define CONTI_TIMEGAP 0.5 // 500 ms/frame
#define RAD2DEG 180 / M_PI
#define DEG2RAD M_PI / 180

namespace autoCalib {
namespace calibration {

static bool timeString2timecount(
    const std::string& time_string, double& time_count_s) {
    if (time_string.size() < 23) return false;
    timeval tv;
    struct tm stm;
    if (!strptime(time_string.substr(0, 19).c_str(), "%Y-%m-%d %H:%M:%S",
                &stm)) {
        LOGW(
            "Convert %s to tm struct failed, please check your time format!\n",
            time_string.substr(0, 19).c_str());
        return false;
    }
    std::string usStr = time_string.substr(20);
    int us;
    if (usStr.size() == 3) {
        us = stoi(usStr) * 1000;  // ms to us
    } else if (usStr.size() == 6) {
        us = stoi(usStr);
    } else {
        LOGW("Please use millisecond or microsecond time format!\n");
        return false;
    }
    time_t sec = mktime(&stm);
    tv.tv_sec = sec;
    tv.tv_usec = us;

    // time_count_us = static_cast<uint64_t>(tv.tv_sec * 1e6 + tv.tv_usec);
    time_count_s = static_cast<double>(tv.tv_sec + tv.tv_usec * 1e-6);
    return true;
}

// transfer Y-M-D-H-M-S-MS to seconds
static bool timeString2timecount_2(
    const std::string& time_string, double& time_count_s) {
    std::vector<std::string> elements;
    std::string elem;
    std::stringstream ss(time_string);
    while (getline(ss, elem, '-')) {
        elements.emplace_back(elem);
    }
    if (elements.size() != 7) return false;
    std::string new_string = elements[0] + "-" + elements[1] + "-" + elements[2] +
                             " " + elements[3] + ":" + elements[4] + ":" +
                             elements[5] + "." + elements[6];
    timeString2timecount(new_string, time_count_s);
    return true;
}

struct RadarObject {
    std::vector<double> timestamps;
    std::vector<double> track_range;
    std::vector<double> track_angle;
    std::vector<double> vi;
    std::vector<double> ve;
    // gps position read from imu
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;

    double dist(const int &idx0, const int &idx1) const {
        double dx = x[idx1] - x[idx0];
        double dy = y[idx1] - y[idx0];
        double dz = z[idx1] - z[idx0];
        return sqrt(dx*dx + dy*dy + dz*dz);
    }

    double timeGap(const int &idx0, const int &idx1) const {
        return timestamps[idx1] - timestamps[idx0];
    }
};

struct ImuFrame {
    double timestamp;
    double yaw;
    double x;
    double y;
    double z;
    double v;
};

class RadarDataLoader 
{
public:
    RadarDataLoader() {
        min_velocity_  = 3; // 3m/s = 10.8km/h
    }

    void setMinVelocity(double v) {min_velocity_ = v;}

    // start_radar_file_num start from 1
    // [start_radar_file_num-1, start_radar_file_num-1+max_radar_file_num]
    // get radar object that 
    void getRadarCalibData(const std::string &radar_file_dir,
                           const std::string &radar_type,
                           const std::string &novatel_enu,
                           std::vector<RadarObject> &radar_frames,
                           double start_sec = 20,
                           double max_sec = 600);

    double min_velocity_;

private:
    void preLoadImu(const std::string &novatel_enu,
                    std::vector<ImuFrame> &imu_frames,
                    double start_sec,
                    double max_sec);
    
    bool getStraightSegment(const std::vector<ImuFrame> &imu_frames,
                            std::vector<std::vector<int>> &segment_idxs);

    // bool getNovatelSpeed(const std::string &novatel_inspva,
    //                      std::vector<double> &timestamps,
    //                      std::vector<double> &novatel_speed);

    bool getDelphiRadarFrame(const std::string &radar_dir,
                             const std::vector<std::string> &radar_files,
                             const std::vector<ImuFrame> &imu_frames,
                             const std::vector<std::vector<int>> &segment_idxs,
                             std::vector<RadarObject> &radar_objects);


    bool getContiRadarFrame(const std::string &radar_dir,
                            const std::vector<std::string> &radar_files,
                            const std::vector<ImuFrame> &imu_frames,
                            const std::vector<std::vector<int>> &segment_idxs,
                            std::vector<RadarObject> &radar_objects);

    double interpolate(double t, double t0, double t1, double v0, double v1) 
    {
        double v = ((t-t0)*v1 + (t1-t)*v0) / (t1-t0);
        return v;
    }

    double dist_thresh_;
    double angle_thresh_;
    double length_thresh_;
};

} // namespace calibration
} // namespace autoCalib

#endif  /*CALIB_RADAR2CARCENTER_DATALOADER_HPP_*/
