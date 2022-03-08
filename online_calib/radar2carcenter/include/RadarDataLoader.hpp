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
#include "utilities.hpp"

#define NOVATEL_INSPVA_FREQ 100 // 100hz

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


// radar information in one frame
struct RadarFrame {
    // get from novatel inspva
    // assume direction is always forward
    // vehicle info??
    std::vector<double> ve;
    std::vector<double> vi;
    std::vector<double> times; // in second
    // timestamp is not needed?
    std::vector<double> track_angle; // in rad
    std::vector<double> track_dist;
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
    void getRadarCalibData(const std::string &radar_file_dir,
                           const std::string &radar_type,
                           const std::string &novatel_inspva,
                           std::vector<RadarFrame> &radar_frames,
                           int start_radar_file_num = 100,
                           int max_radar_file_num = 1000);


    int max_file_num_;
    int start_file_num_;
    double start_timestamp_;
    double end_timestamp_;
    double min_velocity_;

private:
    bool getNovatelSpeed(const std::string &novatel_inspva,
                         std::vector<double> &timestamps,
                         std::vector<double> &novatel_speed);

    bool getDelphiRadarFrame(const std::string &radar_dir,
                             const std::vector<std::string> &radar_files,
                             const std::vector<double> &inspva_times,
                             const std::vector<double> &inspva_speed,
                             std::vector<RadarFrame> &radar_frames);


    bool getContiRadarFrame(const std::string &radar_dir,
                               const std::vector<std::string> &radar_files,
                               const std::vector<double> &inspva_times,
                               const std::vector<double> &inspva_speed,
                               std::vector<RadarFrame> &radar_frames);

    double interpolateSpeed(double t, double t0, double t1, double v0, double v1) 
    {
        double v = ((t-t0)*v1 + (t1-t)*v0) / (t1-t0);
        return v;
    }

};

} // namespace calibration
} // namespace autoCalib

#endif  /*CALIB_RADAR2CARCENTER_DATALOADER_HPP_*/
