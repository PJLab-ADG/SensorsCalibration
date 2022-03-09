/*
 * Copyright (C) 2021-2021 by SenseTime Group Limited. All rights reserved.
 * Liu Zhuochun <liuzhuochun@senseauto.com>
 */

#ifndef COMMON_IMUDATA_READER_HPP_
#define COMMON_IMUDATA_READER_HPP_

#include <stdio.h>
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
#include <opencv2/core/core.hpp>
#include "logging.hpp"

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


static bool readIMU(const std::string &file_path, 
    std::vector<double> &imu_t_vec,
    std::vector<Eigen::Vector3d> &linearAcceleration_vec,
    std::vector<Eigen::Vector3d> &angularVelocity_vec) 
{
    std::ifstream infile(file_path);
    std::string line;
    int cnt = 0;
    if (!infile) {
        LOGE("Open imu file failed.\n");
        return false;
    }
    while (getline(infile, line)) {
        // if (cnt < 3e4) {cnt ++;continue;}
        // if (cnt > 1e4) break;
        std::stringstream ss(line);
        std::vector<std::string> elements;
        std::string elem;
        while (getline(ss, elem, ',')) {    
            elements.emplace_back(elem);
        }
        if (elements.size() != 10) {
            LOGW("num of line elements error! skip this line.\n");
            continue;
        }
        // skip the first line
        if (elements[0] == "timestamp") {
            continue;
        }
        double timestamp;
        if(!timeString2timecount(elements[0], timestamp)) continue;
        imu_t_vec.emplace_back(timestamp);

        Eigen::Vector3d omega(stod(elements[9]), stod(elements[8]), stod(elements[7]));
        Eigen::Vector3d alpha(stod(elements[6]), stod(elements[5]), stod(elements[4]));
        linearAcceleration_vec.emplace_back(alpha);
        angularVelocity_vec.emplace_back(omega);
        cnt ++;
    }

    LOGI("Read %d imu datas.", cnt);
    return true;
}

    // read dataset from video and timestamp file
static bool readCamera(const std::string &video_path,
            const std::string &file_path,
            std::vector<double> &camera_t_vec,
            std::vector<cv::Mat> &img_vec) 
{
    // read timestamp txt file
    std::ifstream infile(file_path);
    if (!infile) {
        LOGE("%s timestamp file open failed\n", file_path);
        return false;
    }
    std::string line;
    // std::vector<uint64_t> times;
    std::vector<double> times;
    size_t idx = 0;

    while (getline(infile, line)) {
        // if (idx < 3e3) {idx ++;continue;}
        // if (idx > 1e3) break;
        std::stringstream ss(line);
        std::vector<std::string> elements;
        std::string elem;
        while (getline(ss, elem, ',')) {
            elements.emplace_back(elem);
        }
        camera_t_vec.emplace_back(stod(elements[1]) * 1e-9);
        idx ++;
    }
    infile.close();
    // load images from video
    cv::Mat img;
    cv::VideoCapture cap(video_path);
    idx = 0;
    while (cap.read(img)) {
        // if (idx < 3e3) {idx ++;continue;}
        // if (idx > 1e3) break;
        cv::cvtColor(img, img, CV_BGR2GRAY);
        img_vec.emplace_back(img);
        idx++;
    }
    LOGI("Read %d image datas.", cnt);
    return true;
}

// read dataset from video and timestamp file
static bool readCamera(const std::string &file_path,
                       std::vector<double> &camera_t_vec) 
{
    // read timestamp txt file
    std::ifstream infile(file_path);
    if (!infile) {
        LOGE("%s timestamp file open failed\n", file_path);
        return false;
    }
    std::string line;
    size_t idx = 0;

    while (getline(infile, line)) {
        std::stringstream ss(line);
        std::vector<std::string> elements;
        std::string elem;
        while (getline(ss, elem, ',')) {
            elements.emplace_back(elem);
        }
        camera_t_vec.emplace_back(stod(elements[1]) * 1e-9);
        idx ++;
    }
    infile.close();
    LOGI("Read %d image datas.", cnt);
    return true;
}


// read dataset from video and timestamp file
static bool loadCamera2ImuEX(const std::string &file_path,
                       std::vector<double> &camera_t_vec) 
{
    // read timestamp txt file
    std::ifstream infile(file_path);
    if (!infile) {
        LOGE("%s timestamp file open failed\n", file_path);
        return false;
    }
    std::string line;
    size_t idx = 0;

    while (getline(infile, line)) {
        std::stringstream ss(line);
        std::vector<std::string> elements;
        std::string elem;
        while (getline(ss, elem, ',')) {
            elements.emplace_back(elem);
        }
        camera_t_vec.emplace_back(stod(elements[1]) * 1e-9);
        idx ++;
    }
    infile.close();
    LOGI("Read %d image datas.", cnt);
    return true;
}

#endif  //  COMMON_IMUDATA_READER_HPP_
