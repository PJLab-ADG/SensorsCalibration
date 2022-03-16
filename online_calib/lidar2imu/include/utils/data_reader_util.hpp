/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Liu Zhuochun <liuzhuochun@senseauto.com>
 */

#pragma once

#include "time.h"
#include <stdio.h>
#include <dirent.h>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <fstream> 
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include "Eigen/Core"
#include "common/logging.hpp"

namespace autoCalib {
namespace calibration {

class DataReaderUtil {
public:
    DataReaderUtil() = default;
    ~DataReaderUtil() = default;

    static bool timeString2timecount(const std::string& time_string, 
                                     double& time_count_s) {
        if (time_string.size() < 23) return false;
        timeval tv;
        struct tm stm;
        if (!strptime(time_string.substr(0, 19).c_str(), "%Y-%m-%d %H:%M:%S",
                    &stm)) {
            LOGW("Convert %s to tm struct failed, please check time format!",
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
            LOGW("Please use millisecond or microsecond time format!");
            return false;
        }
        time_t sec = mktime(&stm);
        tv.tv_sec = sec;
        tv.tv_usec = us;

        time_count_s = static_cast<double>(tv.tv_sec + tv.tv_usec * 1e-6);
        return true;
    }

    // transfer Y-M-D-H-M-S-MS to seconds
    static bool timeString2timecount_2(const std::string& time_string,
                                       double& time_count_s) {
        std::vector<std::string> elements;
        std::string elem;
        std::stringstream ss(time_string);
        while (getline(ss, elem, '-')) {
            elements.emplace_back(elem);
        }
        if (elements.size() != 7) return false;
        std::string new_string = elements[0] + "-" + elements[1] + "-" + 
                                 elements[2] + " " + elements[3] + ":" + 
                                 elements[4] + ":" + elements[5] + "." + 
                                 elements[6];
        timeString2timecount(new_string, time_count_s);
        return true;
    }


    static bool getExtrinsic(const std::string &json_path, 
                             Eigen::Matrix4d &extrinsic) 
    {
        Json::Reader reader;
        Json::Value root;

        std::ifstream in(json_path, std::ios::binary);
        if(!in.is_open()){
            LOGE("Error Opening %s", json_path.c_str());
            return false;
        }

        if(reader.parse(in, root, false)) {
            auto name = root.getMemberNames();
            std::string id = *(name.begin());
            std::cout << id << std::endl;
            Json::Value data = root[id]["param"]["sensor_calib"]["data"];
            extrinsic << data[0][0].asDouble(), data[0][1].asDouble(), data[0][2].asDouble(), data[0][3].asDouble(),
                        data[1][0].asDouble(), data[1][1].asDouble(), data[1][2].asDouble(), data[1][3].asDouble(),
                        data[2][0].asDouble(), data[2][1].asDouble(), data[2][2].asDouble(), data[2][3].asDouble(),
                        data[3][0].asDouble(), data[3][1].asDouble(), data[3][2].asDouble(), data[3][3].asDouble();
        }
        in.close();
        return true;
    }    
};

}  // namespace calibration
}  // namespace autoCalib
