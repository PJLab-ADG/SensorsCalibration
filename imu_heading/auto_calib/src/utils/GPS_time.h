/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */

#pragma once

#include "utils/common.h"

// gps time start point
constexpr uint32_t GPS_YEAR = 1980;
constexpr uint32_t GPS_DAY_OFFSET = 5;

bool timeString2timecount(const std::string &time_string,
                          double &time_count_us);

bool timecount2timeString(double time_count_us, std::string &time_string);

class GPSTime {
public:
  GPSTime() = default;

  GPSTime(uint32_t gps_week, double gps_second);

  inline std::string GetUTCTime() { return utc_time_; }

  inline std::string GetUTCTimeUs() { return utc_time_us_; }

  inline double GetTotalSeconds() { return total_seconds_; }

private:
  // accurate to ms for most of the data
  std::string utc_time_;
  // accurate to us for raw imu data
  std::string utc_time_us_;
  double total_seconds_;
};
