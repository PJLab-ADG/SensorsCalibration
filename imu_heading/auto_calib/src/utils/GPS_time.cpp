/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */

#include "time.h"
#include <string>

#include "utils/GPS_time.h"

GPSTime::GPSTime(uint32_t gps_week, double gps_seconds) {
  double total_seconds = gps_week * SECOND_WEEK + gps_seconds - LEAP_SECOND +
                         TIME_ZONE * SECOND_HOUR + GPS_DAY_OFFSET * SECOND_DAY;
  // reserve the total second
  total_seconds_ = total_seconds;
  uint32_t utc_year = 0;
  uint32_t utc_month = 0;
  uint32_t utc_day = 0;
  uint32_t utc_hour = 0;
  uint32_t utc_minute = 0;
  uint32_t utc_second = 0;
  uint32_t utc_ms = 0;
  uint32_t utc_us = 0;
  uint32_t year_inc = 0;
  // compute year
  uint32_t leap_year;
  while (true) {
    leap_year = static_cast<uint32_t>(is_leap_year(GPS_YEAR + year_inc));
    total_seconds -= MONTH_DAY[leap_year][0] * SECOND_DAY;
    if (total_seconds < 0) {
      utc_year = GPS_YEAR + year_inc;
      total_seconds += MONTH_DAY[leap_year][0] * SECOND_DAY;
      break;
    }
    year_inc += 1;
  }
  // compute month
  uint32_t month_index = 1;
  leap_year = static_cast<uint32_t>(is_leap_year(utc_year));
  while (true) {
    total_seconds -= MONTH_DAY[leap_year][month_index] * SECOND_DAY;
    if (total_seconds < 0) {
      utc_month = month_index;
      total_seconds += MONTH_DAY[leap_year][month_index] * SECOND_DAY;
      break;
    }
    month_index += 1;
  }

  // compute day
  utc_day = static_cast<uint32_t>(total_seconds / SECOND_DAY) + 1;
  total_seconds -= (utc_day - 1) * SECOND_DAY;

  // compute hour
  utc_hour = static_cast<uint32_t>(total_seconds / SECOND_HOUR);
  total_seconds -= utc_hour * SECOND_HOUR;

  // compute minute
  utc_minute = static_cast<uint32_t>(total_seconds / 60);
  total_seconds -= utc_minute * 60;
  // compute second
  utc_second = static_cast<uint32_t>(total_seconds);
  // compute ms
  utc_ms = static_cast<uint32_t>((total_seconds - utc_second) * 1000);
  // compute us
  utc_us = static_cast<uint32_t>((total_seconds - utc_second) * 1000000);
  // format yyyy-mm-dd hh:mm:ss.ms
  char buff[1024];
  sprintf(buff, "%04d-%02d-%02d %02d:%02d:%02d.%03d", utc_year, utc_month,
          utc_day, utc_hour, utc_minute, utc_second, utc_ms);
  utc_time_ = std::string(buff);
  // format yyyy-mm-dd hh:mm:ss.us
  sprintf(buff, "%04d-%02d-%02d %02d:%02d:%02d.%06d", utc_year, utc_month,
          utc_day, utc_hour, utc_minute, utc_second, utc_us);
  utc_time_us_ = std::string(buff);
  return;
}

/*
 * Convert standart utc time string into time_count from unix time start
 * in: time string with utc format
 * out: time count from 1970-01-01 with us
 * Note: different from gps time start point 1980-01-06
 */
bool timeString2timecount(const std::string &time_string,
                          double &time_count_us) {
  if (time_string.size() < 23)
    return false;
  timeval tv;
  struct tm stm;
  if (!strptime(time_string.substr(0, 19).c_str(), "%Y-%m-%d %H:%M:%S", &stm)) {
    printf("Convert %s to tm struct failed, please check your time format!\n",
           time_string.substr(0, 19).c_str());
    return false;
  }
  std::string usStr = time_string.substr(20);
  int us;
  if (usStr.size() == 3) {
    us = stoi(usStr) * 1000; // ms to us
  } else if (usStr.size() == 6) {
    us = stoi(usStr);
  } else {
    printf("Please use millisecond or microsecond time format!\n");
    return false;
  }
  time_t sec = mktime(&stm);
  tv.tv_sec = sec;
  tv.tv_usec = us;

  time_count_us = static_cast<double>(tv.tv_sec * 1e6 + tv.tv_usec);
  return true;
}

/*
 * Convert time count(us) to standart utc format
 * in: time count from 1970-01-01 with us
 * out: time string with utc format
 * Note: different from gps time start point 1980-01-06
 */
bool timecount2timeString(double time_count_us, std::string &time_string) {
  timeval tv;
  tv.tv_sec = static_cast<__time_t>(time_count_us / 1e6);
  tv.tv_usec = static_cast<__suseconds_t>(time_count_us - tv.tv_sec * 1e6);
  unsigned int ms = tv.tv_usec / 1000;
  time_t time(tv.tv_sec);
  struct tm stm;
  localtime_r(&time, &stm);
  char buffer[128];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H-%M-%S", &stm);
  char res[128];
  snprintf(res, sizeof(res), "%s-%03d", buffer, ms);
  time_string = std::string(res);
  return true;
}
