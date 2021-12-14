/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#pragma once

#include "math.h"
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <unistd.h>

#define SHOWDETAIL                                                             \
  printf("\tline: %d;file: %s\n\tdate: %s %s\n", __LINE__, __FILE__, __DATE__, \
         __TIME__)
#define LOGINFO(msg) printf("\033[32m[INFO] %s\033[0m\n", msg)
#define LOGWARN(msg)                                                           \
  printf("\033[33m[WARN] %s\033[0m\n", msg);                                   \
  SHOWDETAIL
#define LOGERROR(msg)                                                          \
  printf("\033[31m[ERROR] %s\033[0m\n", msg);                                  \
  SHOWDETAIL
#define LOG(type, msg) LOG##type(msg)

constexpr uint32_t MONTH_DAY[2][13] = {
    {365, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
    {366, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}};

constexpr uint32_t SECOND_WEEK = 604800;
constexpr uint32_t SECOND_DAY = 86400;
constexpr uint32_t SECOND_HOUR = 3600;
constexpr uint32_t LEAP_SECOND = 18;
constexpr uint32_t TIME_ZONE = 8;

inline std::string GotProjectDir() {
  // parse lidar config path
  char abs_path[1024];
  size_t cnt = readlink("/proc/self/exe", abs_path, 1024);
  if (cnt < 1) {
    LOG(ERROR, "got project dir failed!");
  }
  std::string project_path(abs_path);
  project_path = project_path.substr(0, project_path.rfind("senseauto"));
  return project_path;
};

class TicToc {
public:
  TicToc() { tic(); }

  void tic() { start = std::chrono::system_clock::now(); }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

inline bool is_leap_year(uint32_t year) {
  if (year % 4 == 0 && year % 100 != 0) {
    return true;
  } else if (year % 400 == 0) {
    return true;
  } else
    return false;
};

inline std::string FormatSTDTimeString(const std::string &in_str) {
  if (in_str.size() != 17 && in_str.size() != 23 && in_str.size() != 26) {
    std::cout << "invalid input " << in_str << "(" << in_str.size()
              << ") format is yyyy-mm-dd-hh-mi-ss-ms or "
              << "yyyymmddhhmissms or yyyy-mm-dd-hh-mi-ss-us" << std::endl;
    return in_str;
  }
  std::string out_str;
  // format:yyyymmddhhmissms
  if (in_str.size() == 17) {
    out_str = in_str.substr(0, 4) + "-" +  // yyyy
              in_str.substr(4, 2) + "-" +  // mm
              in_str.substr(6, 2) + " " +  // dd
              in_str.substr(8, 2) + ":" +  // hh
              in_str.substr(10, 2) + ":" + // mi
              in_str.substr(12, 2) + "." + // ss
              in_str.substr(14, 3);
  }
  // format:yyyy-mm-dd-hh-mi-ss-ms
  if (in_str.size() == 23 or in_str.size() == 26) {
    out_str = in_str;
    out_str[10] = ' ';
    out_str[13] = out_str[16] = ':';
    out_str[19] = '.';
  }
  return out_str;
};
