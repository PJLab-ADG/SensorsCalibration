/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */

#include <dirent.h>
#include <iostream>

#include "imu_heading.hpp"

const char usage[] = "\t./bin/run_imu_heading method_id <data_dir>\n"
                     "example:\n\t"
                     "./bin/run_imu_heading 1 ./data\n"
                     "Note: [method_id=1]drives in straight\n"
                     "Note: [method_id=2]free drive";

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cout << usage;
    return 1;
  }
  int method_id = std::stod(argv[1]);
  if (method_id != 1 && method_id != 2) {
    std::cerr << "invalid imu heading method_id." << std::endl;
    return 1;
  }
  std::string input_csv_dir = argv[2];
  std::string output_dir;
  /* check and create output directory of imu heading calibration */
  if (input_csv_dir.rfind('/') != input_csv_dir.size() - 1) {
    output_dir = input_csv_dir + "/" + "calibration/";
    input_csv_dir = input_csv_dir + "/" + "novatel-utm.csv";
  } else {
    output_dir = input_csv_dir + "calibration/";
    input_csv_dir = input_csv_dir + "novatel-utm.csv";
  }
  if (opendir(output_dir.c_str()) == nullptr) {
    char command[1024];
    sprintf(command, "mkdir -p %s", output_dir.c_str());
    system(command);
    printf("Create dir: %s\n", output_dir.c_str());
  }
  ImuHeading headingCalib(input_csv_dir, output_dir);
  headingCalib.Calibrate(method_id);

  return 0;
}
