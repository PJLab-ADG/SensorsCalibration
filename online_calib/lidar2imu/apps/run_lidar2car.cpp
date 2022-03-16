#include <stdio.h>
#include <dirent.h>
#include <iostream>
#include <fstream> 
#include <sstream>
#include <vector>
#include <string>
#include "time.h"
#include "common/logging.hpp"

#include "lidar2car.hpp"


int main(int argc, char **argv)
{

    if(argc != 5 && argc != 7)
    {
        printf("please intput: ./main lidar_dir novatel_pose_path config_path output_dir "
               "start_second[default=10s] max_seconds[default=60s]\n"
               "for example: run_lidar2car ./parsed/top_center_lidar ./parsed/novatel-pose.txt "
               "./config/vehicle/CN-011 ./outputs/\n"
               "         or: run_lidar2car ./parsed/top_center_lidar ./parsed/novatel-pose.txt "
               "./config/vehicle/CN-011 ./outputs/ 50 100\n");
        return 1;
    }

    std::string lidar_dir= argv[1];
    std::string novatel_pose_path = argv[2];
    std::string config_path = argv[3];
    std::string output_dir = argv[4];

    if (lidar_dir.rfind('/') != lidar_dir.size() - 1) {
        lidar_dir = lidar_dir + "/";
    }
    if (config_path.rfind('/') != config_path.size() - 1) {
        config_path = config_path + "/";
    }
    // creat output dir
    if (output_dir.rfind('/') != output_dir.size() - 1) {
        output_dir = output_dir + "/";
    }
    if (opendir(output_dir.c_str()) == nullptr) {
        char command[1024];
        snprintf(command, sizeof(command), "mkdir -p %s", output_dir.c_str());
        if (system(command)) {
            printf("Create dir: %s\n", output_dir.c_str());
        }
    }

    autoCalib::calibration::LidarToCar calibrator;
    autoCalib::calibration::LidarCalibParam param;
    if (argc == 7) {
        std::string start_second = argv[5];
        std::string max_second = argv[6];
        param.start_sec = stod(start_second);
        param.max_sec = stod(max_second);
    }

    calibrator.calib(lidar_dir, novatel_pose_path, config_path, output_dir, param);

    return 0;
}
