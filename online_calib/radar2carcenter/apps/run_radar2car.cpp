#include <stdio.h>
#include <dirent.h>
#include <iostream>
#include <fstream> 
#include <sstream>
#include <vector>
#include <string>
#include "time.h"
#include "logging.hpp"

#include "radar2car.hpp"

int main(int argc, char **argv)
{

    if(argc != 5 && argc != 7)
    {
        printf("please intput: ./main radar_type radar_dir_path inspva_file output_dir "
               "start_radar_file[default=200] max_radar_file_num[default=1000]\n"
               "for example: run_radar2car delphi ./front_radar ./novatel_inspva.csv ./outputs/\n"
               "         or: run_radar2car delphi ./front_radar ./novatel_inspva.csv ./outputs/\n"
               "             200 1000\n");
        return 1;
    }

    std::string radar_type= argv[1];
    std::string radar_dir = argv[2];
    std::string inspva_file = argv[3];
    std::string output_dir = argv[4];

    if (radar_dir.rfind('/') != radar_dir.size() - 1) {
        radar_dir = radar_dir + "/";
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

    autoCalib::calibration::RadarCalibrator calibrator;
    autoCalib::calibration::RadarCalibParam param;
    if (argc == 7) {
        std::string start_file_num = argv[5];
        std::string max_file_num = argv[6];
        param.start_file_num = stoi(start_file_num);
        param.max_file_num = stoi(max_file_num);
    }

    // calibrator.calib(radar_dir, "delphi", inspva_file, output_dir, param);
    calibrator.calib(radar_dir, radar_type, inspva_file, output_dir, param);

    return 0;
}
