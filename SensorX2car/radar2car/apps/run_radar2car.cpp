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

    if(argc != 4 && argc != 6)
    {
        printf("please intput: \n"
               "./bin/run_radar2car radar_type radar_dir_path enu_file "
               "start_second[default=20s] max_seconds[default=500s]\n"
               "for example: \n"
               "./bin/run_radar2car conti ./data/conti/front_radar/ ./data/conti/novatel_enu.csv 20 500\n"
               "./bin/run_radar2car delphi ./data/delphi/front_radar ./data/delphi/novatel_enu.csv \n");
        return 1;
    }

    std::string radar_type= argv[1];
    std::string radar_dir = argv[2];
    std::string enu_file = argv[3];

    if (radar_dir.rfind('/') != radar_dir.size() - 1) {
        radar_dir = radar_dir + "/";
    }

    autoCalib::calibration::RadarCalibrator calibrator;
    autoCalib::calibration::RadarCalibParam param;
    if (argc == 6) {
        std::string start_second = argv[4];
        std::string max_second = argv[5];
        param.start_sec = stod(start_second);
        param.max_sec = stod(max_second);
    }

    calibrator.calib(radar_dir, radar_type, enu_file, param);


    return 0;
}
