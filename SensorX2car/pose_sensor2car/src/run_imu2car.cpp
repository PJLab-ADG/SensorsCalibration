#include <iostream>
#include <time.h>
#include <iterator>
#include <stdio.h>
#include <string>
#include <vector>
#include <dirent.h>
#include <chrono>

#include "yawCalib.h"

const char usage[] = {
    " ./bin/run_imu2car <file_path> <output_dir> <start_frame>(optional) <end_frame>(optional)\n"
    "PARAMS:\n"
    "  file_path: path to enu pose file or utm pose file \n"
    "  output_dir: folder to save output files.\n"
    "  start_frame, end_frame: data range. \n"
    "EXAMPLE:\n"
    "  ./bin/run_imu2car ./data/novatel_enu.csv ./output/ \n"
    "  ./bin/run_imu2car ./data/novatel_enu.csv ./output/ 0 20000\n"};

int main(int argc, char **argv)
{
    
    if (argc != 3 && argc != 4 && argc != 5)
    {
        std::cerr << "Usage:" << usage;
        return 1;
    }
    std::string file_path = argv[1];
    std::string output_dir = argv[2];
    int start_frame = 0;
    int end_frame = INT_MAX;
    if (argc == 4)
    {
        start_frame = atoi(argv[3]);
    }
    if (argc == 5)
    {
        start_frame = atoi(argv[3]);
        end_frame = atoi(argv[4]);
    }

    YawCalib calibrator(output_dir);
    calibrator.LoadData(file_path, start_frame, end_frame);

    double yaw = 0;
    if (calibrator.Calibrate())
    {
        yaw = calibrator.GetFinalYaw();
        std::cout << "yaw = " << rad2deg(yaw) << " degree" << std::endl;
    }
    else{
        std::cout << "No valid data for calibrating yaw." << std::endl;
        exit(1);
    }
}
