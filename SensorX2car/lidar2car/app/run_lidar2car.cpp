#include <iostream>
#include <time.h>
#include <iterator>
#include <stdio.h>
#include <string>
#include <vector>
#include <dirent.h>
#include <chrono>

#include "fastLoam.h"
#include "yawCalib.h"
#include "rpzCalib.h"

const char usage[] = {
    " ./bin/run_lidar2car <dataset_folder> <output_dir> <start_frame>(optional) <end_frame>(optional)\n"
    "PARAMS:\n"
    "  dataset_folder: path to lidar dataset directory \n"
    "  output_dir: folder to save output files.\n"
    "  start_frame, end_frame: data range. \n"
    "EXAMPLE:\n"
    "  ./bin/run_lidar2car ./data/example ./output/ \n"
    "  ./bin/run_lidar2car ./data/example ./output/ 0 1200\n"};

void SaveExtrinsic(Eigen::Matrix4d T, std::string output_dir)
{
    std::string file_name = output_dir + '/' + "calib_result.txt";
    
    std::ofstream ofs(file_name);
    if (!ofs.is_open())
    {
        std::cerr << "open file " << file_name << " failed. Cannot write calib result." << std::endl;
        exit(1);
    }
    ofs << "Extrinsic = " << std::endl;
    ofs << "[" << T(0, 0) << "," << T(0, 1) << "," << T(0, 2) << "," << T(0, 3) << "],"
        << "[" << T(1, 0) << "," << T(1, 1) << "," << T(1, 2) << "," << T(1, 3) << "],"
        << "[" << T(2, 0) << "," << T(2, 1) << "," << T(2, 2) << "," << T(2, 3) << "],"
        << "[" << T(3, 0) << "," << T(3, 1) << "," << T(3, 2) << "," << T(3, 3) << "]" << std::endl;
    ofs.close();

    std::cout << "Calibration result was saved to file calib_result.txt" << std::endl;
}

int main(int argc, char **argv)
{
    
    if (argc != 3 && argc != 5)
    {
        std::cerr << "Usage:" << usage;
        return 1;
    }
    std::string dataset_folder = argv[1];
    std::string output_dir = argv[2];

    int start_frame = 0;
    int end_frame = INT_MAX;
    if (argc == 5)
    {
        start_frame = atoi(argv[3]);
        end_frame = atoi(argv[4]);
    }

    std::vector<Eigen::Matrix4d> lidar_pose;
    RunFastLoam(dataset_folder, output_dir, lidar_pose, start_frame, end_frame);
    std::cout << "Frame num:" << lidar_pose.size() << std::endl;

    RPCalib calibrator1(output_dir);
    calibrator1.LoadData(dataset_folder, lidar_pose, start_frame, end_frame);
    Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
    double roll = 0, pitch = 0;
    if (calibrator1.Calibrate(extrinsic))
    {
        roll = Util::GetRoll(extrinsic);
        pitch = Util::GetPitch(extrinsic);
        std::cout << "Final result:" << std::endl
                    << "roll = " << rad2deg(roll) << " degree" << std::endl
                    << "pitch = " << rad2deg(pitch) << " degree" << std::endl
                    << "z = " << extrinsic(2, 3) << " m" << std::endl;
    }
    else
    {
        std::cout << "No valid data for calibrating pitch and roll." << std::endl;
    }


    YawCalib calibrator2(output_dir);
    calibrator2.LoadData(lidar_pose);

    double yaw = 0;
    if (calibrator2.Calibrate())
    {
        yaw = calibrator2.GetFinalYaw();
        std::cout << "Final result:" << std::endl 
                  << "yaw = " << rad2deg(yaw) << " degree" << std::endl;
    }
    else{
        std::cout << "No valid data for calibrating yaw." << std::endl;
    }
            
    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    extrinsic.block<3, 3>(0, 0) = rotation;
    SaveExtrinsic(extrinsic, output_dir);
}


