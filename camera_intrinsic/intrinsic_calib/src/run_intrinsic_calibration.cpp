/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Liu Zhuochun <liuzhuochun@sensetime.com>
 */

#include <iostream>
#include <dirent.h>

#include "IntrinsicCalibration.hpp"
#include <opencv2/opencv.hpp>

const char usage[] = 
    "\t./bin/run_intrinsic_calibration <calibration_image_dir> \n"
    "example:\n\t"
    "./bin/run_intrinsic_calibration ./data/\n";

int main(int argc, char** argv){
    if (argc < 2){
        std::cout << argv[0] << usage;
        return 1;
    }
    
    std::string input_image_path = argv[1];
    cv::Mat image = cv::imread(input_image_path.c_str(), 0);
    
    IntrinsicCalibration calibrator;
    calibrator.Calibrate(input_image_path);
    return 0;
}