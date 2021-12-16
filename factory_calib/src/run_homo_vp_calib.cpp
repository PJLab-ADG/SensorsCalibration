/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#include "Eigen/Core"
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "calibration/adas_calib.hpp"
#include "dataConvert.hpp"
#include "logging.hpp"

// 0: chessboard
// 1: vertical board
// 2: circle board
// 3: aruco marker board(4 markers)
char usage[] = {
    "[Usage]: ./bin/run_homo_vp_calib image board_type output_dir \n"
    "eg: ./bin/run_homo_vp_calib data/chessboard.jpg 0 output/ \n"};

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "usage:" << usage;
    return -1;
  }

  std::string image_path = argv[1];
  std::string board_type = argv[2];
  std::string output_dir = argv[3];
  cv::Mat image = cv::imread(image_path);
  int type = stoi(board_type);
  if (type < 0 && type > 3) {
    LOGE("board type invalid, only support 0~4");
    return -1;
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
  // convert image to gray vector
  cv::Mat gray, gray_img;
  cv::cvtColor(image, gray, CV_BGR2GRAY);
  gray.convertTo(gray_img, CV_32F);

  std::vector<std::vector<float>> gray_vec;
  cvmat2stdvec(gray_img, gray_vec);

  // not the real size, just for example
  cameracalib::CalibrationBoardTool calibrator;
  cameracalib::InputParam params;
  cameracalib::CalibrationOutputParams outputs;
  params.board_height = 1.2;
  params.distance_between_two_chessboard = 1.5;
  params.chessboard_grid_size = 0.2;
  params.camera_to_board_distance = 5;
  params.camera_height = 1.2;
  params.camera_lateral_offset = 0;
  params.camera_vertical_offset = 0;
  params.camera_focus_length_fx = 1100;
  params.camera_focus_length_fy = 1100;
  params.calibration_board_type = type;
  bool sta = false;
  if (type == 0)
    sta = calibrator.ChessBoardCalibration(gray_vec, params, output_dir,
                                           &outputs);
  else if (type == 1)
    sta = calibrator.VerticalBoardCalibration(gray_vec, params, output_dir,
                                              &outputs);
  else if (type == 2)
    sta = calibrator.CircleBoardCalibration(gray_vec, params, output_dir,
                                            &outputs);
  else if (type == 3)
    sta = calibrator.ArucoMarkerBoardCalibration(gray_vec, params, output_dir,
                                                 &outputs);
  if (sta) {
    std::cout << "\nCalibration Success!\n";
  } else {
    std::cout << "\nCalibration Failed!\n";
  }

  return 0;
}
