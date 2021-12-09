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

#include "adas_calib.hpp"
#include "dataConvert.hpp"
#include "logging.hpp"

// 0: chessboard
// 1: vertical board pattern 1
// 2: vertical board pattern 2
// 3: circle board
// 4: apriltag board(4 markers)
char usage[] = {"[Usage]: ./main image board_type output_dir display\n"
                "eg: ./run_board_detect ./data/1.jpg 0 ./output/ display\n"
                "or ./run_board_detect ./data/1.jpg 0 ./output/\n"};

int main(int argc, char **argv) {
  if (argc != 4 && argc != 5) {
    std::cerr << "usage:" << usage;
    return -1;
  }

  std::string image_path = argv[1];
  std::string board_type = argv[2];
  std::string output_dir = argv[3];
  cv::Mat image = cv::imread(image_path);
  int type = stoi(board_type);
  if (type < 0 && type > 4) {
    LOGE("board type invalid, only support 0~4");
    return -1;
  }
  if (image_path.rfind('/') != image_path.size() - 1) {
    image_path = image_path + "/";
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
  bool display = false;
  if (argc == 5 && std::string(argv[4]) == "display")
    display = true;
  std::vector<std::string> files;
  DIR *dir;
  struct dirent *ptr;
  if ((dir = opendir(image_path.c_str())) == NULL) {
    perror("Open dir error !");
    exit(1);
  }
  while ((ptr = readdir(dir)) != NULL) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
      files.push_back(ptr->d_name);
    ptr++;
  }
  if (files.size() == 0) {
    return 0;
  }
  closedir(dir);
  std::sort(files.begin(), files.end());
  std::vector<std::string> failed_files;
  int success_num = 0;
  for (size_t i = 0; i < files.size(); i++) {
    cv::Mat image = cv::imread(image_path + files[i]);
    // convert image to gray vector
    cv::Mat gray, gray_img;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    gray.convertTo(gray_img, CV_32F);

    std::vector<std::vector<float>> gray_vec;
    cvmat2stdvec(gray_img, gray_vec);

    cameracalib::CalibrationBoardTool calibrator;
    if (display)
      calibrator.setDisplay(image);

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
    std::cout << "\n==>file " << image_path + files[i] << std::endl;

    bool sta = false;
    if (type == 0)
      sta = calibrator.ChessBoardCalibration(gray_vec, params, output_dir,
                                             &outputs);
    else if (type == 1 || type == 2)
      sta = calibrator.VerticalBoardCalibration(gray_vec, params, output_dir,
                                                &outputs);
    else if (type == 3)
      sta = calibrator.CircleBoardCalibration(gray_vec, params, output_dir,
                                              &outputs);
    else if (type == 4)
      sta = calibrator.ApriltagBoardCalibration(gray_vec, params, output_dir,
                                                &outputs);
    if (sta) {
      std::cout << "\nCalibration Success!\n";
      success_num++;
    } else {
      failed_files.emplace_back(files[i]);
    }
  }

  printf("\nSuccess (%d/%d)\n", success_num, files.size());
  if (success_num != files.size()) {
    std::cout << "Failed images:\n";
    for (auto name : failed_files)
      std::cout << name << ", ";
    std::cout << std::endl;
  }
  return 0;
}
