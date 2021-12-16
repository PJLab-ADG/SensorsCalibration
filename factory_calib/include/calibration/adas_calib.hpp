/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#pragma once

#include <dirent.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <stack>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include <opencv2/opencv.hpp>

#define PI acos(-1)

namespace cameracalib {

struct BaseInputParam {
  float camera_height;          // camera height [in meter]
  float camera_lateral_offset;  // lateral offset of camera from car center
                                // [in meter]
  float camera_vertical_offset; // vertical offset of camera from car center
                                // [in meter]
  float camera_focus_length_fx; // camera focus length fx
  float camera_focus_length_fy; // camera focus length fy
  int img_width;                // image width
  int img_height;               // image height
};

struct CameraIntrinsic {
  float camera_focus_length_fx; // camera focus length fx
  float camera_focus_length_fy; // camera focus length fy
  int img_width;                // image width
  int img_height;               // image height
};

struct InputParam {
  float board_height;                    // board height [in meter]
  float distance_between_two_chessboard; // distance between two
                                         // chessboard [in meter]
  float chessboard_grid_size;            // size of chessboard grid [in meter]
  float camera_to_board_distance;        // distance between camera and board
                                         // [in meter]
  float camera_height;                   // camera height [in meter]
  float camera_lateral_offset;           // lateral offset of camera from car
                                         // center [in meter]
  float camera_vertical_offset;          // vertical offset of camera from car
                                         // center [in meter]
  float camera_focus_length_fx;          // camera focus length fx
  float camera_focus_length_fy;          // camera focus length fy

  int calibration_board_type;
  // 0: chessboard
  // 1: vertical board pattern 1
  // 2: vertical board pattern 2
  // 3: circle board
  // 4: apriltag board(4 markers)
};

struct CalibrationOutputParams {
  Eigen::Vector2d vanishing_pt;   // vanishing point [in pixel]
  Eigen::Matrix3d cam_homography; // camera to ground homography
  Eigen::Matrix4d cam2ground_rot; // extrinsic
};

class CalibrationBoardTool {
public:
  CalibrationBoardTool() { display_img_ = false; }

  bool setDisplay(const cv::Mat &img) {
    image_ = img;
    display_img_ = true;
  }

  /*
   * @brief perform chessboard calibration, mainly for factory and carstore
   * calibration
   * @param CalibrationInputParams [in] FactoryCalibrationInputParams or
   * CarstoreCalibrationInputParams.
   * @param std::string [in] output json path.
   * @param CalibrationOutputParams [out] specify the calibration result .
   * @return flag that indicates succeed or failed.
   */
  bool ChessBoardCalibration(const std::vector<std::vector<float>> &gray_img,
                             const InputParam &calibration_inputs,
                             const std::string &output_json_path,
                             CalibrationOutputParams *calibration_result);

  /*
  * @brief perform verticalboard calibration, mainly for factory and carstore
  * calibration
  * @param CalibrationInputParams [in] FactoryCalibrationInputParams or
  * CarstoreCalibrationInputParams.
  * @param std::string [in] output json path.
  * @param CalibrationOutputParams [out] specify the calibration result .
  * @return flag that indicates succeed or failed.
  */
  bool VerticalBoardCalibration(const std::vector<std::vector<float>> &gray_img,
                                const InputParam &calibration_inputs,
                                const std::string &output_json_path,
                                CalibrationOutputParams *calibration_result);

  /*
  * @brief perform circleboard calibration, mainly for factory and carstore
  * calibration
  * @param CalibrationInputParams [in] FactoryCalibrationInputParams or
  * CarstoreCalibrationInputParams.
  * @param std::string [in] output json path.
  * @param CalibrationOutputParams [out] specify the calibration result .
  * @return flag that indicates succeed or failed.
  */
  bool CircleBoardCalibration(const std::vector<std::vector<float>> &gray_img,
                              const InputParam &calibration_inputs,
                              const std::string &output_json_path,
                              CalibrationOutputParams *calibration_result);

  /*
  * @brief perform aruco marker calibration, mainly for factory and carstore
  * calibration
  * @param CalibrationInputParams [in] FactoryCalibrationInputParams or
  * CarstoreCalibrationInputParams.
  * @param std::string [in] output json path.
  * @param CalibrationOutputParams [out] specify the calibration result .
  * @return flag that indicates succeed or failed.
  */
  bool ArucoMarkerBoardCalibration(const std::vector<std::vector<float>> &gray_img,
                                const InputParam &calibration_inputs,
                                const std::string &output_json_path,
                                CalibrationOutputParams *calibration_result);

public:
  // utils
  bool SaveCalibrationResult(const CalibrationOutputParams &calibration_result,
                             const std::string &output_json_path);

private:
  bool SaveJsonCalibration(const CalibrationOutputParams &result,
                           const std::string &output_json_file_name);
  bool SaveJsonExtrinsic(const CalibrationOutputParams &result,
                         const std::string &output_json_file_name);
  bool SaveJsonIntrinsic(const CalibrationOutputParams &result,
                         const std::string &output_json_file_name);
  bool LoadJsonIntrinsic(const std::string &input_json_file_name,
                         CameraIntrinsic *camera_intrinsic);

private:
  CalibrationOutputParams calibration_result_;
  BaseInputParam base_input_param_;
  std::string output_dir_;

  bool display_img_;

public:
  cv::Mat image_;
};

} // namespace cameracalib
