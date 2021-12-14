/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <string>
#include <vector>

struct HeadingResult {
  Eigen::Vector3d RPY_offset_degree;
  Eigen::Vector3d RPY_offset_rad;
};

struct LineClusterParam {
  double max_degree_gap = 0.1;
  double max_degree_range = 0.2;
  double min_line_point_num = 600;
  double min_moving_square = 0.02 * 0.02; // 0.02m/10ms

  void YawParam() {
    max_degree_gap = 0.5;
    max_degree_range = 1;
    min_line_point_num = 800;
    min_moving_square = 0.02 * 0.02;
  }

  void PitchParam() {
    max_degree_gap = 0.08;
    max_degree_range = 0.1;
    min_line_point_num = 200;
    min_moving_square = 0.015 * 0.015;
  }

  void RollParam() {
    max_degree_gap = 0.5;
    max_degree_range = 1;
    min_line_point_num = 500;
    min_moving_square = 0.015 * 0.015;
  }
};

class ImuHeading {
public:
  ImuHeading(const std::string &input_csv_path, const std::string &output_dir)
      : input_csv_path_(input_csv_path), output_dir_(output_dir){};
  ~ImuHeading() {}

  /*
   * @brief Calib Imu Heading roll,pith,yaw offset using method 1/2
   * @param method_id[in] 1:straight drive, 2:arbitrary drive
   * @return false if calibration failure
   */
  bool Calibrate(int method_id);
  bool StraightHeading();
  bool FreeHeading();

  // apply sppech to evaluate accuracy
  void SpeedEvaluate(const std::vector<int> &inliner_idx,
                     const std::vector<double> &yaw_data,
                     const std::vector<double> &ve_data,
                     const std::vector<double> &vn_data,
                     const double &yaw_offset, double &v_componet);

  void SaveJsonResult(const double &yaw_offset_rad);
  void OutputCalibResult(const double &yaw_offset_rad);

private:
  void StraightCalibrateSingle(const std::vector<double> x,
                               const std::vector<double> y,
                               const std::vector<double> gnss_heading_vec,
                               const double threshold, double &calib_offset,
                               std::vector<int> &inliner_idx);

  bool FreeCalibrateSingle(const std::vector<double> x,
                           const std::vector<double> y,
                           const std::vector<double> gnss_heading_vec,
                           const double threshold, const LineClusterParam param,
                           double &calib_offset_mean);

  void LeastSquare(const std::vector<double> x, const std::vector<double> y,
                   const std::vector<int> inliner_idx, double &a, double &b,
                   double &c);

  void GetMeasuredMean(const std::vector<double> input_array,
                       const std::vector<int> inliner_idx, double &mean);

  void GetRPYOffsetArray(const std::vector<double> x,
                         const std::vector<double> y,
                         const std::vector<double> rpy_gnss,
                         std::vector<double> &rpy_offset_array);

  void LineCluster(const std::vector<double> x, const std::vector<double> y,
                   const std::vector<double> rpy_gnss,
                   const LineClusterParam param,
                   std::vector<std::vector<int>> &lines_idx);

  bool WriteToCsv();

private:
  std::string input_csv_path_;
  std::string output_dir_;
  // method_id = 1: straight drive
  // method_id = 2: arbitrary drive
  int method_id_;
  HeadingResult calib_result_;
};
