/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#include "imu_heading.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <unistd.h>

#include "utils/GPS_time.h"
#include "utils/Ransac_fitline.h"
#include "utils/common.h"

class NovAtelEnuDatas {
public:
  NovAtelEnuDatas(){};

  bool add_element(int64_t t, double x, double y, double z, double ve,
                   double vn, double vu, double roll, double pitch,
                   double yaw) {
    timestamp_data_.push_back(t);
    x_data_.push_back(x);
    y_data_.push_back(y);
    z_data_.push_back(z);
    ve_data_.push_back(ve);
    vn_data_.push_back(vn);
    vu_data_.push_back(vu);
    roll_data_.push_back(roll);
    pitch_data_.push_back(pitch);
    yaw_data_.push_back(yaw);
    return true;
  }

  void Clear() {
    timestamp_data_.clear();
    x_data_.clear();
    y_data_.clear();
    z_data_.clear();
    ve_data_.clear();
    vn_data_.clear();
    vu_data_.clear();
    roll_data_.clear();
    pitch_data_.clear();
    yaw_data_.clear();
  }

  int Size() { return x_data_.size(); }

public:
  std::vector<int64_t> timestamp_data_; // millisecond unit
  std::vector<double> x_data_;
  std::vector<double> y_data_;
  std::vector<double> z_data_;
  std::vector<double> ve_data_;
  std::vector<double> vn_data_;
  std::vector<double> vu_data_;
  std::vector<double> roll_data_;
  std::vector<double> pitch_data_;
  std::vector<double> yaw_data_;
};

bool LoadEnuCSVDatas(const std::string rawdata,
                     NovAtelEnuDatas &novatel_enu_datas) {
  std::ifstream ifs(rawdata);
  std::string line1;
  int cnt = 0;
  if (!ifs) {
    std::cerr << "can not find the file"
              << "\n";
    return false;
  } else {
    novatel_enu_datas.Clear();
    while (getline(ifs, line1)) {
      std::stringstream ss(line1);
      std::vector<std::string> elements;
      std::string elem;
      while (getline(ss, elem, ',')) {
        elements.emplace_back(elem);
      }
      if (elements.size() != 10) {
        std::cerr << "LINE " << cnt
                  << ": num of line elements error!: " << elements.size()
                  << " skip this line\n";
        continue;
      }
      // skip the first line
      if (elements[0] == "gps_time") {
        continue;
      }

      novatel_enu_datas.add_element(
          cnt, std::stod(elements[1]), std::stod(elements[2]),
          std::stod(elements[3]), std::stod(elements[4]),
          std::stod(elements[5]), std::stod(elements[6]),
          std::stod(elements[7]), std::stod(elements[8]),
          std::stod(elements[9]));
      cnt++;
    }
  }
  return true;
}

bool ImuHeading::Calibrate(int method_id) {
  if (method_id != 1 && method_id != 2) {
    std::cerr << "invalid imu heading method_id." << std::endl;
    return false;
  }
  method_id_ = method_id;
  if (method_id_ == 1) {
    return this->StraightHeading();
  }
  if (method_id_ == 2) {
    return this->FreeHeading();
  }

  return true;
}

bool ImuHeading::StraightHeading() {
  /* get file data */
  NovAtelEnuDatas csv_datas;
  if (!LoadEnuCSVDatas(input_csv_path_, csv_datas)) {
    std::cerr << "Load NovAtel ENU CSV failed." << std::endl;
    return false;
  }
  std::vector<int> inliner_idx;

  // calibrate yaw pitch row
  std::cout << "\nYaw Calib:\n";
  double yaw_offset_degree;
  this->StraightCalibrateSingle(csv_datas.x_data_, csv_datas.y_data_,
                                csv_datas.yaw_data_, 0.15, yaw_offset_degree,
                                inliner_idx);
  double yaw_offset_rad = yaw_offset_degree / 180.0 * M_PI;

  double v_componet;
  SpeedEvaluate(inliner_idx, csv_datas.yaw_data_, csv_datas.ve_data_,
                csv_datas.vn_data_, yaw_offset_rad, v_componet);

  // std::cout << "\nPitch Calib:\n";
  // double pitch_offset_degree;
  // this->StraightCalibrateSingle(csv_datas.y_data_, csv_datas.z_data_,
  //                               csv_datas.pitch_data_, 0.15,
  //                               pitch_offset_degree,
  //                               inliner_idx);
  // double pitch_offset_rad = pitch_offset_degree / 180.0 * M_PI;

  // std::cout << "\nRoll Calib:\n";
  // double roll_offset_degree;
  // this->StraightCalibrateSingle(csv_datas.x_data_, csv_datas.z_data_,
  //                               csv_datas.roll_data_, 0.15,
  //                               roll_offset_degree,
  //                               inliner_idx);
  // double roll_offset_rad = roll_offset_degree / 180.0 * M_PI;

  // calib_result_.RPY_offset_degree(0) = roll_offset_degree;
  // calib_result_.RPY_offset_degree(1) = pitch_offset_degree;
  // calib_result_.RPY_offset_degree(2) = yaw_offset_degree;
  // calib_result_.RPY_offset_rad(0) = roll_offset_rad;
  // calib_result_.RPY_offset_rad(1) = pitch_offset_rad;
  // calib_result_.RPY_offset_rad(2) = yaw_offset_rad;

  calib_result_.RPY_offset_degree(0) = 0;
  calib_result_.RPY_offset_degree(1) = 0;
  calib_result_.RPY_offset_degree(2) = yaw_offset_degree;
  calib_result_.RPY_offset_rad(0) = 0;
  calib_result_.RPY_offset_rad(1) = 0;
  calib_result_.RPY_offset_rad(2) = yaw_offset_rad;

  this->WriteToCsv();
  // this->SaveJsonResult(yaw_offset_rad);
  this->OutputCalibResult(yaw_offset_rad);
  return true;
}

bool ImuHeading::FreeHeading() {
  /* get file data */
  NovAtelEnuDatas csv_datas;
  if (!LoadEnuCSVDatas(input_csv_path_, csv_datas)) {
    std::cerr << "Load NovAtel ENU CSV failed." << std::endl;
    return false;
  }
  LineClusterParam lineparam;

  std::cout << "\n==>Yaw Calib:\n";
  lineparam.YawParam();
  double yaw_offset_degree;
  this->FreeCalibrateSingle(csv_datas.x_data_, csv_datas.y_data_,
                            csv_datas.yaw_data_, 0.05, lineparam,
                            yaw_offset_degree);
  double yaw_offset_rad = yaw_offset_degree / 180.0 * M_PI;

  std::cout << "\n==>Pitch Calib:\n";
  lineparam.PitchParam();
  double pitch_offset_degree;
  this->FreeCalibrateSingle(csv_datas.y_data_, csv_datas.z_data_,
                            csv_datas.pitch_data_, 0.05, lineparam,
                            pitch_offset_degree);
  double pitch_offset_rad = pitch_offset_degree / 180.0 * M_PI;

  std::cout << "\n==>Roll Calib:\n";
  lineparam.RollParam();
  double roll_offset_degree;
  this->FreeCalibrateSingle(csv_datas.x_data_, csv_datas.z_data_,
                            csv_datas.roll_data_, 0.05, lineparam,
                            roll_offset_degree);
  double roll_offset_rad = roll_offset_degree / 180.0 * M_PI;

  calib_result_.RPY_offset_degree(0) = roll_offset_degree;
  calib_result_.RPY_offset_degree(1) = pitch_offset_degree;
  calib_result_.RPY_offset_degree(2) = yaw_offset_degree;
  calib_result_.RPY_offset_rad(0) = roll_offset_rad;
  calib_result_.RPY_offset_rad(1) = pitch_offset_rad;
  calib_result_.RPY_offset_rad(2) = yaw_offset_rad;

  this->WriteToCsv();

  return true;
}

void ImuHeading::StraightCalibrateSingle(
    const std::vector<double> x, const std::vector<double> y,
    const std::vector<double> gnss_heading_vec,
    const double threshold, // max distance between point and line is 15cm
    double &calib_offset, std::vector<int> &inliner_idx) {
  RansacFitLine ransac_fitliner(threshold, 15);
  double a, b, c; // ax + by + c =0
  inliner_idx.clear();
  ransac_fitliner.Estimate(x, y, a, b, c, inliner_idx);
  std::cout << "best score: " << double(inliner_idx.size()) / double(x.size())
            << std::endl;

  double gnss_heading;
  this->GetMeasuredMean(gnss_heading_vec, inliner_idx, gnss_heading);

  double adjust_a, adjust_b, adjust_c;
  this->LeastSquare(x, y, inliner_idx, adjust_a, adjust_b, adjust_c);

  double gt_degree = atan(-adjust_a / adjust_b) / M_PI * 180.0;
  // double gt_degree = atan(- a / b) / M_PI * 180.0;
  double gnss_degree = gnss_heading / M_PI * 180.0;
  if (gnss_degree >= 90 && gnss_degree < 270)
    gt_degree += 180;
  if (gnss_degree >= 270 && gnss_degree < 360)
    gt_degree += 360;
  calib_offset = gt_degree - gnss_degree;
  std::cout << "GT value(degree): " << gt_degree << std::endl;
  std::cout << "GNSS value(degree): " << gnss_degree << std::endl;
  std::cout << "Offset(degree): " << calib_offset << std::endl;
}

bool ImuHeading::FreeCalibrateSingle(
    const std::vector<double> x, const std::vector<double> y,
    const std::vector<double> gnss_heading_vec,
    const double threshold, // max distance between point and line is 15cm
    const LineClusterParam param, double &calib_offset_mean) {
  std::vector<std::vector<int>> lines_idx;
  this->LineCluster(x, y, gnss_heading_vec, param, lines_idx);
  if (lines_idx.size() <= 0) {
    std::cerr << "[LineCLuste Failed]:No straight lines." << std::endl;
    return false;
  }

  double mean_offset = 0;
  double line_point_sum = 0;
  for (int i = 0; i < lines_idx.size(); i++) {
    std::vector<double> new_x;
    std::vector<double> new_y;
    std::vector<double> new_rpy;
    for (int j = 0; j < lines_idx[i].size(); j++) {
      new_x.push_back(x[lines_idx[i][j]]);
      new_y.push_back(y[lines_idx[i][j]]);
      new_rpy.push_back(gnss_heading_vec[lines_idx[i][j]]);
    }
    // threshold = lines_idx[i].size() *
    RansacFitLine ransac_fitliner(threshold, 20);
    double a, b, c; // ax + by + c =0
    std::vector<int> inliner_idx;
    ransac_fitliner.Estimate(new_x, new_y, a, b, c, inliner_idx);
    // std::cout << "\nbest score: "
    //           << double(inliner_idx.size()) / double(new_x.size())
    //           << std::endl;

    double gnss_heading;
    this->GetMeasuredMean(new_rpy, inliner_idx, gnss_heading);
    double adjust_a, adjust_b, adjust_c;
    this->LeastSquare(new_x, new_y, inliner_idx, adjust_a, adjust_b, adjust_c);

    double gt_degree = atan(-adjust_a / adjust_b) / M_PI * 180.0;
    // double gt_degree = atan(- a / b) / M_PI * 180.0;
    double gnss_degree = gnss_heading / M_PI * 180.0;
    if (gnss_degree >= 90 && gnss_degree < 270)
      gt_degree += 180;
    if (gnss_degree >= 270 && gnss_degree < 360)
      gt_degree += 360;
    double calib_offset = gt_degree - gnss_degree;
    // std::cout << "Line point " << lines_idx[i].size() << std::endl;
    // std::cout << "GT value(degree): " << gt_degree << std::endl;
    // std::cout << "GNSS value(degree): " << gnss_degree << std::endl;
    // std::cout << "Offset(degree): " << calib_offset << std::endl;

    line_point_sum += lines_idx[i].size();
    mean_offset += lines_idx[i].size() * calib_offset;
  }
  mean_offset = mean_offset / line_point_sum;
  calib_offset_mean = mean_offset;
  std::cout << "\nMean offset: " << calib_offset_mean << std::endl;

  return true;
}

void ImuHeading::LeastSquare(const std::vector<double> x,
                             const std::vector<double> y,
                             const std::vector<int> inliner_idx, double &a,
                             double &b, double &c) {
  double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
  int inliner_num = inliner_idx.size();
  if (inliner_num <= 0 || inliner_num > x.size()) {
    std::cerr << "Line Inliner num invalid." << std::endl;
  }
  for (int i = 0; i < inliner_num; ++i) {
    t1 += x[inliner_idx[i]] * x[inliner_idx[i]];
    t2 += x[inliner_idx[i]];
    t3 += x[inliner_idx[i]] * y[inliner_idx[i]];
    t4 += y[inliner_idx[i]];
  }
  if (t1 * inliner_num == t2 * t2) {
    b = 0;
    a = 1;
    c = -x[0];
  } else {
    a = -(t3 * static_cast<double>(inliner_num) - t2 * t4) /
        (t1 * static_cast<double>(inliner_num) - t2 * t2);
    b = 1;
    c = -(t1 * t4 - t2 * t3) /
        (t1 * static_cast<double>(inliner_num) - t2 * t2);
  }
}

void ImuHeading::GetMeasuredMean(const std::vector<double> input_array,
                                 const std::vector<int> inliner_idx,
                                 double &mean) {
  mean = 0;
  for (size_t j = 0; j < inliner_idx.size(); j++) {
    mean += input_array[inliner_idx[j]];
  }
  mean /= static_cast<double>(inliner_idx.size());
}

void ImuHeading::GetRPYOffsetArray(const std::vector<double> x,
                                   const std::vector<double> y,
                                   const std::vector<double> rpy_gnss,
                                   std::vector<double> &rpy_offset_array) {
  rpy_offset_array.clear();
  std::vector<int> inliner_idx;

  int time_gap = 5;
  for (int m = 0; m < time_gap * 2 + 1; m++) {
    inliner_idx.push_back(m);
  }
  for (int i = 1000 + time_gap; i < x.size() - time_gap - 1000; i++) {
    double x_move = x[i - time_gap] - x[i + time_gap];
    double y_move = y[i - time_gap] - y[i + time_gap];
    double gt_degree = atan((y_move) / (x_move)) / M_PI * 180.0;
    double gnss_degree =
        std::accumulate(std::begin(rpy_gnss) + i - time_gap,
                        std::begin(rpy_gnss) + i + time_gap, 0.0) /
        static_cast<double>(time_gap * 2) / M_PI * 180.0;
    if (gnss_degree >= 90 && gnss_degree < 270)
      gt_degree += 180;
    if (gnss_degree >= 270 && gnss_degree < 360)
      gt_degree += 360;
    double a, b, c;
    this->LeastSquare(x, y, inliner_idx, a, b, c);
    double offset = gt_degree - gnss_degree;

    if (fabs(x_move) + fabs(y_move) > 0.02 && fabs(offset) < 3) {
      rpy_offset_array.push_back(offset);
      if (rpy_offset_array.size() % 100 == 0) {
        std::cout << i << ": " << offset << "\ty: " << y_move
                  << "\tx: " << x_move << std::endl;
        std::cout << "\t" << gt_degree << "\t" << gnss_degree << std::endl;
      }
    }

    for (int m = 0; m < time_gap * 2 + 1; m++) {
      inliner_idx[m] += 1;
    }
  }
}

void ImuHeading::LineCluster(const std::vector<double> x,
                             const std::vector<double> y,
                             const std::vector<double> rpy_gnss,
                             const LineClusterParam param,
                             std::vector<std::vector<int>> &lines_idx) {
  int idx = 0;
  double max_degree_gap = param.max_degree_gap;
  double max_degree_range = param.max_degree_range;
  double min_line_point_num = param.min_line_point_num;
  double min_moving_square = param.min_moving_square;
  // double max_moving_square = 0.02 * 0.02;
  double cur_line_rpy = -100;
  std::vector<int> cur_line_idx;

  while (idx < x.size()) {
    cur_line_rpy = rpy_gnss[idx] / M_PI * 180.0;
    double next_gnss_degree = cur_line_rpy;
    double gnss_degree = cur_line_rpy;

    while (fabs(next_gnss_degree - gnss_degree) < max_degree_gap &&
           fabs(next_gnss_degree - cur_line_rpy) < max_degree_range) {
      cur_line_idx.push_back(idx);
      if (++idx >= x.size())
        break;
      if ((x[idx] - x[idx - 1]) * (x[idx] - x[idx - 1]) +
              (y[idx] - y[idx - 1]) * (y[idx] - y[idx - 1]) <
          min_moving_square) {
        break;
      }
      // std::cout << idx << " move: " << sqrt((x[idx] - x[idx - 1]) *
      // (x[idx] - x[idx - 1])
      //     + (y[idx] - y[idx - 1]) * (y[idx] - y[idx - 1])) <<
      //     std::endl;
      // not moving
      gnss_degree = next_gnss_degree;
      next_gnss_degree = rpy_gnss[idx] / M_PI * 180.0;
      if (next_gnss_degree > 270 && cur_line_rpy < 90)
        next_gnss_degree -= 360;
    }

    // std::cout << "\tline size: " << cur_line_idx.size() << std::endl;
    if (cur_line_idx.size() >= min_line_point_num) {
      lines_idx.push_back(cur_line_idx);
    }
    cur_line_idx.clear();
  }
  std::cout << "\nFind " << lines_idx.size() << " lines: ";
  for (int num = 0; num < lines_idx.size(); num++) {
    std::cout << "\t" << lines_idx[num].size() << "[" << lines_idx[num][0]
              << " " << lines_idx[num][lines_idx[num].size() - 1] << "],";
  }
  std::cout << std::endl;
}

bool ImuHeading::WriteToCsv() {
  std::string output_file_path = output_dir_ + "imu_heading.csv";
  // TODO：
  std::ofstream out_file;
  out_file.open(output_file_path, std::fstream::out);
  out_file << "roll_offset_degree,pitch_offset_degree,yaw_offset_degree,"
              "roll_offset_rad,pitch_offset_rad,yaw_offset_rad"
           << std::endl;
  char msg[1024];
  sprintf(msg, "%lf,%lf,%lf,%lf,%lf,%lf\n", calib_result_.RPY_offset_degree(0),
          calib_result_.RPY_offset_degree(1),
          calib_result_.RPY_offset_degree(2), calib_result_.RPY_offset_rad(0),
          calib_result_.RPY_offset_rad(1), calib_result_.RPY_offset_rad(2));
  out_file << std::string(msg);
  return true;
}

void ImuHeading::SpeedEvaluate(const std::vector<int> &inliner_idx,
                               const std::vector<double> &yaw_data,
                               const std::vector<double> &ve_data,
                               const std::vector<double> &vn_data,
                               const double &yaw_offset, double &v_componet) {
  double calibrated_v_componet = 0;
  double max_calibrated_v_componet = 0;
  double min_calibrated_v_componet = 900;
  double origin_v_componet = 0;
  double max_origin_v_componet = 0;
  double min_origin_v_componet = 900;
  for (int i = 0; i < inliner_idx.size(); i++) {
    int idx = inliner_idx[i];
    double measured_yaw = yaw_data[idx];
    double calibrated_yaw = (yaw_data[idx] + yaw_offset);
    double current_calibrated = (-cos(calibrated_yaw) * vn_data[idx] +
                                 sin(calibrated_yaw) * ve_data[idx]);
    double current_origin =
        (-cos(measured_yaw) * vn_data[idx] + sin(measured_yaw) * ve_data[idx]);
    if (fabs(current_calibrated) > max_calibrated_v_componet)
      max_calibrated_v_componet = fabs(current_calibrated);
    if (fabs(current_calibrated) < min_calibrated_v_componet)
      min_calibrated_v_componet = fabs(current_calibrated);
    if (fabs(current_origin) > max_origin_v_componet)
      max_origin_v_componet = fabs(current_origin);
    if (fabs(current_origin) < min_origin_v_componet)
      min_origin_v_componet = fabs(current_origin);

    origin_v_componet += current_origin;
    calibrated_v_componet += current_calibrated;
  }
  origin_v_componet /= static_cast<double>(inliner_idx.size());
  calibrated_v_componet /= static_cast<double>(inliner_idx.size());

  std::cout << "\nCheck result:\n";
  std::cout << "origin_v_componet: " << origin_v_componet << std::endl;
  std::cout << "  max: " << max_origin_v_componet << std::endl;
  std::cout << "  min: " << min_origin_v_componet << std::endl;
  std::cout << "calibrated_v_componet: " << calibrated_v_componet << std::endl;
  std::cout << "  max: " << max_calibrated_v_componet << std::endl;
  std::cout << "  min: " << min_calibrated_v_componet << std::endl;
}

void ImuHeading::SaveJsonResult(const double &yaw_offset_rad) {
  std::vector<std::vector<double>> extrinsic_mat(4, std::vector<double>(4));
  // extrinsic_mat[0][0] = sin(-yaw_offset_rad);
  // extrinsic_mat[0][1] = cos(-yaw_offset_rad);
  // extrinsic_mat[1][0] = -cos(-yaw_offset_rad);
  // extrinsic_mat[1][1] = sin(-yaw_offset_rad);
  // extrinsic_mat[2][2] = 1;

  extrinsic_mat[1][0] = -cos(-yaw_offset_rad);
  extrinsic_mat[1][2] = -sin(-yaw_offset_rad);
  extrinsic_mat[2][0] = -sin(-yaw_offset_rad);
  extrinsic_mat[2][2] = cos(-yaw_offset_rad);
  extrinsic_mat[0][1] = 1;

  std::string output_json_path =
      output_dir_ + "gnss-to-car_center-extrinsic.json";
  Json::Value headname;
  Json::Value data;
  Json::Value param;
  Json::Value sensor_calib;
  Json::Value sensor_calib_matrix_data(Json::arrayValue);
  Json::Value sensor_calib_matrix_data1(Json::arrayValue);
  Json::Value sensor_calib_matrix_data2(Json::arrayValue);
  Json::Value sensor_calib_matrix_data3(Json::arrayValue);
  Json::Value sensor_calib_matrix_data4(Json::arrayValue);

  sensor_calib["rows"] = 4;
  sensor_calib["cols"] = 4;
  sensor_calib["type"] = 6;
  sensor_calib["continuous"] = Json::Value(true);
  sensor_calib_matrix_data1.append(extrinsic_mat[0][0]);
  sensor_calib_matrix_data1.append(extrinsic_mat[0][1]);
  sensor_calib_matrix_data1.append(extrinsic_mat[0][2]);
  sensor_calib_matrix_data1.append(extrinsic_mat[0][3]);
  sensor_calib_matrix_data2.append(extrinsic_mat[1][0]);
  sensor_calib_matrix_data2.append(extrinsic_mat[1][1]);
  sensor_calib_matrix_data2.append(extrinsic_mat[1][2]);
  sensor_calib_matrix_data2.append(extrinsic_mat[1][3]);
  sensor_calib_matrix_data3.append(extrinsic_mat[2][0]);
  sensor_calib_matrix_data3.append(extrinsic_mat[2][1]);
  sensor_calib_matrix_data3.append(extrinsic_mat[2][2]);
  sensor_calib_matrix_data3.append(extrinsic_mat[2][3]);
  sensor_calib_matrix_data4.append(extrinsic_mat[3][0]);
  sensor_calib_matrix_data4.append(extrinsic_mat[3][1]);
  sensor_calib_matrix_data4.append(extrinsic_mat[3][2]);
  sensor_calib_matrix_data4.append(extrinsic_mat[3][3]);
  sensor_calib_matrix_data.append(sensor_calib_matrix_data1);
  sensor_calib_matrix_data.append(sensor_calib_matrix_data2);
  sensor_calib_matrix_data.append(sensor_calib_matrix_data3);
  sensor_calib_matrix_data.append(sensor_calib_matrix_data4);
  sensor_calib["data"] = Json::Value(sensor_calib_matrix_data);

  param["time_lag"] = 0;
  param["sensor_calib"] = Json::Value(sensor_calib);

  data["sensor_name"] = Json::Value("gnss");
  data["target_sensor_name"] = Json::Value("car_center");
  data["device_type"] = Json::Value("relational");
  data["param_type"] = Json::Value("extrinsic");
  data["param"] = Json::Value(param);

  headname["gnss-to-car_center-extrinsic"] = Json::Value(data);
  std::cout << "Writing File " << output_json_path.c_str() << std::endl;

  Json::StreamWriterBuilder write_builder;
  std::unique_ptr<Json::StreamWriter> json_writer(
      write_builder.newStreamWriter());
  std::ofstream output;
  output.open(output_json_path);
  json_writer->write(headname, &output);
  output.close();
}

void ImuHeading::OutputCalibResult(const double &yaw_offset_rad) {
  Eigen::Matrix3d rot;
  Eigen::AngleAxisd rollAngle(-yaw_offset_rad + M_PI / 2,
                              Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yawAngle(0, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
  rot = q.matrix();
  std::cout << "Euler2RotationMatrix result is:" << std::endl;
  std::cout << "R = " << std::endl << rot << std::endl;
}