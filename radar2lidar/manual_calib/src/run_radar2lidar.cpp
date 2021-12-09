/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Core>
#include <iostream>
#include <string>

#include "extrinsic_param.hpp"
// #include "projector.hpp"

using namespace std;

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049
#define MAX_RADAR_TIME_GAP 15 * 1e6
// #define APPLY_COLOR_TO_LIDAR_INTENSITY  // to set intensity colored or not

pangolin::GlBuffer *vertexBuffer_;
pangolin::GlBuffer *colorBuffer_;

double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
static Eigen::Matrix4d calibration_matrix_ = Eigen::Matrix4d::Identity();
static Eigen::Matrix4d orign_calibration_matrix_ = Eigen::Matrix4d::Identity();
std::vector<Eigen::Matrix4d> modification_list_;
bool display_mode_ = false;
int radar_line_size_ = 4;

struct RGB {
  unsigned char r;
  unsigned char g;
  unsigned char b;
};

bool kbhit() {
  termios term;
  tcgetattr(0, &term);
  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);
  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  tcsetattr(0, TCSANOW, &term);
  return byteswaiting > 0;
}

void CalibrationInit(Eigen::Matrix4d json_param) {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  calibration_matrix_ = json_param;
  orign_calibration_matrix_ = json_param;
  modification_list_.reserve(12);
  for (int32_t i = 0; i < 12; i++) {
    std::vector<int> transform_flag(6, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    tmp.block(0, 0, 3, 3) = rot_tmp;
    tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
    tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
    tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
    modification_list_[i] = tmp;
  }
  std::cout << "=>Calibration scale Init!\n";
}

void CalibrationScaleChange() {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  modification_list_.reserve(12);
  for (int32_t i = 0; i < 12; i++) {
    std::vector<int> transform_flag(6, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    tmp.block(0, 0, 3, 3) = rot_tmp;
    tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
    tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
    tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
    modification_list_[i] = tmp;
  }
  std::cout << "=>Calibration scale update done!\n";
}

void saveResult(const int &frame_id) {
  std::string file_name =
      "radar2lidar_extrinsic_" + std::to_string(frame_id) + ".txt";
  std::ofstream fCalib(file_name);
  if (!fCalib.is_open()) {
    std::cerr << "open file " << file_name << " failed." << std::endl;
    return;
  }
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "R:\n"
         << calibration_matrix_(0, 0) << " " << calibration_matrix_(0, 1) << " "
         << calibration_matrix_(0, 2) << "\n"
         << calibration_matrix_(1, 0) << " " << calibration_matrix_(1, 1) << " "
         << calibration_matrix_(1, 2) << "\n"
         << calibration_matrix_(2, 0) << " " << calibration_matrix_(2, 1) << " "
         << calibration_matrix_(2, 2) << std::endl;
  fCalib << "t: " << calibration_matrix_(0, 3) << " "
         << calibration_matrix_(1, 3) << " " << calibration_matrix_(2, 3)
         << std::endl;

  fCalib << "************* json format *************" << std::endl;
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "[" << calibration_matrix_(0, 0) << "," << calibration_matrix_(0, 1)
         << "," << calibration_matrix_(0, 2) << "," << calibration_matrix_(0, 3)
         << "],"
         << "[" << calibration_matrix_(1, 0) << "," << calibration_matrix_(1, 1)
         << "," << calibration_matrix_(1, 2) << "," << calibration_matrix_(1, 3)
         << "],"
         << "[" << calibration_matrix_(2, 0) << "," << calibration_matrix_(2, 1)
         << "," << calibration_matrix_(2, 2) << "," << calibration_matrix_(2, 3)
         << "],"
         << "[" << calibration_matrix_(3, 0) << "," << calibration_matrix_(3, 1)
         << "," << calibration_matrix_(3, 2) << "," << calibration_matrix_(3, 3)
         << "]" << std::endl;
  fCalib.close();
}

bool ManualCalibration(int key_input) {
  char table[] = {'q', 'a', 'w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h'};
  bool real_hit = false;
  for (int32_t i = 0; i < 12; i++) {
    if (key_input == table[i]) {
      calibration_matrix_ = calibration_matrix_ * modification_list_[i];
      real_hit = true;
    }
  }
  return real_hit;
}

bool ReadRadarPoint(const std::string &radar_csv_path,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_points,
                    const int &radar_type) {
  // clear buffer
  radar_points->points.clear();
  // load radar data
  std::ifstream file(radar_csv_path);
  if (!file.is_open()) {
    std::cout << "ERROR--->>> cannot open: " << radar_csv_path << std::endl;
    return false;
  }
  std::string line;
  getline(file, line);
  // conti radar: the input point is the x and y coordinate
  if (radar_type == 1) {
    bool whether_first = true;
    std::string first_time_str;
    while (getline(file, line)) {
      std::stringstream ss(line);
      std::string str;
      std::string time_str;
      std::string position_x_str;
      std::string position_y_str;
      int index = 0;
      while (getline(ss, str, ',')) {
        if (index == 0) {
          time_str = str;
          if (whether_first) {
            first_time_str = str;
            whether_first = false;
          } else {
            long long gap = std::stoll(time_str) - std::stoll(first_time_str);
            if (gap > MAX_RADAR_TIME_GAP) {
              std::cout << "radar point size: " << radar_points->points.size()
                        << std::endl;
              return true;
            }
          }
        }
        if (index == 4) {
          position_x_str = str;
        } else if (index == 5) {
          position_y_str = str;
        }
        index++;
      }

      pcl::PointXYZ radar_point;
      radar_point.x = std::atof(position_x_str.c_str());
      radar_point.y = std::atof(position_y_str.c_str());
      radar_point.z = 0;
      if (std::abs(radar_point.x) < 1e-6 || std::abs(radar_point.y) < 1e-6) {
        continue;
      }
      radar_points->points.push_back(radar_point);
    }
  }
  // delphi: the input point is the rho, phi
  else {
    bool whether_first = true;
    std::string first_time_str;
    while (getline(file, line)) {
      std::stringstream ss(line);
      std::string str;
      std::string time_str;
      std::string rad_str;
      std::string range_str;
      int index = 0;
      while (getline(ss, str, ',')) {
        if (index == 0) {
          time_str = str;
          if (whether_first) {
            first_time_str = str;
            whether_first = false;
          } else {
            long long gap = std::stoll(time_str) - std::stoll(first_time_str);
            if (gap > MAX_RADAR_TIME_GAP) {
              std::cout << "radar point size: " << radar_points->points.size()
                        << std::endl;
              return true;
            }
          }
        }
        if (index == 7) {
          rad_str = str;
        } else if (index == 8) {
          range_str = str;
        }
        index++;
      }

      pcl::PointXYZ radar_point;
      radar_point.x =
          std::atof(range_str.c_str()) * cos(std::atof(rad_str.c_str()));
      radar_point.y =
          std::atof(range_str.c_str()) * sin(std::atof(rad_str.c_str()));
      radar_point.z = 0;
      if (std::abs(radar_point.x) < 1e-6 || std::abs(radar_point.y) < 1e-6) {
        continue;
      }
      radar_points->points.push_back(radar_point);
    }
  }
  return true;
}

RGB GreyToColorMix(int val) {
  int r, g, b;
  if (val < 128) {
    r = 0;
  } else if (val < 192) {
    r = 255 / 64 * (val - 128);
  } else {
    r = 255;
  }
  if (val < 64) {
    g = 255 / 64 * val;
  } else if (val < 192) {
    g = 255;
  } else {
    g = -255 / 63 * (val - 192) + 255;
  }
  if (val < 64) {
    b = 255;
  } else if (val < 128) {
    b = -255 / 63 * (val - 192) + 255;
  } else {
    b = 0;
  }
  RGB rgb;
  rgb.b = b;
  rgb.g = g;
  rgb.r = r;
  return rgb;
}

void ProcessSingleFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudLidar,
                        const bool &diaplay_mode) {
  if (vertexBuffer_ != nullptr)
    delete (vertexBuffer_);
  if (colorBuffer_ != nullptr)
    delete (colorBuffer_);
  int pointsNum = cloudLidar->points.size();
  std::cout << "lidar point size: " << pointsNum << std::endl;
  pangolin::GlBuffer *vertexbuffer = new pangolin::GlBuffer(
      pangolin::GlArrayBuffer, pointsNum, GL_FLOAT, 3, GL_DYNAMIC_DRAW);
  pangolin::GlBuffer *colorbuffer = new pangolin::GlBuffer(
      pangolin::GlArrayBuffer, pointsNum, GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW);

  float *dataUpdate = new float[pointsNum * 3];
  unsigned char *colorUpdate = new unsigned char[pointsNum * 3];
  for (int ipt = 0; ipt < pointsNum; ipt++) {
    Eigen::Vector4d pointPos(cloudLidar->points[ipt].x,
                             cloudLidar->points[ipt].y,
                             cloudLidar->points[ipt].z, 1.0);
    dataUpdate[ipt * 3 + 0] = pointPos.x();
    dataUpdate[ipt * 3 + 1] = pointPos.y();
    dataUpdate[ipt * 3 + 2] = pointPos.z();

    if (diaplay_mode) {
      RGB colorFake = GreyToColorMix(cloudLidar->points[ipt].intensity);
      colorUpdate[ipt * 3 + 0] = static_cast<unsigned char>(colorFake.r);
      colorUpdate[ipt * 3 + 1] = static_cast<unsigned char>(colorFake.g);
      colorUpdate[ipt * 3 + 2] = static_cast<unsigned char>(colorFake.b);
    } else {
      for (int k = 0; k < 3; k++) {
        colorUpdate[ipt * 3 + k] =
            static_cast<unsigned char>(cloudLidar->points[ipt].intensity);
      }
    }
  }

  (vertexbuffer)->Upload(dataUpdate, sizeof(float) * 3 * pointsNum, 0);
  (colorbuffer)->Upload(colorUpdate, sizeof(unsigned char) * 3 * pointsNum, 0);

  vertexBuffer_ = vertexbuffer;
  colorBuffer_ = colorbuffer;
  std::cout << "Process lidar frame!\n";
}

int main(int argc, char **argv) {
  if (argc != 5 && argc != 4) {
    cout << "Usage: ./run_radar2lidar <pcd_path> <radar_file_path> <extrinsic_json> "
              "\nexample:\n\t"
              "./bin/run_radar2lidar data/lidar.pcd data/front_radar.csv data/front_radar-to-top_center_lidar-extrinsic.json"
            << endl;
    return 0;
  }

  string lidar_path = argv[1];
  string radar_csv_path = argv[2];
  string extrinsic_json = argv[3];
  int radar_type = 1;
  if (argc == 5)
    radar_type = std::atoi(argv[4]);
  // load lidar points
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>); // 创建点云（指针）
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_path, *cloud) ==
      -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
  {
    PCL_ERROR("Couldn't read lidar file \n");
    return (-1);
  }
  pcl::PointCloud<pcl::PointXYZI> pcd = *cloud;
  // load radar points
  pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud(
      new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）
  if (!ReadRadarPoint(radar_csv_path, radar_cloud, radar_type)) {
    std::cerr << "[ERROR]Couldn't read radar file.\n";
    return (-1);
  }
  pcl::PointCloud<pcl::PointXYZ> radar_pcd = *radar_cloud;
  // load extrinsic
  Eigen::Matrix4d json_param;
  LoadExtrinsic(extrinsic_json, json_param);
  std::cout << "radar to lidar extrinsic:\n" << json_param << std::endl;

  cout << "Loading data completed!" << endl;
  CalibrationInit(json_param);
  const int width = 1920, height = 1280;
  pangolin::CreateWindowAndBind("radar2lidar player", width, height);

  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_LESS);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(-100, 0, 0, 0, 0, 0, 0.707, 0.0, 0.707));
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(150),
                                         1.0, -1.0 * width / height)
                              .SetHandler(new pangolin::Handler3D(s_cam));
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  pangolin::OpenGlMatrix Twc; // camera to world
  Twc.SetIdentity();

  // control panel
  pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(30), 1.0, 0.0,
                                        pangolin::Attach::Pix(150));
  pangolin::Var<bool> displayMode("cp.Intensity Color", display_mode_,
                                  true); // logscale
  pangolin::Var<double> degreeStep("cp.deg step", cali_scale_degree_, 0,
                                   1); // logscale
  pangolin::Var<double> tStep("cp.t step(cm)", 6, 0, 15);
  pangolin::Var<int> pointSize("cp.radar line size", radar_line_size_, 1, 10);

  pangolin::Var<bool> addXdegree("cp.+ x degree", false, false);
  pangolin::Var<bool> minusXdegree("cp.- x degree", false, false);
  pangolin::Var<bool> addYdegree("cp.+ y degree", false, false);
  pangolin::Var<bool> minusYdegree("cp.- y degree", false, false);
  pangolin::Var<bool> addZdegree("cp.+ z degree", false, false);
  pangolin::Var<bool> minusZdegree("cp.- z degree", false, false);
  pangolin::Var<bool> addXtrans("cp.+ x trans", false, false);
  pangolin::Var<bool> minusXtrans("cp.- x trans", false, false);
  pangolin::Var<bool> addYtrans("cp.+ y trans", false, false);
  pangolin::Var<bool> minusYtrans("cp.- y trans", false, false);
  pangolin::Var<bool> addZtrans("cp.+ z trans", false, false);
  pangolin::Var<bool> minusZtrans("cp.- z trans", false, false);

  pangolin::Var<bool> resetButton("cp.Reset", false, false);
  pangolin::Var<bool> saveImg("cp.Save Image", false, false);

  std::vector<pangolin::Var<bool>> mat_calib_box;
  mat_calib_box.push_back(addXdegree);
  mat_calib_box.push_back(minusXdegree);
  mat_calib_box.push_back(addYdegree);
  mat_calib_box.push_back(minusYdegree);
  mat_calib_box.push_back(addZdegree);
  mat_calib_box.push_back(minusZdegree);
  mat_calib_box.push_back(addXtrans);
  mat_calib_box.push_back(minusXtrans);
  mat_calib_box.push_back(addYtrans);
  mat_calib_box.push_back(minusYtrans);
  mat_calib_box.push_back(addZtrans);
  mat_calib_box.push_back(minusZtrans);

  int frame_num = 0;
  int lidar_point_size = cloud->points.size();
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_radar_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  ProcessSingleFrame(cloud, display_mode_);

  std::cout << "\n=>START\n";
  while (!pangolin::ShouldQuit()) {
    s_cam.Follow(Twc);
    d_cam.Activate(s_cam);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (displayMode) {
      if (display_mode_ == false) {
        display_mode_ = true;
        ProcessSingleFrame(cloud, display_mode_);
      }
    } else {
      if (display_mode_ == true) {
        display_mode_ = false;
        ProcessSingleFrame(cloud, display_mode_);
      }
    }

    if (degreeStep.GuiChanged()) {
      cali_scale_degree_ = degreeStep.Get();
      CalibrationScaleChange();
      std::cout << "Degree calib scale changed to " << cali_scale_degree_
                << " degree\n";
    }
    if (tStep.GuiChanged()) {
      cali_scale_trans_ = tStep.Get() / 100.0;
      CalibrationScaleChange();
      std::cout << "Trans calib scale changed to " << cali_scale_trans_ * 100
                << " cm\n";
    }
    if (pointSize.GuiChanged()) {
      radar_line_size_ = pointSize.Get();
      std::cout << "radar line size changed to " << radar_line_size_
                << std::endl;
    }
    for (int i = 0; i < 12; i++) {
      if (pangolin::Pushed(mat_calib_box[i])) {
        calibration_matrix_ = calibration_matrix_ * modification_list_[i];
        std::cout << "Changed!\n";
      }
    }

    if (pangolin::Pushed(resetButton)) {
      calibration_matrix_ = orign_calibration_matrix_;
      std::cout << "Reset!\n";
    }
    if (pangolin::Pushed(saveImg)) {
      saveResult(frame_num);
      std::cout << "\n==>Save Result " << frame_num << std::endl;
      Eigen::Matrix4d transform = calibration_matrix_;
      cout << "Transfromation Matrix:\n" << transform << std::endl;
      frame_num++;
    }

    if (kbhit()) {
      int c = getchar();
      if (ManualCalibration(c)) {
        Eigen::Matrix4d transform = calibration_matrix_;
        cout << "\nTransfromation Matrix:\n" << transform << std::endl;
      }
    }

    // draw points
    glDisable(GL_LIGHTING);
    glPointSize(2);
    // draw lidar points
    colorBuffer_->Bind();
    glColorPointer(colorBuffer_->count_per_element, colorBuffer_->datatype, 0,
                   0);
    glEnableClientState(GL_COLOR_ARRAY);
    vertexBuffer_->Bind();
    glVertexPointer(vertexBuffer_->count_per_element, vertexBuffer_->datatype,
                    0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, lidar_point_size);
    glDisableClientState(GL_VERTEX_ARRAY);
    vertexBuffer_->Unbind();
    glDisableClientState(GL_COLOR_ARRAY);
    colorBuffer_->Unbind();

    // draw radar lines

    transformed_radar_cloud->clear();
    pcl::transformPointCloud(*radar_cloud, *transformed_radar_cloud,
                             calibration_matrix_);
    for (int i = 0; i < radar_cloud->points.size(); i++) {
      double x = transformed_radar_cloud->points[i].x;
      double y = transformed_radar_cloud->points[i].y;
      double z = transformed_radar_cloud->points[i].z;
      glLineWidth(radar_line_size_);
      glColor3f(1.0f, 0.0f, 0.0f);
      glBegin(GL_LINES);
      glVertex3d(x, y, z);
      glVertex3d(x, y, z + 2.5);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(100);
    glFinish();
  }

  // delete[] imageArray;

  Eigen::Matrix4d transform = calibration_matrix_;
  cout << "\nFinal Transfromation Matrix:\n" << transform << std::endl;

  return 0;
}
