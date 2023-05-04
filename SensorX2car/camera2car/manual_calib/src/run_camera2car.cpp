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

#include "intrinsic_param.hpp"
#include "projector_camera.hpp"

using namespace std;

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049
#define APPLY_COLOR_TO_LIDAR_INTENSITY // to set intensity colored or not

double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
static Eigen::Matrix3d calibration_matrix_ = Eigen::Matrix3d::Identity();
static Eigen::Matrix3d orign_calibration_matrix_ = Eigen::Matrix3d::Identity();
static Eigen::Matrix3d intrinsic_matrix_ = Eigen::Matrix3d::Identity();
static Eigen::Matrix3d orign_intrinsic_matrix_ = Eigen::Matrix3d::Identity();
std::vector<float> distortions_;
std::vector<Eigen::Matrix3d> modification_list_;
bool display_mode_ = true;

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

void CalibrationInit(Eigen::Matrix3d json_param) {
  calibration_matrix_ = json_param;
  orign_calibration_matrix_ = json_param;
  modification_list_.reserve(6);
  for (int32_t i = 0; i < 6; i++) {
    std::vector<int> transform_flag(3, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    modification_list_[i] = rot_tmp;
  }
  std::cout << "=>Calibration scale Init!\n";
}

void CalibrationScaleChange() {
  modification_list_.reserve(6);
  for (int32_t i = 0; i < 6; i++) {
    std::vector<int> transform_flag(3, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    modification_list_[i] = rot_tmp;
  }
  std::cout << "=>Calibration scale update done!\n";
}

void saveResult(const cv::Mat &calib_img, const int &frame_id) {
  std::string file_name = "calibration_" + std::to_string(frame_id) + ".txt";
  std::ofstream fCalib(file_name);
  if (!fCalib.is_open()) {
    std::cerr << "open file " << file_name << " failed." << std::endl;
    return;
  }
  fCalib << "Rotation Matrix:" << std::endl;
  fCalib << calibration_matrix_(0, 0) << " " << calibration_matrix_(0, 1) << " "
         << calibration_matrix_(0, 2) << "\n"
         << calibration_matrix_(1, 0) << " " << calibration_matrix_(1, 1) << " "
         << calibration_matrix_(1, 2) << "\n"
         << calibration_matrix_(2, 0) << " " << calibration_matrix_(2, 1) << " "
         << calibration_matrix_(2, 2) << std::endl;

  fCalib.close();

  std::string img_name = "calibimg_" + std::to_string(frame_id) + ".jpg";
  cv::imwrite(img_name, calib_img);
}

bool ManualCalibration(int key_input) {
  char table[] = {'q', 'a', 'w', 's', 'e', 'd'};
  bool real_hit = false;
  for (int32_t i = 0; i < 6; i++) {
    if (key_input == table[i]) {
      calibration_matrix_ = calibration_matrix_ * modification_list_[i];
      real_hit = true;
    }
  }
  return real_hit;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cout << "Usage: ./run_camera2car <image_path> "
            "<intrinsic_json>"
            "\nexample:\n\t"
            "./bin/run_camera2car data/0.png"
            "data/center_camera-intrinsic.json "
         << endl;
    return 0;
  }

  string camera_path = argv[1];
  string intrinsic_json = argv[2];
  
  cv::Mat img = cv::imread(camera_path);
  std::cout << intrinsic_json << std::endl;
  
  // load intrinsic
  Eigen::Matrix3d K;
  std::vector<double> dist;
  LoadIntrinsic(intrinsic_json, K, dist);
  for (size_t i = 0; i < dist.size(); i++) {
    distortions_.push_back(dist[i]);
  }

  intrinsic_matrix_ = K;
  orign_intrinsic_matrix_ = intrinsic_matrix_;
  std::cout << "intrinsic:\n"
            << K(0, 0) << " " << K(0, 1) << " " << K(0, 2) << "\n"
            << K(1, 0) << " " << K(1, 1) << " " << K(1, 2) << "\n"
            << K(2, 0) << " " << K(2, 1) << " " << K(2, 2) << "\n";
  std::cout << "dist:\n" << dist[0] << " " << dist[1] << "\n";

  Eigen::Matrix3d json_param = Eigen::Matrix3d::Identity();

  std::cout << "Loading data completed!" << endl;
  CalibrationInit(json_param);
  Projector projector;
  projector.init(img, K, dist);

  // view
  int width = img.cols;
  int height = img.rows;
  std::cout << "width:" << width << " , height:" << height << std::endl;
  pangolin::CreateWindowAndBind("camera2car player", width, height);

  glEnable(GL_DEPTH_TEST);
  // glDepthMask(GL_TRUE);
  // glDepthFunc(GL_LESS);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));

  pangolin::View &project_image = pangolin::Display("project")
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(150), 1.0,
                     -1.0 * (width - 200) / height)
          .SetLock(pangolin::LockLeft, pangolin::LockTop);

  unsigned char *imageArray = new unsigned char[3 * width * height];
  pangolin::GlTexture imageTexture(width, height, GL_RGB, false, 0, GL_RGB,
                                   GL_UNSIGNED_BYTE);

  // control panel
  pangolin::CreatePanel("cp").SetBounds(0.0, 1.0, 0.0,
                                        pangolin::Attach::Pix(150));
  pangolin::Var<bool> displayMode("cp.gird ON/OFF", true, true);
  pangolin::Var<double> degreeStep("cp.deg step", 0.3, 0, 1);

  pangolin::Var<bool> addXdegree("cp.+ x degree", false, false);
  pangolin::Var<bool> minusXdegree("cp.- x degree", false, false);
  pangolin::Var<bool> addYdegree("cp.+ y degree", false, false);
  pangolin::Var<bool> minusYdegree("cp.- y degree", false, false);
  pangolin::Var<bool> addZdegree("cp.+ z degree", false, false);
  pangolin::Var<bool> minusZdegree("cp.- z degree", false, false);

  pangolin::Var<bool> resetButton("cp.Reset", false, false);
  pangolin::Var<bool> save("cp.Save", false, false);

  std::vector<pangolin::Var<bool>> mat_calib_box;
  mat_calib_box.push_back(addXdegree);
  mat_calib_box.push_back(minusXdegree);
  mat_calib_box.push_back(addYdegree);
  mat_calib_box.push_back(minusYdegree);
  mat_calib_box.push_back(addZdegree);
  mat_calib_box.push_back(minusZdegree);

  cv::Mat current_frame = projector.project(calibration_matrix_);
  int frame_num = 0;

  std::cout << "\n=>START\n";
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (displayMode) {
      if (display_mode_ == false) {
        projector.setDisplayMode(true);
        current_frame = projector.project(calibration_matrix_);
        display_mode_ = true;
      }
    } else {
      if (display_mode_ == true) {
        projector.setDisplayMode(false);
        current_frame = projector.project(calibration_matrix_);
        display_mode_ = false;
      }
    }

    if (degreeStep.GuiChanged()) {
      cali_scale_degree_ = degreeStep.Get();
      CalibrationScaleChange();
      std::cout << "Degree calib scale changed to " << cali_scale_degree_
                << " degree\n";
    }
    
    for (int i = 0; i < 6; i++) {
      if (pangolin::Pushed(mat_calib_box[i])) {
        calibration_matrix_ = calibration_matrix_ * modification_list_[i];
        std::cout << "Changed!\n";
        current_frame = projector.project(calibration_matrix_);
      }
    }

    if (pangolin::Pushed(resetButton)) {
      calibration_matrix_ = orign_calibration_matrix_;
      current_frame = projector.project(calibration_matrix_);
      std::cout << "Reset!\n";
    }

    if (pangolin::Pushed(save)) {
      saveResult(current_frame, frame_num);
      std::cout << "\n==>Save Result " << frame_num << std::endl;
      std::cout << "Rotation Matrix:\n" << calibration_matrix_ << std::endl;
      frame_num++;
    }

    if (kbhit()) {
      int c = getchar();
      if (ManualCalibration(c)) {
        std::cout << "\nRotation Matrix:\n" << calibration_matrix_ << std::endl;
      }
      current_frame = projector.project(calibration_matrix_);
    }

    imageArray = current_frame.data;
    imageTexture.Upload(imageArray, GL_BGR, GL_UNSIGNED_BYTE);

    project_image.Activate();
    glColor3f(1.0, 1.0, 1.0);
    imageTexture.RenderToViewportFlipY();

    pangolin::FinishFrame();
    glFinish();
  }

  // delete[] imageArray;

  std::cout << "\nFinal Rotation Matrix:\n"
            << calibration_matrix_ << std::endl;

  return 0;
}
