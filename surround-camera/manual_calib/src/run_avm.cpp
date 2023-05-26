/*
 * Copyright (C) 2023 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Core>
#include <iostream>
#include <string>

#include "extrinsic_param.hpp"
#include "intrinsic_param.hpp"

using namespace std;

bool cali_stiching_mode = false;
double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
int cali_frame = 0;

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

vector<Eigen::Matrix4d> calibration_matrix_(4);
vector<Eigen::Matrix4d> orign_calibration_matrix_(4);
vector<Eigen::Matrix3d> intrinsic_matrix_(4);
vector<Eigen::Matrix3d> orign_intrinsic_matrix_(4);
vector<vector<float>> distortions_(4);
vector<vector<Eigen::Matrix4d>> modification_list_(4);

void world2cam(double point2D[2], double point3D[3], Eigen::Matrix3d K,
               vector<double> D) {
  double norm = sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1]);
  double theta = atan(point3D[2] / norm);
  double t, t_i;
  double rho, x, y;
  double invnorm;
  int i;

  if (norm != 0) {
    invnorm = 1 / norm;
    t = theta;
    rho = D[0];
    t_i = 1;

    for (i = 1; i < D.size(); i++) {
      t_i *= t;
      rho += t_i * D[i];
    }

    x = point3D[0] * invnorm * rho;
    y = point3D[1] * invnorm * rho;

    point2D[0] = x * K(0, 0) + y * K(0, 1) + K(0, 2);
    point2D[1] = x * K(1, 0) + y + K(1, 2);
  } else {
    point2D[0] = K(0, 2);
    point2D[1] = K(1, 2);
  }
}

void distortPointsOcam(cv::Mat &P_GC1, cv::Mat &p_GC, Eigen::Matrix3d &K_C,
                       vector<double> &D_C) {

  double M[3];
  double m[2];
  for (int i = 0; i < P_GC1.cols; i++) {
    M[0] = P_GC1.at<cv::Vec2d>(0, i)[0];
    M[1] = P_GC1.at<cv::Vec2d>(0, i)[1];
    M[2] = -1;
    world2cam(m, M, K_C, D_C);
    p_GC.at<cv::Vec2d>(0, i)[0] = m[0];
    p_GC.at<cv::Vec2d>(0, i)[1] = m[1];
  }
}

void distortPoints(cv::Mat &P_GC1, cv::Mat &p_GC, Eigen::Matrix3d &K_C) {
  for (int i = 0; i < P_GC1.cols; i++) {
    double x = P_GC1.at<cv::Vec2d>(0, i)[0];
    double y = P_GC1.at<cv::Vec2d>(0, i)[1];

    double u = x * K_C(0, 0) + K_C(0, 2);
    double v = y * K_C(1, 1) + K_C(1, 2);

    p_GC.at<cv::Vec2d>(0, i)[0] = u;
    p_GC.at<cv::Vec2d>(0, i)[1] = v;
  }
}

cv::Mat project_on_ground(cv::Mat img, Eigen::Matrix4d T_CG,
                          Eigen::Matrix3d K_C, vector<double> D_C, cv::Mat K_G,
                          int rows, int cols, int camera_model, float height) {
  // 	cout<<"--------------------Init p_G and
  // P_G------------------------"<<endl;
  cv::Mat p_G = cv::Mat::ones(3, rows * cols, CV_64FC1);
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      p_G.at<double>(0, cols * i + j) = j;
      p_G.at<double>(1, cols * i + j) = i;
    }
  }

  cv::Mat P_G = cv::Mat::ones(4, rows * cols, CV_64FC1);
  P_G(cv::Rect(0, 0, rows * cols, 3)) = K_G.inv() * p_G * height;
  // P_G(cv::Rect(0,2,rows*cols,1)) = cv::Mat::zeros(1,rows*cols,CV_64FC1);

  cv::Mat P_GC = cv::Mat::zeros(4, rows * cols, CV_64FC1);

  cv::Mat T_CG_ =
      (cv::Mat_<double>(4, 4) << T_CG(0, 0), T_CG(0, 1), T_CG(0, 2), T_CG(0, 3),
       T_CG(1, 0), T_CG(1, 1), T_CG(1, 2), T_CG(1, 3), T_CG(2, 0), T_CG(2, 1),
       T_CG(2, 2), T_CG(2, 3), T_CG(3, 0), T_CG(3, 1), T_CG(3, 2), T_CG(3, 3));
  P_GC = T_CG_ * P_G;

  cv::Mat P_GC1 = cv::Mat::zeros(1, rows * cols, CV_64FC2);
  vector<cv::Mat> channels(2);
  cv::split(P_GC1, channels);
  channels[0] = P_GC(cv::Rect(0, 0, rows * cols, 1)) /
                P_GC(cv::Rect(0, 2, rows * cols, 1));
  channels[1] = P_GC(cv::Rect(0, 1, rows * cols, 1)) /
                P_GC(cv::Rect(0, 2, rows * cols, 1));
  cv::merge(channels, P_GC1);

  cv::Mat p_GC = cv::Mat::zeros(1, rows * cols, CV_64FC2);
  cv::Mat K_C_ =
      (cv::Mat_<double>(3, 3) << K_C(0, 0), K_C(0, 1), K_C(0, 2), K_C(1, 0),
       K_C(1, 1), K_C(1, 2), K_C(2, 0), K_C(2, 1), K_C(2, 2));
  if (camera_model == 0) {
    cv::fisheye::distortPoints(P_GC1, p_GC, K_C_, D_C);
  } else if (camera_model == 1) {
    distortPointsOcam(P_GC1, p_GC, K_C, D_C);
  } else {
    distortPoints(P_GC1, p_GC, K_C);
  }

  p_GC.reshape(rows, cols);
  cv::Mat p_GC_table = p_GC.reshape(0, rows);
  cv::Mat p_GC_table_32F;
  p_GC_table.convertTo(p_GC_table_32F, CV_32FC2);

  cv::Mat img_GC;

  cv::remap(img, img_GC, p_GC_table_32F, cv::Mat(), cv::INTER_LINEAR);

  return img_GC;
}

cv::Mat generate_surround_view(cv::Mat img_GF, cv::Mat img_GL, cv::Mat img_GB,
                               cv::Mat img_GR, int rows, int cols) {
  cv::Mat img_G(rows, cols, CV_8UC3);
  for (int i = 0; i < 400; i++) {
    for (int j = 0; j < 400; j++) {
      img_G.at<cv::Vec3b>(i, j) =
          0.5 * img_GL.at<cv::Vec3b>(i, j) + 0.5 * img_GF.at<cv::Vec3b>(i, j);
    }
    for (int j = 600; j < cols; j++) {
      img_G.at<cv::Vec3b>(i, j) =
          0.5 * img_GR.at<cv::Vec3b>(i, j) + 0.5 * img_GF.at<cv::Vec3b>(i, j);
    }
  }
  for (int i = 600; i < rows; i++) {
    for (int j = 0; j < 400; j++) {
      img_G.at<cv::Vec3b>(i, j) =
          0.5 * img_GL.at<cv::Vec3b>(i, j) + 0.5 * img_GB.at<cv::Vec3b>(i, j);
    }
    for (int j = 600; j < cols; j++) {
      img_G.at<cv::Vec3b>(i, j) =
          0.5 * img_GR.at<cv::Vec3b>(i, j) + 0.5 * img_GB.at<cv::Vec3b>(i, j);
    }
  }
  for (int i = 400; i < 600; i++) {
    for (int j = 0; j < 400; j++) {
      img_G.at<cv::Vec3b>(i, j) = img_GL.at<cv::Vec3b>(i, j);
    }
    for (int j = 600; j < cols; j++) {
      img_G.at<cv::Vec3b>(i, j) = img_GR.at<cv::Vec3b>(i, j);
    }
  }
  for (int j = 400; j < 600; j++) {
    for (int i = 0; i < 400; i++) {
      img_G.at<cv::Vec3b>(i, j) = img_GF.at<cv::Vec3b>(i, j);
    }
    for (int i = 600; i < rows; i++) {
      img_G.at<cv::Vec3b>(i, j) = img_GB.at<cv::Vec3b>(i, j);
    }
  }

  for (int i = 400; i < 600; i++) {
    for (int j = 400; j < 600; j++) {
      img_G.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
    }
  }

  return img_G;
}

void CalibrationInit(vector<Eigen::Matrix4d> json_param) {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  for (int i = 0; i < json_param.size(); i++) {
    calibration_matrix_[i] = json_param[i];
    orign_calibration_matrix_[i] = json_param[i];
    modification_list_[i].reserve(12);
    for (int32_t j = 0; j < 12; j++) {
      std::vector<int> transform_flag(6, 0);
      transform_flag[j / 2] = (j % 2) ? (-1) : 1;
      Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
      Eigen::Matrix3d rot_tmp;
      rot_tmp = Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ /
                                      180.0 * M_PI,
                                  Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ /
                                      180.0 * M_PI,
                                  Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ /
                                      180.0 * M_PI,
                                  Eigen::Vector3d::UnitZ());
      tmp.block(0, 0, 3, 3) = rot_tmp;
      tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
      tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
      tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
      modification_list_[i][j] = tmp;
    }
  }

  std::cout << "=>Calibration scale Init!\n";
}

void CalibrationScaleChange(int frame) {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  modification_list_[frame].reserve(12);
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
    modification_list_[frame][i] = tmp;
  }
  std::cout << "=>Calibration scale update done!\n";
}

void saveResult(const int &frame_id) {
  std::string file_name = "calibration_" + std::to_string(frame_id) + ".txt";
  std::ofstream fCalib(file_name);
  if (!fCalib.is_open()) {
    std::cerr << "open file " << file_name << " failed." << std::endl;
    return;
  }
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "R:\n"
         << calibration_matrix_[frame_id](0, 0) << " "
         << calibration_matrix_[frame_id](0, 1) << " "
         << calibration_matrix_[frame_id](0, 2) << "\n"
         << calibration_matrix_[frame_id](1, 0) << " "
         << calibration_matrix_[frame_id](1, 1) << " "
         << calibration_matrix_[frame_id](1, 2) << "\n"
         << calibration_matrix_[frame_id](2, 0) << " "
         << calibration_matrix_[frame_id](2, 1) << " "
         << calibration_matrix_[frame_id](2, 2) << std::endl;
  fCalib << "t: " << calibration_matrix_[frame_id](0, 3) << " "
         << calibration_matrix_[frame_id](1, 3) << " "
         << calibration_matrix_[frame_id](2, 3) << std::endl;

  fCalib << "************* json format *************" << std::endl;
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "[" << calibration_matrix_[frame_id](0, 0) << ","
         << calibration_matrix_[frame_id](0, 1) << ","
         << calibration_matrix_[frame_id](0, 2) << ","
         << calibration_matrix_[frame_id](0, 3) << "],"
         << "[" << calibration_matrix_[frame_id](1, 0) << ","
         << calibration_matrix_[frame_id](1, 1) << ","
         << calibration_matrix_[frame_id](1, 2) << ","
         << calibration_matrix_[frame_id](1, 3) << "],"
         << "[" << calibration_matrix_[frame_id](2, 0) << ","
         << calibration_matrix_[frame_id](2, 1) << ","
         << calibration_matrix_[frame_id](2, 2) << ","
         << calibration_matrix_[frame_id](2, 3) << "],"
         << "[" << calibration_matrix_[frame_id](3, 0) << ","
         << calibration_matrix_[frame_id](3, 1) << ","
         << calibration_matrix_[frame_id](3, 2) << ","
         << calibration_matrix_[frame_id](3, 3) << "]" << std::endl;

  fCalib.close();
}

bool ManualCalibration(int key_input, int frame) {
  char table[] = {'q', 'a', 'w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h'};
  bool real_hit = false;
  for (int32_t i = 0; i < 12; i++) {
    if (key_input == table[i]) {
      calibration_matrix_[frame] =
          calibration_matrix_[frame] * modification_list_[frame][i];
      real_hit = true;
    }
  }
  return real_hit;
}

int main(int argc, char **argv) {
  if (argc != 5) {
    cout << "Usage: ./run_avm <image_path> "
            "<intrinsic_path> <extrinsic_path>"
            "<camera_model>"
            "0 represents fisheye model in opencv,1 represents Omnidirectional "
            "model and 2 represents pinhole model"
            "\nexample:\n\t"
            "./bin/run_avm pinhole_samples/imgs pinhole_samples/intrinsic "
            "pinhole_samples/extrinsic 2"
            "\nor\n\t"
            "./bin/run_avm ocam_samples/imgs ocam_samples/intrinsic "
            "ocam_samples/extrinsic 1"
         << endl;
    return 0;
  }

  string camera_path = argv[1];
  string intrinsic_path = argv[2];
  string extrinsic_path = argv[3];
  int camera_model = stoi(argv[4]);

  string front_camera = camera_path + "/Front.png";
  string back_camera = camera_path + "/Back.png";
  string left_camera = camera_path + "/Left.png";
  string right_camera = camera_path + "/Right.png";

  vector<Eigen::Matrix3d> K(4);
  vector<vector<double>> dist(4);
  vector<string> intrinsic_json(4);
  intrinsic_json[0] = intrinsic_path + "/Front.json";
  intrinsic_json[1] = intrinsic_path + "/Left.json";
  intrinsic_json[2] = intrinsic_path + "/Back.json";
  intrinsic_json[3] = intrinsic_path + "/Right.json";

  for (int i = 0; i < 4; i++) {
    LoadIntrinsic(intrinsic_json[i], K[i], dist[i]);
  }

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < dist[i].size(); j++)
      distortions_[i].push_back(dist[i][j]);
  }
  intrinsic_matrix_ = K;
  orign_intrinsic_matrix_ = intrinsic_matrix_;

  // load extrinsic
  vector<Eigen::Matrix4d> json_param(4);
  vector<string> extrinsic_json(4);
  extrinsic_json[0] = extrinsic_path + "/Front.json";
  extrinsic_json[1] = extrinsic_path + "/Left.json";
  extrinsic_json[2] = extrinsic_path + "/Back.json";
  extrinsic_json[3] = extrinsic_path + "/Right.json";
  for (int i = 0; i < 4; i++) {
    LoadExtrinsic(extrinsic_json[i], json_param[i]);
  }
  CalibrationInit(json_param);
  cout << "Loading data completed!" << endl;

  cout << "--------------------Init K_G--------------------------" << endl;

  int rows = 1000;
  int cols = 1000;
  double dX = 0.004;
  double dY = 0.004;
  cv::Mat K_G = cv::Mat::zeros(3, 3, CV_64FC1);

  K_G.at<double>(0, 0) = 1 / dX;
  K_G.at<double>(1, 1) = 1 / dY;
  K_G.at<double>(0, 2) = cols / 2;
  K_G.at<double>(1, 2) = rows / 2;
  K_G.at<double>(2, 2) = 1.0;

  cout << "--------------------Project images on the "
          "ground--------------------------"
       << endl;
  vector<cv::Mat> imgs(4);
  imgs[0] = cv::imread(front_camera);
  imgs[1] = cv::imread(left_camera);
  imgs[2] = cv::imread(back_camera);
  imgs[3] = cv::imread(right_camera);
  // note!!!different heights for different samples
  vector<float> bev2ground(4, 5.1); // height for pinhole-samples
  if (camera_model == 1) {
    // height for ocam-samples
    bev2ground[0] = 6.07826;
    bev2ground[1] = 6.25217;
    bev2ground[2] = 6.31014;
    bev2ground[3] = 6.28116;
  }

  vector<cv::Mat> imgs_G(4);
  for (int i = 0; i < 4; i++) {
    imgs_G[i] = project_on_ground(imgs[i], json_param[i], K[i], dist[i], K_G,
                                  rows, cols, camera_model, bev2ground[i]);
    string file_name = to_string(i) + ".png";
  }
  cv::Mat img_G = generate_surround_view(imgs_G[0], imgs_G[1], imgs_G[2],
                                         imgs_G[3], rows, cols);
  int width = 1000, height = 1000;

  pangolin::CreateWindowAndBind("avm player", width, height);
  glEnable(GL_DEPTH_TEST);
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));

  pangolin::View &project_image =
      pangolin::Display("project")
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(150), 1.0,
                     1.0 * (width) / height)
          .SetLock(pangolin::LockLeft, pangolin::LockTop);

  unsigned char *imageArray = new unsigned char[3 * width * height];
  pangolin::GlTexture imageTexture(width, height, GL_RGB, false, 0, GL_RGB,
                                   GL_UNSIGNED_BYTE);

  // control panel
  pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(0), 1.0, 0.0,
                                        pangolin::Attach::Pix(150));
  pangolin::Var<bool> displayMode("cp.stiching", false, true);
  pangolin::Var<double> degreeStep("cp.deg step", 0.3, 0, 5);
  pangolin::Var<double> tStep("cp.t step(cm)", 6, 0, 200);
  pangolin::Var<int> frameStep("cp.frame", 0, 0, 3);

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

  cv::Mat current_frame = imgs_G[0].clone();

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (displayMode.GuiChanged()) {
      cali_stiching_mode = displayMode.Get();
    }

    if (degreeStep.GuiChanged()) {
      cali_scale_degree_ = degreeStep.Get();
      CalibrationScaleChange(cali_frame);
      std::cout << "Degree calib scale changed to " << cali_scale_degree_
                << " degree\n";
    }
    if (tStep.GuiChanged()) {
      cali_scale_trans_ = tStep.Get() / 100.0;
      CalibrationScaleChange(cali_frame);
      std::cout << "Trans calib scale changed to " << cali_scale_trans_ * 100
                << " cm\n";
    }

    if (frameStep.GuiChanged()) {
      cali_frame = frameStep.Get();
      std::cout << "Frame changed to " << cali_frame;
      CalibrationScaleChange(cali_frame);
      imgs_G[cali_frame] =
          project_on_ground(imgs[cali_frame], calibration_matrix_[cali_frame],
                            K[cali_frame], dist[cali_frame], K_G, rows, cols,
                            camera_model, bev2ground[cali_frame]);
    }

    for (int i = 0; i < 12; i++) {
      if (pangolin::Pushed(mat_calib_box[i])) {
        if (i == 10) {
          bev2ground[cali_frame] -= cali_scale_trans_;
        } else if (i == 11) {
          bev2ground[cali_frame] += cali_scale_trans_;
        }
        cout << "height:" << bev2ground[cali_frame] << endl;
        calibration_matrix_[cali_frame] =
            calibration_matrix_[cali_frame] * modification_list_[cali_frame][i];
        std::cout << "Changed!\n";
        imgs_G[cali_frame] =
            project_on_ground(imgs[cali_frame], calibration_matrix_[cali_frame],
                              K[cali_frame], dist[cali_frame], K_G, rows, cols,
                              camera_model, bev2ground[cali_frame]);
      }
    }

    if (pangolin::Pushed(resetButton)) {
      calibration_matrix_[cali_frame] = orign_calibration_matrix_[cali_frame];
      imgs_G[cali_frame] =
          project_on_ground(imgs[cali_frame], calibration_matrix_[cali_frame],
                            K[cali_frame], dist[cali_frame], K_G, rows, cols,
                            camera_model, bev2ground[cali_frame]);
      std::cout << "Reset!\n";
    }
    if (pangolin::Pushed(saveImg)) {
      for (int cnt = 0; cnt < 4; cnt++) {
        cout << "height:" << bev2ground[cnt] << endl;
      }
      if (cali_stiching_mode) {
        for (int count = 0; count < 4; count++) {
          saveResult(count);
        }
        cv::imwrite("stiching.png", img_G);
      } else {
        saveResult(cali_frame);
        string img_name = "calibimg_" + std::to_string(cali_frame) + ".png";
        cv::imwrite(img_name, current_frame);
        cout << "\n==>Save Result " << cali_frame << endl;
        Eigen::Matrix4d transform = calibration_matrix_[cali_frame];
        cout << "Transfromation Matrix:\n" << transform << endl;
      }
    }

    if (kbhit()) {
      int c = getchar();
      if (ManualCalibration(c, cali_frame)) {
        Eigen::Matrix4d transform = calibration_matrix_[cali_frame];
        cout << "\nTransfromation Matrix:\n" << transform << std::endl;
      }
      imgs_G[cali_frame] =
          project_on_ground(imgs[cali_frame], calibration_matrix_[cali_frame],
                            K[cali_frame], dist[cali_frame], K_G, rows, cols,
                            camera_model, bev2ground[cali_frame]);
    }

    if (cali_stiching_mode) {
      img_G = generate_surround_view(imgs_G[0], imgs_G[1], imgs_G[2], imgs_G[3],
                                     rows, cols);
      current_frame = img_G.clone();
    } else {
      current_frame = imgs_G[cali_frame].clone();
    }
    imageArray = current_frame.data;
    imageTexture.Upload(imageArray, GL_BGR, GL_UNSIGNED_BYTE);

    project_image.Activate();
    glColor3f(1.0, 1.0, 1.0);
    imageTexture.RenderToViewportFlipY();

    pangolin::FinishFrame();
    glFinish();
  }

  return 0;
}
