/*
    front camera fixed
*/
#include "optimizer.h"
#include "texture_extractor.h"
#include "transform_util.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <thread>
#include <time.h>
#include <vector>

using namespace cv;
using namespace std;

double during_bev;
double during_compute_error;
double during_wrap;

double CameraOptimization(Optimizer &opt, string cameraType) {
  std::chrono::_V2::steady_clock::time_point end_calib_ =
      chrono::steady_clock::now();
  ;
  double during_calib_ = 0;
  if (opt.coarse_flag) {
    cout << "**************************************1st*************************"
            "***************"
         << endl;
    int thread_num = 7;
    vector<thread> threads(thread_num);
    auto start_calib = chrono::steady_clock::now();
    if (cameraType == "right") {
      int iter_nums = 100000;
      threads[0] = thread(&Optimizer::Calibrate_right, &opt, iter_nums, -3, 3,
                          -3, 3, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[1] = thread(&Optimizer::Calibrate_right, &opt, iter_nums, -3, 0,
                          -3, 3, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[2] = thread(&Optimizer::Calibrate_right, &opt, iter_nums, 0, 3,
                          -3, 3, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[3] = thread(&Optimizer::Calibrate_right, &opt, iter_nums, -3, 3,
                          -3, 0, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[4] = thread(&Optimizer::Calibrate_right, &opt, iter_nums, -3, 3,
                          0, 3, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[5] = thread(&Optimizer::Calibrate_right, &opt, iter_nums, -3, 3,
                          -3, 3, -3, 0, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[6] = thread(&Optimizer::Calibrate_right, &opt, iter_nums, -3, 3,
                          -3, 3, 0, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
    } else if (cameraType == "left") {
      int iter_nums = 100000;
      threads[0] = thread(&Optimizer::Calibrate_left, &opt, iter_nums, -3, 3,
                          -3, 3, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[1] = thread(&Optimizer::Calibrate_left, &opt, iter_nums, -3, 0,
                          -3, 3, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[2] = thread(&Optimizer::Calibrate_left, &opt, iter_nums, 0, 3, -3,
                          3, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[3] = thread(&Optimizer::Calibrate_left, &opt, iter_nums, -3, 3,
                          -3, 0, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[4] = thread(&Optimizer::Calibrate_left, &opt, iter_nums, -3, 3, 0,
                          3, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[5] = thread(&Optimizer::Calibrate_left, &opt, iter_nums, -3, 3,
                          -3, 3, -3, 0, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[6] = thread(&Optimizer::Calibrate_left, &opt, iter_nums, -3, 3,
                          -3, 3, 0, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
    } else if (cameraType == "behind") {
      int iter_nums = 100000;
      threads[0] = thread(&Optimizer::Calibrate_behind, &opt, iter_nums, -3, 3,
                          -3, 3, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[1] = thread(&Optimizer::Calibrate_behind, &opt, iter_nums, -3, 0,
                          -3, 3, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[2] = thread(&Optimizer::Calibrate_behind, &opt, iter_nums, 0, 3,
                          -3, 3, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[3] = thread(&Optimizer::Calibrate_behind, &opt, iter_nums, -3, 3,
                          -3, 0, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[4] = thread(&Optimizer::Calibrate_behind, &opt, iter_nums, -3, 3,
                          0, 3, -3, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[5] = thread(&Optimizer::Calibrate_behind, &opt, iter_nums, -3, 3,
                          -3, 3, -3, 0, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
      threads[6] = thread(&Optimizer::Calibrate_behind, &opt, iter_nums, -3, 3,
                          -3, 3, 0, 3, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
    }
    for (int i = 0; i < thread_num; i++) {
      threads[i].join();
    }
    end_calib_ = chrono::steady_clock::now();
    during_calib_ =
        std::chrono::duration<double>(end_calib_ - start_calib).count();
    cout << "time:" << during_calib_ << endl;
    if (cameraType == "left") {
      opt.show("left", opt.prefix + "/after_left_calib1.png");
      cout << "luminorsity loss before pre opt:" << opt.max_left_loss << endl;
      cout << "luminorsity loss after pre opt:" << opt.cur_left_loss << endl;
      cout << "extrinsic after pre opt:" << endl
           << opt.extrinsic_left_opt << endl;
      cout << "best search parameters:" << endl;
      for (auto e : opt.bestVal_[0])
        cout << fixed << setprecision(6) << e << " ";
    } else if (cameraType == "behind") {
      opt.show("behind", opt.prefix + "/after_behind_calib1.png");
      cout << "luminorsity loss before pre opt:" << opt.max_behind_loss << endl;
      cout << "luminorsity loss after pre opt:" << opt.cur_behind_loss << endl;
      cout << "extrinsic after pre opt:" << endl
           << opt.extrinsic_behind_opt << endl;
      cout << "best search parameters:" << endl;
      for (auto e : opt.bestVal_[2])
        cout << fixed << setprecision(6) << e << " ";
    } else if (cameraType == "right") {
      opt.show("right", opt.prefix + "/after_right_calib1.png");
      cout << "luminorsity loss before pre opt:" << opt.max_right_loss << endl;
      cout << "luminorsity loss after pre opt:" << opt.cur_right_loss << endl;
      cout << "extrinsic after pre opt:" << endl
           << opt.extrinsic_right_opt << endl;
      cout << "best search parameters:" << endl;
      for (auto e : opt.bestVal_[1])
        cout << fixed << setprecision(6) << e << " ";
    }
    cout << endl;
  }

  cout << "**************************************2nd***************************"
          "*************"
       << endl;
  int thread_num_ = 6;
  int iter_nums_ = 15000;
  vector<thread> threads_(thread_num_);
  if (cameraType == "right") {
    threads_[0] =
        thread(&Optimizer::fine_Calibrate_right, &opt, iter_nums_, -1, 1, -1, 1,
               -1, 1, -0.005, 0.005, -0.005, 0.005, -0.005, 0.005);
    threads_[1] =
        thread(&Optimizer::fine_Calibrate_right, &opt, iter_nums_, -0.5, 0.5,
               -0.5, 0.5, -0.5, 0.5, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
    threads_[2] = thread(&Optimizer::fine_Calibrate_right, &opt, iter_nums_,
                         -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.005, 0.005, -0.005,
                         0.005, -0.005, 0.005);
    threads_[3] = thread(&Optimizer::fine_Calibrate_right, &opt, iter_nums_,
                         -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.005, 0.005, -0.005,
                         0.005, -0.005, 0.005);
    threads_[4] = thread(&Optimizer::fine_Calibrate_right, &opt, iter_nums_,
                         -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.005, 0.005, -0.005,
                         0.005, -0.005, 0.005);
    threads_[5] = thread(&Optimizer::fine_Calibrate_right, &opt, iter_nums_,
                         -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.005, 0.005, -0.005,
                         0.005, -0.005, 0.005);
  } else if (cameraType == "left") {
    threads_[0] =
        thread(&Optimizer::fine_Calibrate_left, &opt, iter_nums_, -1, 1, -1, 1,
               -1, 1, -0.005, 0.005, -0.005, 0.005, -0.005, 0.005);
    threads_[1] =
        thread(&Optimizer::fine_Calibrate_left, &opt, iter_nums_, -0.5, 0.5,
               -0.5, 0.5, -0.5, 0.5, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
    threads_[2] = thread(&Optimizer::fine_Calibrate_left, &opt, iter_nums_,
                         -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.005, 0.005, -0.005,
                         0.005, -0.005, 0.005);
    threads_[3] = thread(&Optimizer::fine_Calibrate_left, &opt, iter_nums_,
                         -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.005, 0.005, -0.005,
                         0.005, -0.005, 0.005);
    threads_[4] = thread(&Optimizer::fine_Calibrate_left, &opt, iter_nums_,
                         -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.005, 0.005, -0.005,
                         0.005, -0.005, 0.005);
    threads_[5] = thread(&Optimizer::fine_Calibrate_left, &opt, iter_nums_,
                         -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.005, 0.005, -0.005,
                         0.005, -0.005, 0.005);
  } else if (cameraType == "behind") {
    threads_[0] =
        thread(&Optimizer::fine_Calibrate_behind, &opt, iter_nums_, -1, 1, -1,
               1, -1, 1, -0.005, 0.005, -0.005, 0.005, -0.005, 0.005);
    threads_[1] =
        thread(&Optimizer::fine_Calibrate_behind, &opt, iter_nums_, -0.5, 0.5,
               -0.5, 0.5, -0.5, 0.5, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
    threads_[2] = thread(&Optimizer::fine_Calibrate_behind, &opt, iter_nums_,
                         -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.005, 0.005, -0.005,
                         0.005, -0.005, 0.005);
    threads_[3] = thread(&Optimizer::fine_Calibrate_behind, &opt, iter_nums_,
                         -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.005, 0.005, -0.005,
                         0.005, -0.005, 0.005);
    threads_[4] = thread(&Optimizer::fine_Calibrate_behind, &opt, iter_nums_,
                         -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.005, 0.005, -0.005,
                         0.005, -0.005, 0.005);
    threads_[5] = thread(&Optimizer::fine_Calibrate_behind, &opt, iter_nums_,
                         -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.005, 0.005, -0.005,
                         0.005, -0.005, 0.005);
  }
  for (int i = 0; i < thread_num_; i++) {
    threads_[i].join();
  }
  auto end_calib__ = chrono::steady_clock::now();
  double during_calib__ =
      std::chrono::duration<double>(end_calib__ - end_calib_).count();
  cout << "time:" << during_calib__ << endl;
  if (cameraType == "left") {
    opt.show("left", opt.prefix + "/after_left_calib2.png");
    cout << "luminorsity loss after opt:" << opt.cur_left_loss << endl;
    cout << "extrinsic after opt:" << endl << opt.extrinsic_left_opt << endl;
    cout << "best search parameters:" << endl;
    for (auto e : opt.bestVal_[0])
      cout << fixed << setprecision(6) << e << " ";
  } else if (cameraType == "behind") {
    opt.show("behind", opt.prefix + "/after_behind_calib2.png");
    cout << "luminorsity loss after opt:" << opt.cur_behind_loss << endl;
    cout << "extrinsic after opt:" << endl << opt.extrinsic_behind_opt << endl;
    cout << "best search parameters:" << endl;
    for (auto e : opt.bestVal_[2])
      cout << fixed << setprecision(6) << e << " ";
  } else if (cameraType == "right") {
    opt.show("right", opt.prefix + "/after_right_calib2.png");
    cout << "luminorsity loss after opt:" << opt.cur_right_loss << endl;
    cout << "extrinsic after opt:" << endl << opt.extrinsic_right_opt << endl;
    cout << "best search parameters:" << endl;
    for (auto e : opt.bestVal_[1])
      cout << fixed << setprecision(6) << e << " ";
  }
  cout << endl;

  cout << "**************************************3rd***************************"
          "*************"
       << endl;
  int thread_num__ = 3;
  int iter_nums__ = 8000;
  vector<thread> threads__(thread_num__);
  if (cameraType == "right") {
    threads__[0] = thread(&Optimizer::fine_Calibrate_right, &opt, iter_nums__,
                          -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.002, 0.002,
                          -0.002, 0.002, -0.002, 0.002);
    threads__[1] = thread(&Optimizer::fine_Calibrate_right, &opt, iter_nums__,
                          -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.001, 0.001,
                          -0.001, 0.001, -0.001, 0.001);
    threads__[2] = thread(&Optimizer::fine_Calibrate_right, &opt, iter_nums__,
                          -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.001, 0.01,
                          -0.001, 0.001, -0.001, 0.001);
  } else if (cameraType == "left") {
    threads__[0] = thread(&Optimizer::fine_Calibrate_left, &opt, iter_nums__,
                          -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.002, 0.002,
                          -0.002, 0.002, -0.002, 0.002);
    threads__[1] = thread(&Optimizer::fine_Calibrate_left, &opt, iter_nums__,
                          -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.001, 0.001,
                          -0.001, 0.001, -0.001, 0.001);
    threads__[2] = thread(&Optimizer::fine_Calibrate_left, &opt, iter_nums__,
                          -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.001, 0.001,
                          -0.001, 0.001, -0.001, 0.001);
  } else if (cameraType == "behind") {
    threads__[0] = thread(&Optimizer::fine_Calibrate_behind, &opt, iter_nums__,
                          -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.002, 0.002,
                          -0.002, 0.002, -0.002, 0.002);
    threads__[1] = thread(&Optimizer::fine_Calibrate_behind, &opt, iter_nums__,
                          -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.001, 0.001,
                          -0.001, 0.001, -0.001, 0.001);
    threads__[2] = thread(&Optimizer::fine_Calibrate_behind, &opt, iter_nums__,
                          -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.001, 0.001,
                          -0.001, 0.001, -0.001, 0.001);
  }
  for (int i = 0; i < thread_num__; i++) {
    threads__[i].join();
  }
  auto end_calib___ = chrono::steady_clock::now();
  double during_calib___ =
      std::chrono::duration<double>(end_calib___ - end_calib__).count();
  cout << "time:" << during_calib___ << endl;
  if (cameraType == "left") {
    opt.show("left", opt.prefix + "/after_left_calib3.png");
    cout << "luminorsity loss after opt:" << opt.cur_left_loss << endl;
    cout << "extrinsic after opt:" << endl << opt.extrinsic_left_opt << endl;
    cout << "best search parameters:" << endl;
    for (auto e : opt.bestVal_[0])
      cout << fixed << setprecision(6) << e << " ";
  } else if (cameraType == "behind") {
    opt.show("behind", opt.prefix + "/after_behind_calib3.png");
    cout << "luminorsity loss after opt:" << opt.cur_behind_loss << endl;
    cout << "extrinsic after opt:" << endl << opt.extrinsic_behind_opt << endl;
    cout << "best search parameters:" << endl;
    for (auto e : opt.bestVal_[2])
      cout << fixed << setprecision(6) << e << " ";
  } else if (cameraType == "right") {
    opt.show("right", opt.prefix + "/after_right_calib3.png");
    cout << "luminorsity loss after opt:" << opt.cur_right_loss << endl;
    cout << "extrinsic after opt:" << endl << opt.extrinsic_right_opt << endl;
    cout << "best search parameters:" << endl;
    for (auto e : opt.bestVal_[1])
      cout << fixed << setprecision(6) << e << " ";
  }
  cout << endl
       << cameraType << " calibration time: "
       << during_calib_ + during_calib__ + during_calib___ << "s" << endl;
  return during_calib_ + during_calib__ + during_calib___;
}

int main() {
  // camera_model:0-fisheye;1-Ocam;2-pinhole
  int camera_model = 2;

  // read frames(imgs3/4/5)
  Mat imgf = cv::imread("./imgs/Front.png");
  Mat imgl = cv::imread("./imgs/Left.png");
  Mat imgb = cv::imread("./imgs/Back.png");
  Mat imgr = cv::imread("./imgs/Right.png");

  // bev rows、cols
  int bev_rows = 1500, bev_cols = 1500;

  // if add coarse search(1st search)
  int coarse_search_flag = 1;

  // which data_set(common or fisheye camera)
  string data_set = "common";

  string fixed = "front";
  // initilize the optimizer
  Optimizer opt(&imgf, &imgl, &imgb, &imgr, camera_model, bev_rows, bev_cols,
                fixed, coarse_search_flag, data_set);

  // bev images before optimization
  Mat GF = opt.project_on_ground(imgf, opt.extrinsic_front, opt.intrinsic_front,
                                 opt.distortion_params_front, opt.KG, opt.brows,
                                 opt.bcols, opt.hf);
  Mat GB = opt.project_on_ground(
      imgb, opt.extrinsic_behind, opt.intrinsic_behind,
      opt.distortion_params_behind, opt.KG, opt.brows, opt.bcols, opt.hb);
  Mat GL = opt.project_on_ground(imgl, opt.extrinsic_left, opt.intrinsic_left,
                                 opt.distortion_params_left, opt.KG, opt.brows,
                                 opt.bcols, opt.hl);
  Mat GR = opt.project_on_ground(imgr, opt.extrinsic_right, opt.intrinsic_right,
                                 opt.distortion_params_right, opt.KG, opt.brows,
                                 opt.bcols, opt.hr);

  GF = opt.tail(GF, "f");
  GB = opt.tail(GB, "b");
  GL = opt.tail(GL, "l");
  GR = opt.tail(GR, "r");

  Mat bev_before = opt.generate_surround_view(GF, GL, GB, GR);
  imwrite("./before_all_calib.png", bev_before);

  // texture extraction
  int edge_flag_fl = 0;     // if filter common-view
  int exposure_flag_fl = 0; // if add exposure solution
  extractor ext1(GF, GL, edge_flag_fl, exposure_flag_fl);
  ext1.Binarization();
  ext1.findcontours();
  opt.fl_pixels_texture = ext1.extrac_textures_and_save(
      opt.prefix + "/texture_fl.png", opt.prefix + "/fl.csv", "fl", opt.sizef);
  if (ext1.exposure_flag)
    opt.ncoef_fl = ext1.ncoef;
  else
    opt.ncoef_fl = 1;
  Mat pG_fl = Mat::ones(3, opt.fl_pixels_texture.size(), CV_64FC1);
  for (int i = 0; i < opt.fl_pixels_texture.size(); i++) {
    pG_fl.at<double>(0, i) = opt.fl_pixels_texture[i].x;
    pG_fl.at<double>(1, i) = opt.fl_pixels_texture[i].y;
  }
  opt.pG_fl = pG_fl;
  Mat PG_fl = Mat::ones(4, opt.fl_pixels_texture.size(), CV_64FC1);
  PG_fl(cv::Rect(0, 0, opt.fl_pixels_texture.size(), 3)) =
      opt.eigen2mat(opt.KG.inverse()) * pG_fl * opt.hf;
  opt.PG_fl = PG_fl;

  int edge_flag_fr = 0;     // if filter common-view
  int exposure_flag_fr = 0; // if add exposure solution
  extractor ext2(GF, GR, edge_flag_fr, exposure_flag_fr);
  ext2.Binarization();
  ext2.findcontours();
  opt.fr_pixels_texture = ext2.extrac_textures_and_save(
      opt.prefix + "/texture_fr.png", opt.prefix + "/fr.csv", "fr", opt.sizef);
  if (ext2.exposure_flag)
    opt.ncoef_fr = ext2.ncoef;
  else
    opt.ncoef_fr = 1;
  Mat pG_fr = Mat::ones(3, opt.fr_pixels_texture.size(), CV_64FC1);
  for (int i = 0; i < opt.fr_pixels_texture.size(); i++) {
    pG_fr.at<double>(0, i) = opt.fr_pixels_texture[i].x;
    pG_fr.at<double>(1, i) = opt.fr_pixels_texture[i].y;
  }
  opt.pG_fr = pG_fr;
  Mat PG_fr = Mat::ones(4, opt.fr_pixels_texture.size(), CV_64FC1);
  PG_fr(cv::Rect(0, 0, opt.fr_pixels_texture.size(), 3)) =
      opt.eigen2mat(opt.KG.inverse()) * pG_fr * opt.hf;
  opt.PG_fr = PG_fr;

  cout << "*********************************start "
          "right*************************************"
       << endl;
  double during1 = CameraOptimization(opt, "right");

  cout << "*********************************start "
          "left**************************************"
       << endl;
  double during2 = CameraOptimization(opt, "left");

  // texture extraction
  int edge_flag_bl = 0;     // if filter common-view
  int exposure_flag_bl = 0; // if add exposure solution
  opt.imgl_bev_rgb = opt.imgl_bev_rgb;
  opt.imgl_bev = opt.gray_gamma(opt.imgl_bev_rgb);
  extractor ext3(opt.imgl_bev_rgb, GB, edge_flag_bl, exposure_flag_bl);
  ext3.Binarization();
  ext3.findcontours();
  opt.bl_pixels_texture = ext3.extrac_textures_and_save(
      opt.prefix + "/texture_bl.png", opt.prefix + "/bl.csv", "bl", opt.sizel);
  if (ext3.exposure_flag)
    opt.ncoef_bl = ext3.ncoef;
  else
    opt.ncoef_bl = 1;
  Mat pG_bl = Mat::ones(3, opt.bl_pixels_texture.size(), CV_64FC1);
  for (int i = 0; i < opt.bl_pixels_texture.size(); i++) {
    pG_bl.at<double>(0, i) = opt.bl_pixels_texture[i].x;
    pG_bl.at<double>(1, i) = opt.bl_pixels_texture[i].y;
  }
  opt.pG_bl = pG_bl;
  Mat PG_bl = Mat::ones(4, opt.bl_pixels_texture.size(), CV_64FC1);
  PG_bl(cv::Rect(0, 0, opt.bl_pixels_texture.size(), 3)) =
      opt.eigen2mat(opt.KG.inverse()) * pG_bl * opt.hl;
  opt.PG_bl = PG_bl;
  opt.imgr_bev_rgb = opt.imgr_bev_rgb;
  opt.imgr_bev = opt.gray_gamma(opt.imgr_bev_rgb);

  int edge_flag_br = 0;     // if filter common-view
  int exposure_flag_br = 0; // if add exposure solution
  extractor ext4(opt.imgr_bev_rgb, GB, edge_flag_br, exposure_flag_br);
  ext4.Binarization();
  ext4.findcontours();
  opt.br_pixels_texture = ext4.extrac_textures_and_save(
      opt.prefix + "/texture_br.png", opt.prefix + "/br.csv", "br", opt.sizer);
  if (ext4.exposure_flag)
    opt.ncoef_br = ext4.ncoef;
  else
    opt.ncoef_br = 1;
  Mat pG_br = Mat::ones(3, opt.br_pixels_texture.size(), CV_64FC1);
  for (int i = 0; i < opt.br_pixels_texture.size(); i++) {
    pG_br.at<double>(0, i) = opt.br_pixels_texture[i].x;
    pG_br.at<double>(1, i) = opt.br_pixels_texture[i].y;
  }
  opt.pG_br = pG_br;
  Mat PG_br = Mat::ones(4, opt.br_pixels_texture.size(), CV_64FC1);
  PG_br(cv::Rect(0, 0, opt.br_pixels_texture.size(), 3)) =
      opt.eigen2mat(opt.KG.inverse()) * pG_br * opt.hr;
  opt.PG_br = PG_br;
  cout << "*********************************start "
          "behind***********************************"
       << endl;
  double during3 = CameraOptimization(opt, "behind");

  cout << "************************online calibration "
          "finished!!!**************************"
       << endl;
  cout << "total calibration time:" << during1 + during2 + during3 << "s"
       << endl;

  opt.SaveOptResult("./after_all_calib.png");
}