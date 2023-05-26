#include <Eigen/Core>
#include <Eigen/Dense>
#include <glog/logging.h>
#include <iostream>
#include <math.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <stdlib.h>
#include <time.h>
#include <vector>
using namespace std;
using namespace cv;
using namespace Eigen;

class extractor {
public:
  Mat img1_bev;
  Mat img2_bev;

  Mat intrinsic1;
  Mat intrinsic2;

  Mat bin_of_imgs;
  Mat bev_of_imgs;

  int edge_flag; // if filter edge

  int exposure_flag; // if add exposure solution

  double ncoef; // exposure coefficients

  vector<vector<Point>> contours;

  Eigen::Matrix4d extrinsic1;
  Eigen::Matrix4d extrinsic2;

  extractor(Mat img1_bev, Mat img2_bev, int edge_flag, int exposure_flag);
  extractor(Mat img1_bev, Mat img2_bev, Mat intrinsic1, Mat intrinsic2,
            Eigen::Matrix4d extrinsic1, Eigen::Matrix4d extrinsic2);
  ~extractor();
  void Binarization();
  void writetocsv(string filename, vector<Point> vec);
  void findcontours();
  std::vector<std::vector<cv::Point>>
  fillContour(const std::vector<std::vector<cv::Point>> &_contours);
  vector<cv::Point> extrac_textures_and_save(string pic_filename,
                                             string csv_filename, string idx,
                                             double size);
  static bool cmpx(Point p1, Point p2) { return p1.x < p2.x; };
  static bool cmpy(Point p1, Point p2) { return p1.y < p2.y; };
};