#include "texture_extractor.h"
#include <fstream>

extractor::extractor(Mat img1_bev_, Mat img2_bev_, int edge_flag_,
                     int exposure_flag_) {
  img1_bev = img1_bev_;
  img2_bev = img2_bev_;
  edge_flag = edge_flag_;
  exposure_flag = exposure_flag_;
  Mat bev;
  addWeighted(img1_bev, 0.5, img2_bev, 0.5, 0, bev);
  bev_of_imgs = bev;
}

extractor::extractor(Mat img1_bev_, Mat img2_bev_, Mat intrinsic1_,
                     Mat intrinsic2_, Eigen::Matrix4d extrinsic1_,
                     Eigen::Matrix4d extrinsic2_) {
  img1_bev = img1_bev_;
  img2_bev = img2_bev_;
  intrinsic1 = intrinsic1_;
  intrinsic2 = intrinsic2_;
  extrinsic1 = extrinsic1_;
  extrinsic2 = extrinsic2_;
}

extractor::~extractor() {}

void extractor::writetocsv(string filename, vector<Point> vec) {
  ofstream outfile;
  outfile.open(filename, ios::out);
  for (auto pixel : vec) {
    outfile << pixel.x << " " << pixel.y << endl;
  }
  outfile.close();
}

void extractor::Binarization() {
  assert(img1_bev.rows == img2_bev.rows);
  assert(img1_bev.cols == img2_bev.cols);
  int rows = img1_bev.rows;
  int cols = img2_bev.cols;
  Mat dst(img1_bev.size(), CV_8UC1);
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      if (img1_bev.at<Vec3b>(i, j) != Vec3b(0, 0, 0) &&
          img2_bev.at<Vec3b>(i, j) != Vec3b(0, 0, 0)) {
        dst.at<uchar>(i, j) = 255;
      } else {
        dst.at<uchar>(i, j) = 0;
      }
    }
  }
  bin_of_imgs = dst;
}

void extractor::findcontours() {
  vector<vector<Point>> contours;
  Mat erode_img, dilate_img;
  Mat dilate_kernel = getStructuringElement(0, Size(5, 5));
  dilate(bin_of_imgs, dilate_img, dilate_kernel);
  Mat erode_kernel = getStructuringElement(0, Size(5, 5));
  erode(dilate_img, erode_img, erode_kernel, Point(-1, -1), 2);
  cv::findContours(erode_img, contours, cv::noArray(), CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_NONE);

  int maxsize = 0;
  int index = 0;
  for (int i = 0; i < contours.size(); i++) {
    if (contours[i].size() > maxsize) {
      maxsize = contours[i].size();
      index = i;
    }
  }

  vector<vector<Point>> contours_after_filter;
  contours_after_filter.push_back(contours[index]);
  ;

  std::vector<std::vector<cv::Point>> contours_pixels;
  contours_pixels = fillContour(contours_after_filter);
  this->contours = contours_pixels;
}

std::vector<std::vector<cv::Point>>
extractor::fillContour(const std::vector<std::vector<cv::Point>> &_contours) {
  // sort as x descent y descent.
  std::vector<std::vector<cv::Point>> contours(_contours);
  for (size_t i = 0; i < contours.size(); ++i) {
    std::vector<cv::Point> sub(contours[i]);
    std::sort(sub.begin(), sub.end(), [&](cv::Point &A, cv::Point &B) {
      if (A.x == B.x)
        return A.y < B.y;
      else
        return A.x < B.x;
    });
    contours[i] = sub;
  }
  // restore as pairs with same ys.
  std::vector<std::vector<std::pair<cv::Point, cv::Point>>> contours_pair;
  for (size_t i = 0; i < contours.size(); ++i) {
    std::vector<std::pair<cv::Point, cv::Point>> pairs;
    for (size_t j = 0; j < contours[i].size(); ++j) {
      // j==0
      if (pairs.size() == 0) {
        pairs.push_back({contours[i][j], contours[i][j]});
        continue;
      }
      // j>0
      if (contours[i][j].x != pairs[pairs.size() - 1].first.x) {
        pairs.push_back({contours[i][j], contours[i][j]});
        continue;
      }

      if (contours[i][j].x == pairs[pairs.size() - 1].first.x) {
        if (contours[i][j].y > pairs[pairs.size() - 1].second.y)
          pairs[pairs.size() - 1].second = contours[i][j];
        continue;
      }
    }
    contours_pair.push_back(pairs);
  }

  // fill contour coordinates.
  std::vector<std::vector<cv::Point>> fill_con;
  for (auto pair_set : contours_pair) {
    std::vector<cv::Point> pointSet;
    for (auto aPair : pair_set) {
      if (aPair.first == aPair.second) {
        pointSet.push_back(aPair.first);

      } else {
        for (int i = aPair.first.y; i <= aPair.second.y; ++i) {
          pointSet.push_back(cv::Point(aPair.first.x, i));
        }
      }
    }
    fill_con.push_back(pointSet);
  }
  return fill_con;
}

vector<cv::Point> extractor::extrac_textures_and_save(string pic_filename,
                                                      string csv_filename,
                                                      string idx, double size) {
  int down_sample = 500;
  Mat gray1, gray2;
  cvtColor(img1_bev, gray1, COLOR_BGR2GRAY);
  cvtColor(img2_bev, gray2, COLOR_BGR2GRAY);
  if (exposure_flag)
    ncoef = mean(gray1).val[0] / mean(gray2).val[0];
  GaussianBlur(gray1, gray1, Size(3, 3), 0);
  GaussianBlur(gray2, gray2, Size(3, 3), 0);

  double camera_center_x, camera_center_y;
  if (idx == "fl") {
    camera_center_x = img1_bev.cols / 2;
    camera_center_y = size;
  } else if (idx == "fr") {
    camera_center_x = img1_bev.cols / 2;
    camera_center_y = size;
  } else if (idx == "bl") {
    camera_center_x = img1_bev.cols / 2;
    camera_center_y = img1_bev.rows - size;
  } else if (idx == "br") {
    camera_center_x = img1_bev.cols / 2;
    camera_center_y = img1_bev.rows - size;
  } else if (idx == "lf") {
    camera_center_x = size;
    camera_center_y = img1_bev.rows / 2;
  } else if (idx == "rf") {
    camera_center_x = img1_bev.cols - size;
    camera_center_y = img1_bev.rows / 2;
  } else if (idx == "lb") {
    camera_center_x = size;
    camera_center_y = img1_bev.rows / 2;
  } else if (idx == "rb") {
    camera_center_x = img1_bev.cols - size;
    camera_center_y = img1_bev.rows / 2;
  }

  vector<cv::Point> commonview;
  vector<Point> contour1_pixels = contours[0];
  for (auto pixel : contour1_pixels) {
    if (pixel.x < 10 || pixel.y < 10 || (pixel.x + 10) > img1_bev.cols ||
        (pixel.y + 10) > img1_bev.rows)
      continue;
    if (edge_flag && (abs(pixel.x - camera_center_x) > (size * 4.5 / 5) ||
                      abs(pixel.y - camera_center_y) > (size * 4.5 / 5)))
      continue;
    Eigen::Vector2d delta((gray1.ptr<uchar>(pixel.y)[pixel.x] -
                           gray1.ptr<uchar>(pixel.y)[pixel.x - 2]),
                          (gray1.ptr<uchar>(pixel.y)[pixel.x] -
                           gray1.ptr<uchar>(pixel.y - 2)[pixel.x]));
    if (delta.norm() < 15)
      continue;
    commonview.push_back(pixel);
  }
  vector<cv::Point> commonview_down;
  Mat show = img1_bev.clone();
  srand((int)time(0));
  for (int i = 0; i < down_sample; i++) {
    int k = rand() % commonview.size();
    commonview_down.push_back(commonview[k]);
    circle(show, commonview[k], 1, Scalar(0, 0, 0), -1);
  }
  return commonview_down;
}
