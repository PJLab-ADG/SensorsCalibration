/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#pragma once

#include <algorithm>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include <vector>

#include "birdview.hpp"
#include "pcl/io/pcd_io.h"

struct Pt {
  cv::Point point;
  float dist;
  float x;
  float intensity;
};

class BirdView222 {
public:
  Eigen::Matrix3d ground2img_hmat_;
  Eigen::Matrix3d img2ground_hmat_;
  cv::Mat img2bv_hmat_;
  cv::Mat ground2bv_hmat_;
  int width_;
  int height_;

  void init(const int &width, const int &height,
            const Eigen::Matrix3d &img2ground_hmat) {
    width_ = width;
    height_ = height;
    img2ground_hmat_ = img2ground_hmat;
    Eigen::Matrix3d ground2img_hmat_ = img2ground_hmat.inverse().eval();
    std::vector<cv::Point2f> ground_pts;
    std::vector<cv::Point2f> img_pts;
    std::vector<cv::Point2f> bv_pts;
    ground_pts.push_back(cv::Point2f(-5, 0));
    ground_pts.push_back(cv::Point2f(-5, 100));
    ground_pts.push_back(cv::Point2f(5, 0));
    ground_pts.push_back(cv::Point2f(5, 100));
    for (int i = 0; i < 4; i++) {
      Eigen::Vector3d img_pt =
          ground2img_hmat_ *
          Eigen::Vector3d(ground_pts[i].x, ground_pts[i].y, 1);
      img_pt /= img_pt(2);
      img_pts.push_back(cv::Point2f(img_pt(0), img_pt(1)));
    }
    float left = width_ / 6;
    float right = width_ - width_ / 6;
    float up = height_ / 35.0;
    float down = height_ - height_ / 35.0;
    bv_pts.push_back(cv::Point2f(left, down));
    bv_pts.push_back(cv::Point2f(left, up));
    bv_pts.push_back(cv::Point2f(right, down));
    bv_pts.push_back(cv::Point2f(right, up));

    img2bv_hmat_ = cv::findHomography(img_pts, bv_pts);
    ground2bv_hmat_ = cv::findHomography(ground_pts, bv_pts);
  }

  cv::Mat drawProjectBirdView(const cv::Mat &img, const int &point_size,
                              const std::vector<cv::Point2f> grount_pts) {
    cv::Mat output;
    cv::warpPerspective(img, output, img2bv_hmat_, cv::Size(width_, height_),
                        cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                        cv::Scalar(255, 255, 255));
    Eigen::Matrix3d hmat;
    cv::cv2eigen(ground2bv_hmat_, hmat);
    for (size_t i = 0; i < grount_pts.size(); i++) {
      Eigen::Vector3d pt(grount_pts[i].x, grount_pts[i].y, 1.0);
      Eigen::Vector3d bv_pt = hmat * pt;
      bv_pt /= bv_pt(2);
      int xbv = cvRound(bv_pt(0));
      int ybv = cvRound(bv_pt(1));
      cv::Scalar color = cv::Scalar(0, 0, 255);
      cv::circle(output, cv::Point(xbv, ybv), point_size, color, -1);
    }
    return output;
  }
};

class Projector {
public:
  cv::Mat oriCloud;
  // std::vector<float> intensitys;
  const float ROI[6] = {-4, 3.5, 5.0, 10.0, -2.1, 3.0};
  int point_size_ = 6;
  Eigen::Matrix3d homograph_;
  BirdView bv_handler;

  void init(const cv::Mat &img, const Eigen::Matrix3d &homograph,
            const int &bv_w, const int &bv_h) {
    homograph_ = homograph;
    bv_handler.init(img, bv_w, bv_h, homograph);
  }

  void ROIFilter() {
    cv::Mat temp(cv::Size(oriCloud.cols, oriCloud.rows), CV_32FC1);
    int cnt = 0;
    for (int i = 0; i < oriCloud.cols; ++i) {
      float x = oriCloud.at<float>(0, i);
      float y = oriCloud.at<float>(1, i);
      float z = oriCloud.at<float>(2, i);
      if (x > ROI[0] && x < ROI[1] && y > ROI[2] && y < ROI[3] && z > ROI[4] &&
          z < ROI[5]) {
        temp.at<float>(0, cnt) = x;
        temp.at<float>(1, cnt) = y;
        temp.at<float>(2, cnt) = z;
        ++cnt;
      }
    }
    oriCloud = temp.colRange(0, cnt);
  }

  void setPointSize(int size) { point_size_ = size; }

  bool loadPointCloud(pcl::PointCloud<pcl::PointXYZ> pcl) {
    oriCloud = cv::Mat(cv::Size(pcl.points.size(), 3), CV_32FC1);
    for (size_t i = 0; i < pcl.points.size(); ++i) {
      oriCloud.at<float>(0, i) = pcl.points[i].x;
      oriCloud.at<float>(1, i) = pcl.points[i].y;
      oriCloud.at<float>(2, i) = pcl.points[i].z;
      // intensitys.push_back(pcl.points[i].intensity);
    }
    // ROIFilter();
    return true;
  }

  cv::Scalar fakeColor(float value) {
    float posSlope = 255 / 60.0;
    float negSlope = -255 / 60.0;
    value *= 255;
    cv::Vec3f color;
    if (value < 60) {
      color[0] = 255;
      color[1] = posSlope * value + 0;
      color[2] = 0;
    } else if (value < 120) {
      color[0] = negSlope * value + 2 * 255;
      color[1] = 255;
      color[2] = 0;
    } else if (value < 180) {
      color[0] = 0;
      color[1] = 255;
      color[2] = posSlope * value - 2 * 255;
    } else if (value < 240) {
      color[0] = 0;
      color[1] = negSlope * value + 4 * 255;
      color[2] = 255;
    } else if (value < 300) {
      color[0] = posSlope * value - 4 * 255;
      color[1] = 0;
      color[2] = 255;
    } else {
      color[0] = 255;
      color[1] = 0;
      color[2] = negSlope * value + 6 * 255;
    }
    return cv::Scalar(color[0], color[1], color[2]);
  }

  void ProjectToRawMat(cv::Mat img, cv::Mat K, cv::Mat D, cv::Mat R, cv::Mat T,
                       cv::Mat &current_frame, cv::Mat &bv_frame) {
    cv::Mat I = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat mapX, mapY;
    cv::Mat outImg = cv::Mat(img.size(), CV_32FC3);
    cv::initUndistortRectifyMap(K, D, I, K, img.size(), CV_32FC1, mapX, mapY);
    cv::remap(img, outImg, mapX, mapY, cv::INTER_LINEAR);
    cv::Mat dist = oriCloud.rowRange(0, 1).mul(oriCloud.rowRange(0, 1)) +
                   oriCloud.rowRange(1, 2).mul(oriCloud.rowRange(1, 2)) +
                   oriCloud.rowRange(2, 3).mul(oriCloud.rowRange(2, 3));
    cv::Mat R_ = R;
    cv::Mat T_ = T;

    // cv::Mat projCloud2d =
    //     K * (R_ * oriCloud + repeat(T_, 1, oriCloud.cols));
    cv::Mat transCloud2d = R_ * oriCloud + repeat(T_, 1, oriCloud.cols);
    float maxDist = 0;
    float maxx = 0;
    std::vector<Pt> points;
    std::vector<cv::Point2f> bv_radar_pts;
    for (int32_t i = 0; i < transCloud2d.cols; ++i) {
      float x = transCloud2d.at<float>(0, i);
      float y = transCloud2d.at<float>(1, i);
      float z = transCloud2d.at<float>(2, i);

      Eigen::Vector3d world_pt(x, z, 1);
      // x, z, 1???
      Eigen::Vector3d bv_pt = homograph_.inverse().eval() * world_pt;
      bv_pt /= bv_pt(2);
      // cv::Point result(cvRound(bv_pt(0)), cvRound(bv_pt(1)));
      int x2d = cvRound(bv_pt(0));
      int y2d = cvRound(bv_pt(1));
      float d = sqrt(dist.at<float>(0, i));
      if (x2d >= 0 && y2d >= 0 && x2d < img.cols && y2d < img.rows && z > 0) {
        maxx = std::max(maxx, std::fabs(x));
        maxDist = std::max(maxDist, d);
        points.push_back(Pt{cv::Point(x2d, y2d), d, std::fabs(x)});
        bv_radar_pts.push_back(cv::Point2f(x, z));
      }
    }

    bv_frame = bv_handler.drawProjectBirdView(point_size_, bv_radar_pts);

    sort(points.begin(), points.end(),
         [](const Pt &a, const Pt &b) { return a.dist > b.dist; });
    for (size_t i = 0; i < points.size(); ++i) {
      cv::Scalar color;
      // distance
      float d = points[i].dist;
      float x = points[i].x;
      // color = fakeColor(d / maxDist);
      color = fakeColor(x / maxx);
      circle(outImg, points[i].point, point_size_, color, -1);
    }
    current_frame = outImg;
  }

  // cv::Mat ProjectToRawImage(cv::Mat img,
  //                           Eigen::Matrix3d K,
  //                           std::vector<double> D,
  //                           Eigen::Matrix4d json_param) {
  void ProjectToRawImage(const cv::Mat &img, const Eigen::Matrix3d &K,
                         const std::vector<double> &D,
                         const Eigen::Matrix4d &json_param,
                         cv::Mat &current_frame, cv::Mat &bv_frame) {
    cv::Mat K1, D1, R1, T1;
    float k[9], d[8], r[9], t[3];

    k[0] = K(0, 0);
    k[1] = K(0, 1);
    k[2] = K(0, 2);
    k[3] = K(1, 0);
    k[4] = K(1, 1);
    k[5] = K(1, 2);
    k[6] = K(2, 0);
    k[7] = K(2, 1);
    k[8] = K(2, 2);

    // d[0] = D(0);
    // d[1] = D(1);
    // d[2] = D(2);
    // d[3] = D(3);
    for (size_t i = 0; i < D.size(); i++) {
      d[i] = D[i];
    }

    r[0] = json_param(0, 0);
    r[1] = json_param(0, 1);
    r[2] = json_param(0, 2);
    r[3] = json_param(1, 0);
    r[4] = json_param(1, 1);
    r[5] = json_param(1, 2);
    r[6] = json_param(2, 0);
    r[7] = json_param(2, 1);
    r[8] = json_param(2, 2);

    t[0] = json_param(0, 3);
    t[1] = json_param(1, 3);
    t[2] = json_param(2, 3);

    K1 = cv::Mat(3, 3, CV_32FC1, k);
    D1 = cv::Mat(D.size(), 1, CV_32FC1, d);
    R1 = cv::Mat(3, 3, CV_32FC1, r);
    T1 = cv::Mat(3, 1, CV_32FC1, t);
    // cv::Mat img = cv::imread(imgName);
    ProjectToRawMat(img, K1, D1, R1, T1, current_frame, bv_frame);
  }

  cv::Mat ProjectToFisheyeMat(cv::Mat img, cv::Mat K, cv::Mat D, cv::Mat R,
                              cv::Mat T) {
    cv::Mat I = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat mapX, mapY;
    cv::Mat outImg = cv::Mat(img.size(), CV_32FC3);
    // if broader view needed, change the new_K and new_Size
    cv::fisheye::initUndistortRectifyMap(K, D, I, K, img.size(), CV_32FC1, mapX,
                                         mapY);
    cv::remap(img, outImg, mapX, mapY, cv::INTER_LINEAR);
    cv::Mat dist = oriCloud.rowRange(0, 1).mul(oriCloud.rowRange(0, 1)) +
                   oriCloud.rowRange(1, 2).mul(oriCloud.rowRange(1, 2)) +
                   oriCloud.rowRange(2, 3).mul(oriCloud.rowRange(2, 3));
    cv::Mat R_ = R;
    cv::Mat T_ = T;

    cv::Mat projCloud2d = K * (R_ * oriCloud + repeat(T_, 1, oriCloud.cols));
    float maxDist = 0;

    std::vector<Pt> points;
    for (int32_t i = 0; i < projCloud2d.cols; ++i) {
      float x = projCloud2d.at<float>(0, i);
      float y = projCloud2d.at<float>(1, i);
      float z = projCloud2d.at<float>(2, i);
      int32_t x2d = cvRound(x / z);
      int32_t y2d = cvRound(y / z);
      float d = sqrt(dist.at<float>(0, i));
      if (x2d >= 0 && y2d >= 0 && x2d < img.cols && y2d < img.rows && z > 0) {
        maxDist = std::max(maxDist, d);
        points.push_back(Pt{cv::Point(x2d, y2d), d});
      }
    }
    sort(points.begin(), points.end(),
         [](const Pt &a, const Pt &b) { return a.dist > b.dist; });
    for (int32_t i = 0; i < static_cast<int32_t>(points.size()); ++i) {
      cv::Scalar color;
      // distance
      float d = points[i].dist;
      color = fakeColor(d / maxDist);
      circle(outImg, points[i].point, point_size_, color, -1);
    }
    return outImg;
  }
  cv::Mat ProjectToFisheyeImage(std::string imgName, cv::Mat K, cv::Mat D,
                                cv::Mat R, cv::Mat T) {
    cv::Mat img = cv::imread(imgName);
    return ProjectToFisheyeMat(img, K, D, R, T);
  }
};
