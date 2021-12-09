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

#include "pcl/io/pcd_io.h"

#define OVERLAP_FILTER_WINDOW 4
#define OVERLAP_DEPTH_TH 0.4 // 0.4m

struct Pt {
  cv::Point point;
  float dist;
  float z;
  float intensity;
};

class Projector {
public:
  cv::Mat oriCloud;
  std::vector<float> intensitys;
  const float ROI[6] = {-4, 3.5, 5.0, 10.0, -2.1, 3.0};
  int point_size_ = 3;
  bool intensity_color_ = false;
  bool overlap_filter_ = false;

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
  void setDisplayMode(bool intensity_show) {
    intensity_color_ = intensity_show;
  }
  void setFilterMode(bool filter_mode) { overlap_filter_ = filter_mode; }

  bool loadPointCloud(pcl::PointCloud<pcl::PointXYZI> pcl) {
    oriCloud = cv::Mat(cv::Size(pcl.points.size(), 3), CV_32FC1);
    for (size_t i = 0; i < pcl.points.size(); ++i) {
      oriCloud.at<float>(0, i) = pcl.points[i].x;
      oriCloud.at<float>(1, i) = pcl.points[i].y;
      oriCloud.at<float>(2, i) = pcl.points[i].z;
      intensitys.push_back(pcl.points[i].intensity);
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

  cv::Mat ProjectToRawMat(cv::Mat img, cv::Mat K, cv::Mat D, cv::Mat R,
                          cv::Mat T) {
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

    cv::Mat projCloud2d = K * (R_ * oriCloud + repeat(T_, 1, oriCloud.cols));
    float maxDist = 0;
    float maxIntensity = 0;
    std::vector<Pt> points;
    std::vector<std::vector<int>> filter_pts(img.rows,
                                             std::vector<int>(img.cols, -1));

    for (int32_t i = 0; i < projCloud2d.cols; ++i) {
      float x = projCloud2d.at<float>(0, i);
      float y = projCloud2d.at<float>(1, i);
      float z = projCloud2d.at<float>(2, i);
      int x2d = cvRound(x / z);
      int y2d = cvRound(y / z);
      float d = sqrt(dist.at<float>(0, i));
      float intensity = intensitys[i];

      if (x2d >= 0 && y2d >= 0 && x2d < img.cols && y2d < img.rows && z > 0) {
        maxDist = std::max(maxDist, d);
        maxIntensity = std::max(maxIntensity, intensity);
        points.push_back(Pt{cv::Point(x2d, y2d), d, z, intensity});
        // add size
        if (filter_pts[y2d][x2d] != -1) {
          int32_t p_idx = filter_pts[y2d][x2d];
          if (z < points[p_idx].z)
            filter_pts[y2d][x2d] = points.size() - 1;
        } else
          filter_pts[y2d][x2d] = points.size() - 1;
      }
    }
    if (overlap_filter_) {
      std::vector<int> filtered_idxes;
      for (int32_t m = 0; m < img.rows; m++) {
        for (int32_t n = 0; n < img.cols; n++) {
          int current_idx = filter_pts[m][n];
          if (current_idx == -1)
            continue;
          // search window
          bool front = true;
          for (int j = std::max(0, m - OVERLAP_FILTER_WINDOW);
               j < std::min(img.rows, m + OVERLAP_FILTER_WINDOW + 1); j++) {
            for (int k = std::max(0, n - OVERLAP_FILTER_WINDOW);
                 k < std::min(img.cols, n + OVERLAP_FILTER_WINDOW + 1); k++) {
              if (filter_pts[j][k] == -1)
                continue;
              int32_t p_idx = filter_pts[j][k];
              if (points[current_idx].z - points[p_idx].z > OVERLAP_DEPTH_TH) {
                front = false;
                break;
              }
            }
          }
          if (front)
            filtered_idxes.push_back(current_idx);
        }
      }
      for (size_t i = 0; i < filtered_idxes.size(); ++i) {
        int pt_idx = filtered_idxes[i];
        cv::Scalar color;
        if (intensity_color_) {
          // intensity
          float intensity = points[pt_idx].intensity;
          color = fakeColor(intensity / maxIntensity);
        } else {
          // distance
          float d = points[pt_idx].dist;
          color = fakeColor(d / maxDist);
        }
        circle(outImg, points[pt_idx].point, point_size_, color, -1);
      }
    } else {
      sort(points.begin(), points.end(),
           [](const Pt &a, const Pt &b) { return a.dist > b.dist; });
      for (size_t i = 0; i < points.size(); ++i) {
        cv::Scalar color;
        if (intensity_color_) {
          // intensity
          float intensity = points[i].intensity;
          color = fakeColor(intensity / maxIntensity);
        } else {
          // distance
          float d = points[i].dist;
          color = fakeColor(d / maxDist);
        }
        circle(outImg, points[i].point, point_size_, color, -1);
      }
    }
    return outImg;
  }

  cv::Mat ProjectToRawImage(cv::Mat img, Eigen::Matrix3d K,
                            std::vector<double> D, Eigen::Matrix4d json_param) {
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
    return ProjectToRawMat(img, K1, D1, R1, T1);
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
    float maxIntensity = 0;

    std::vector<Pt> points;
    for (int32_t i = 0; i < projCloud2d.cols; ++i) {
      float x = projCloud2d.at<float>(0, i);
      float y = projCloud2d.at<float>(1, i);
      float z = projCloud2d.at<float>(2, i);
      int32_t x2d = cvRound(x / z);
      int32_t y2d = cvRound(y / z);
      float d = sqrt(dist.at<float>(0, i));
      float intensity = intensitys[i];
      if (x2d >= 0 && y2d >= 0 && x2d < img.cols && y2d < img.rows && z > 0) {
        maxDist = std::max(maxDist, d);
        maxIntensity = std::max(maxIntensity, intensity);
        points.push_back(Pt{cv::Point(x2d, y2d), d, z, intensity});
      }
    }
    sort(points.begin(), points.end(),
         [](const Pt &a, const Pt &b) { return a.dist > b.dist; });
    for (int32_t i = 0; i < static_cast<int32_t>(points.size()); ++i) {
      cv::Scalar color;
      if (intensity_color_) {
        // intensity
        float intensity = points[i].intensity;
        color = fakeColor(intensity / maxIntensity);
      } else {
        // distance
        float d = points[i].dist;
        color = fakeColor(d / maxDist);
      }
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
