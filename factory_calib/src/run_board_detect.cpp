/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#include "Eigen/Core"
#include "calibration_board.hpp"
#include "logging.hpp"
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
// chessboard
#include "chessboard/corner_detect.hpp"
// vertical board
#include "vertical_board/corner_detect.hpp"
// circle board
#include "circle_board/corner_detect.hpp"
// apriltag
#include "apriltags/Tag36h11.h"
#include "apriltags/TagDetector.h"
#include "aruco_marker/corner_detect.hpp"

// 0: chessboard
// 1: vertical board
// 2: circle board
// 3: aruco marker board(4 markers)
// 4: apriltag board
char usage[] = {"[Usage]: ./bin/run_board_detect image board_type \n"
                "eg: ./bin/run_board_detect data/chessboard.jpg 0 \n"};

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage:" << usage;
    return -1;
  }
  std::string image_path = argv[1];
  std::string board_type = argv[2];
  cv::Mat image = cv::imread(image_path);
  int type = stoi(board_type);
  if (type < 0 && type > 4) {
    LOGE("board type invalid, only support 0~4");
    return -1;
  }
  // convert image to gray vector
  cv::Mat gray, gray_img;
  cv::cvtColor(image, gray, CV_BGR2GRAY);
  gray.convertTo(gray_img, CV_32F);
  std::vector<std::vector<float>> gray_vec;
  cvmat2stdvec(gray_img, gray_vec);
  bool display_img = true;
  bool sta = false;
  if (type == 0) {
    int board_center_point_x = 0;
    int board_center_point_y = 0;
    int grid_x_dis = 0;
    int grid_y_dis = 0;
    Point slope_point1(0, 0);
    Point slope_point2(0, 0);
    std::vector<Point> grid_center_points;
    CornerDetect corner_detect;
    LOGI("here");
    sta = corner_detect.CornerDetection(
        gray_vec, &board_center_point_x, &board_center_point_y, &slope_point1,
        &slope_point2, &grid_x_dis, &grid_y_dis, &grid_center_points);
    LOGI("here");
    // display
    if (display_img && sta) {
      std::vector<cv::Scalar> color_box;
      color_box.emplace_back(cv::Scalar(235, 80, 80));
      color_box.emplace_back(cv::Scalar(205, 200, 0));
      color_box.emplace_back(cv::Scalar(80, 235, 80));
      color_box.emplace_back(cv::Scalar(0, 200, 200));
      color_box.emplace_back(cv::Scalar(80, 80, 235));
      color_box.emplace_back(cv::Scalar(200, 0, 200));
      cv::Mat display_img = image.clone();
      for (int i = 0; i < grid_center_points.size(); ++i) {
        cv::Point2f cv_pt;
        cv_pt.x = grid_center_points[i].x;
        cv_pt.y = grid_center_points[i].y;
        cv::circle(display_img, cv_pt, 3, cv::Scalar(60, 60, 230), -1);
      }
      cv::imshow("chessboard", display_img);
      cv::waitKey();
      cv::imwrite("chessboard_detection.png", display_img);
    }
  } else if (type == 1) {
    int calibration_board_type = 1;
    cameracalib::VerticalBoard v_board;
    v_board.set(calibration_board_type);
    // check board pattern
    if (!v_board.check()) {
      LOGE("wrong param");
      return false;
    }
    // corner detect
    cameracalib::verticalBoard::CornerDetector vboard_detector;
    cameracalib::verticalBoard::CornerPoints corners;
    vboard_detector.image_ = image.clone();
    sta = vboard_detector.detect(gray_vec, v_board, &corners);
    // display
    if (display_img && sta) {
      std::vector<cv::Scalar> color_box;
      color_box.emplace_back(cv::Scalar(235, 80, 80));
      color_box.emplace_back(cv::Scalar(205, 200, 0));
      color_box.emplace_back(cv::Scalar(80, 235, 80));
      color_box.emplace_back(cv::Scalar(0, 200, 200));
      color_box.emplace_back(cv::Scalar(80, 80, 235));
      color_box.emplace_back(cv::Scalar(200, 0, 200));
      cv::Mat display_img = image.clone();
      for (int i = 0; i < corners.lines_.size(); ++i) {
        std::vector<Eigen::Vector2d> l_pts = corners.lines_[i]->m_line_pts;
        cv::Scalar color = color_box[i % 6];
        for (auto p : l_pts) {
          cv::Point2f cv_pt;
          cv_pt.x = p(0);
          cv_pt.y = p(1);
          cv::circle(display_img, cv_pt, 3, color, -1);
        }
      }
      cv::imshow("verticalboard", display_img);
      cv::waitKey();
      cv::imwrite("verticalboard_detection.png", display_img);
    }
  } else if (type == 2) {
    cameracalib::CircleBoard c_board;
    // corner detect
    cameracalib::circleBoard::CornerDetector cboard_detector;
    cameracalib::circleBoard::CornerPoints corners;
    sta = cboard_detector.detect(gray_vec, c_board, &corners);
    if (display_img && sta) {
      std::vector<cv::Scalar> color_box;
      color_box.emplace_back(cv::Scalar(235, 80, 80));
      color_box.emplace_back(cv::Scalar(205, 200, 0)); // 5: round hole board
      color_box.emplace_back(cv::Scalar(80, 235, 80));
      color_box.emplace_back(cv::Scalar(0, 200, 200));
      color_box.emplace_back(cv::Scalar(80, 80, 235));
      color_box.emplace_back(cv::Scalar(200, 0, 200));
      cv::Mat display_img = image.clone();
      for (int i = 0; i < corners.points.size(); ++i) {
        for (int j = 0; j < corners.points[i].size(); ++j) {
          Point2f pt = corners.points[i][j];
          if (pt.x < 1)
            continue;
          cv::Point2f cv_pt;
          cv::Point2f cv_pt_text;
          cv_pt.x = pt.x;
          cv_pt.y = pt.y;
          cv_pt_text = cv_pt;
          cv_pt_text.x += 10;
          cv::Scalar color = color_box[j % 6];
          cv::circle(display_img, cv_pt, 4, color, -1);
          cv::putText(display_img, std::to_string(i), cv_pt_text,
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, color);
        }
      }
      cv::imshow("circleboard", display_img);
      cv::waitKey();
      cv::imwrite("circleboard_detection.png", display_img);
    }
  } else if (type == 3) {
    cameracalib::ArucoMarker aruco_marker;
    // corner detect
    cameracalib::arucomarker::ArucoMarkerDetector board_detector;
    cameracalib::arucomarker::CornerPoints corners;
    sta = board_detector.detect(gray_vec, aruco_marker, &corners);
    if (display_img && sta) {
      cv::Mat display_img = image.clone();
      for (size_t i = 0; i < corners.points.size(); ++i) {
        auto pts = corners.points[i];
        if (pts.size() != 4) {
          LOGE("failed to detect corner");
          return false;
        }
        cv::Point p0(pts[0].x, pts[0].y);
        cv::Point p1(pts[1].x, pts[1].y);
        cv::Point p2(pts[2].x, pts[2].y);
        cv::Point p3(pts[3].x, pts[3].y);
        cv::circle(display_img, p0, 3, cv::Scalar(230, 100, 100), -1);
        cv::circle(display_img, p1, 3, cv::Scalar(100, 230, 100), -1);
        cv::circle(display_img, p2, 3, cv::Scalar(100, 100, 230), -1);
        cv::circle(display_img, p3, 3, cv::Scalar(210, 210, 70), -1);
        // identify id
        cv::Point p(pts[0].x - 20, pts[0].y);
        cv::putText(display_img, std::to_string(corners.tag_ids[i]), p,
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(230, 100, 100));
      }
      cv::imshow("arucomarker", display_img);
      cv::waitKey();
      cv::imwrite("arucomarker_detection.png", display_img);
    }
  } else if (type == 4) {
    AprilTags::TagCodes m_tagCodes(AprilTags::tagCodes36h11);
    AprilTags::TagDetector *m_tagDetector =
        new AprilTags::TagDetector(m_tagCodes);
    vector<AprilTags::TagDetection> detections =
        m_tagDetector->extractTags(gray);
    if (detections.size() == 36)
      sta = true;
    if (display_img && sta) {
      for (int i = 0; i < detections.size(); i++) {
        // also highlight in the image
        detections[i].draw(image);
      }
      cv::imshow("apriltags", image);
      cv::waitKey();
      cv::imwrite("apriltags_detection.png", image);
    }
  }
  if (sta) {
    std::cout << "\nDetection Success!\n";
  } else {
    std::cout << "\nDetection Failed!\n";
  }

  return 0;
}
