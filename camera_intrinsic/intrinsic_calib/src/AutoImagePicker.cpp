/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#include "AutoImagePicker.hpp"

#define CHESSBOARD_MIN_AREA_PORTION 0.04
#define IMAGE_MARGIN_PERCENT 0.1
#define CHESSBOARD_MIN_AREA_CHANGE_PARTION_THRESH 0.005
#define CHESSBOARD_MIN_MOVE_THRESH 0.08
#define CHESSBOARD_MIN_ANGLE_CHANGE_THRESH 5.0
#define CHESSBOARD_MIN_ANGLE 40.0
#define MAX_SELECTED_SAMPLE_NUM 45
#define REGION_MAX_SELECTED_SAMPLE_NUM 15

AutoImagePicker::AutoImagePicker(const int &img_width, const int &img_height,
                                 const int &board_width,
                                 const int &board_height) {
  img_width_ = img_width;
  img_height_ = img_height;
  board_width_ = board_width;
  board_height_ = board_height;

  area_box_ = std::vector<std::vector<int>>(img_height_,
                                            std::vector<int>(img_width_, 5));
  // encourage edge position
  int d_imgh = static_cast<int>(IMAGE_MARGIN_PERCENT * img_height);
  int d_imgw = static_cast<int>(IMAGE_MARGIN_PERCENT * img_width);
  for (int i = d_imgh; i < img_height - d_imgh; i++) {
    for (int j = d_imgw; j < img_width - d_imgw; j++) {
      area_box_[i][j] = 1;
    }
  }
  candidate_board_.clear();
  board_square_box_.clear();
}

bool AutoImagePicker::addImage(const std::vector<cv::Point2f> &image_corners) {
  cv::Point p1 = image_corners[0];
  cv::Point p2 = image_corners[0 + board_width_ - 1];
  cv::Point p3 = image_corners[image_corners.size() - board_width_];
  cv::Point p4 = image_corners[image_corners.size() - 1];
  BoardSquare current_square(p1, p2, p3, p4);
  if (!checkValidity(current_square))
    return false;
  if (checkMoveThresh(current_square) || checkAreaThresh(current_square)) {
    // current_square.index = candidate_board_.size();
    candidate_board_.push_back(current_square);
    this->fillAreaBox(current_square);
  } else {
    if (!checkPoseAngleThresh(current_square))
      return false;
    candidate_board_.push_back(current_square);
    this->fillAreaBox(current_square);
  }
  return true;
}

bool AutoImagePicker::status() {
  if (candidate_board_.size() >= MAX_SELECTED_SAMPLE_NUM) {
    std::cout << "[ImagePicker] Enough selected images.\n";
    return true;
  }
  return false;
}

bool AutoImagePicker::checkValidity(const BoardSquare &board) {
  if (board.area < CHESSBOARD_MIN_AREA_PORTION * img_width_ * img_height_)
    return false; // board is too far
  if (board.angle_left_top < CHESSBOARD_MIN_ANGLE ||
      board.angle_right_top < CHESSBOARD_MIN_ANGLE ||
      board.angle_left_bottom < CHESSBOARD_MIN_ANGLE ||
      board.angle_right_bottom < CHESSBOARD_MIN_ANGLE)
    return false; // board is too inclined
  return true;
}

bool AutoImagePicker::checkMoveThresh(const BoardSquare &board) {
  for (size_t i = 0; i < candidate_board_.size(); i++) {
    auto candidate = candidate_board_[i];
    double dx = board.midpoint.x - candidate.midpoint.x;
    double dy = board.midpoint.y - candidate.midpoint.y;
    double dist = sqrt(dx * dx + dy * dy);
    if (dist <
        CHESSBOARD_MIN_MOVE_THRESH * double(img_width_ + img_height_) / 2.0)
      return false;
  }
  return true;
}

bool AutoImagePicker::checkAreaThresh(const BoardSquare &board) {
  int score = 0;
  for (int i = std::max(0, int(board.min_y));
       i < std::min(img_height_, int(board.max_y)); i++) {
    for (int j = std::max(0, int(board.min_x));
         j < std::min(img_width_, int(board.max_x)); j++) {
      score += area_box_[i][j];
    }
  }
  if (score <
      CHESSBOARD_MIN_AREA_CHANGE_PARTION_THRESH * img_width_ * img_height_)
    return false;
  else
    return true;
}

bool AutoImagePicker::checkPoseAngleThresh(const BoardSquare &board) {
  for (size_t i = 0; i < candidate_board_.size(); i++) {
    auto candidate = candidate_board_[i];
    double dx = board.midpoint.x - candidate.midpoint.x;
    double dy = board.midpoint.y - candidate.midpoint.y;
    double dist = sqrt(dx * dx + dy * dy);
    if (dist <
        CHESSBOARD_MIN_MOVE_THRESH * double(img_width_ + img_height_) / 2.0) {
      if (fabs(board.angle_left_top - candidate.angle_left_top) <
              CHESSBOARD_MIN_ANGLE_CHANGE_THRESH &&
          fabs(board.angle_right_top - candidate.angle_right_top) <
              CHESSBOARD_MIN_ANGLE_CHANGE_THRESH &&
          fabs(board.angle_left_bottom - candidate.angle_left_bottom) <
              CHESSBOARD_MIN_ANGLE_CHANGE_THRESH &&
          fabs(board.angle_right_bottom - candidate.angle_right_bottom) <
              CHESSBOARD_MIN_ANGLE_CHANGE_THRESH)
        return false;
    }
  }
  return true;
}

int AutoImagePicker::getAreaScore(const BoardSquare &board) {
  int score = 0;
  for (int i = std::max(0, int(board.min_y));
       i < std::min(img_height_, int(board.max_y)); i++) {
    for (int j = std::max(0, int(board.min_x));
         j < std::min(img_width_, int(board.max_x)); j++) {
      score += area_box_[i][j];
    }
  }
  return score;
}

void AutoImagePicker::fillAreaBox(const BoardSquare &board) {
  for (int i = std::max(0, int(board.min_y));
       i < std::min(img_height_, int(board.max_y)); i++) {
    for (int j = std::max(0, int(board.min_x));
         j < std::min(img_width_, int(board.max_x)); j++) {
      area_box_[i][j] = 0;
    }
  }
}