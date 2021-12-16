/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */

#include "calibration/adas_calib.hpp"
#include "calibration_board.hpp"
#include "calibration/find_homography.hpp"
#include "logging.hpp"
#include "calibration/pnp_solver.hpp"
#include <jsoncpp/json/json.h>
// chessboard
#include "chessboard/corner_detect.hpp"
// vertical board
#include "vertical_board/corner_detect.hpp"
// circle board
#include "circle_board/corner_detect.hpp"
// apriltag
#include "aruco_marker/corner_detect.hpp"

namespace cameracalib {

bool CalibrationBoardTool::ChessBoardCalibration(
    const std::vector<std::vector<float>> &gray_img,
    const InputParam &calibration_inputs, const std::string &output_json_path,
    CalibrationOutputParams *calibration_result) {
  if (calibration_result == nullptr) {
    return false;
  }
  // copy input param to base calibration param struct
  base_input_param_.camera_height = calibration_inputs.camera_height;
  base_input_param_.camera_lateral_offset =
      calibration_inputs.camera_lateral_offset;
  base_input_param_.camera_vertical_offset =
      calibration_inputs.camera_vertical_offset;
  base_input_param_.camera_focus_length_fx =
      calibration_inputs.camera_focus_length_fx;
  base_input_param_.camera_focus_length_fy =
      calibration_inputs.camera_focus_length_fy;
  base_input_param_.img_width = gray_img[0].size();
  base_input_param_.img_height = gray_img.size();

  int board_center_point_x = 0;
  int board_center_point_y = 0;
  int grid_x_dis = 0;
  int grid_y_dis = 0;
  Point slope_point1(0, 0);
  Point slope_point2(0, 0);
  std::vector<Point> grid_center_points;

  // corner detect
  CornerDetect corner_detect;
  bool is_success = corner_detect.CornerDetection(
      gray_img, &board_center_point_x, &board_center_point_y, &slope_point1,
      &slope_point2, &grid_x_dis, &grid_y_dis, &grid_center_points);
  if (!is_success) {
    LOGE("failed to detect corners");
    return false;
  }
  // display
  if (display_img_) {
    std::vector<cv::Scalar> color_box;
    color_box.emplace_back(cv::Scalar(235, 80, 80));
    color_box.emplace_back(cv::Scalar(205, 200, 0));
    color_box.emplace_back(cv::Scalar(80, 235, 80));
    color_box.emplace_back(cv::Scalar(0, 200, 200));
    color_box.emplace_back(cv::Scalar(80, 80, 235));
    color_box.emplace_back(cv::Scalar(200, 0, 200));
    cv::Mat display_img = image_.clone();
    for (int i = 0; i < grid_center_points.size(); ++i) {
      cv::Point2f cv_pt;
      cv_pt.x = grid_center_points[i].x;
      cv_pt.y = grid_center_points[i].y;
      cv::circle(display_img, cv_pt, 3, cv::Scalar(60, 60, 230), -1);
    }
    cv::imshow("chessboard", display_img);
    cv::waitKey();
    cv::imwrite(output_dir_ + "/chessboard.png", display_img);
  }

  // sort by x
  for (size_t i = 0; i < grid_center_points.size() - 1; i++) {
    for (size_t j = 0; j < grid_center_points.size() - 1 - i; j++) {
      Point point_temp;
      if (grid_center_points[j].x > grid_center_points[j + 1].x) {
        point_temp = grid_center_points[j];
        grid_center_points[j] = grid_center_points[j + 1];
        grid_center_points[j + 1] = point_temp;
      }
    }
  }

  std::vector<Point2float> img_quad;
  std::vector<Point2float> world_quad;
  Mat3d homo_mat;
  // prepare the point in the image
  for (size_t i = 0; i < grid_center_points.size(); i++) {
    img_quad.push_back(
        Point2float(grid_center_points[i].x, grid_center_points[i].y));
  }

  // prepare the corresponding point in the world
  float distance_between_two_chessboard =
      calibration_inputs.distance_between_two_chessboard;
  float grid_size = calibration_inputs.chessboard_grid_size;

  Point2f p1, p2, p3, p4, p5, p6;
  p1.x = -(distance_between_two_chessboard / 2 + 2.5 * grid_size);
  p1.y = 0.5 * grid_size;
  p2.x = -(distance_between_two_chessboard / 2 + 1.5 * grid_size);
  p2.y = -0.5 * grid_size;
  p3.x = -(distance_between_two_chessboard / 2 + 0.5 * grid_size);
  p3.y = 0.5 * grid_size;

  p4.x = distance_between_two_chessboard / 2 + 0.5 * grid_size;
  p4.y = -0.5 * grid_size;
  p5.x = distance_between_two_chessboard / 2 + 1.5 * grid_size;
  p5.y = 0.5 * grid_size;
  p6.x = distance_between_two_chessboard / 2 + 2.5 * grid_size;
  p6.y = -0.5 * grid_size;

  world_quad.push_back(Point2float(p1.x, p1.y));
  world_quad.push_back(Point2float(p2.x, p2.y));
  world_quad.push_back(Point2float(p3.x, p3.y));
  world_quad.push_back(Point2float(p4.x, p4.y));
  world_quad.push_back(Point2float(p5.x, p5.y));
  world_quad.push_back(Point2float(p6.x, p6.y));

  float expected_symmetric_distance = 0.02;
  bool homography_flag = CeresSolveHomography(
      img_quad, world_quad, expected_symmetric_distance, &homo_mat);
  if (!homography_flag) {
    LOGE("failed to compute homography");
    return false;
  }

  /* solve the final vanishing point */
  typedef Eigen::Matrix<double, 3, 1> Vec3;
  float camera_vertical_pos =
      calibration_inputs.camera_height - calibration_inputs.board_height;
  Vec3 camera_pos(-calibration_inputs.camera_lateral_offset,
                  camera_vertical_pos, 1);
  Vec3 vanishing_point = homo_mat.inverse() * camera_pos;
  vanishing_point /= vanishing_point(2, 0);
  calibration_result->vanishing_pt(0) = vanishing_point(0, 0);
  calibration_result->vanishing_pt(1) = vanishing_point(1, 0);

  // prepare ground A,B points
  float board_height = calibration_inputs.board_height;
  float camera_to_board_distance = calibration_inputs.camera_to_board_distance;
  float camera_vertical_offset = calibration_inputs.camera_vertical_offset;
  float car_center_to_board = camera_to_board_distance + camera_vertical_offset;
  if (grid_size == 0) {
    LOGE("wrong param");
    return false;
  }
  Point2f pA_chessboard, pB_chessboard;
  pA_chessboard.x = -(distance_between_two_chessboard / 2 + 1.0 * grid_size);
  pA_chessboard.y = -board_height;
  pB_chessboard.x = (distance_between_two_chessboard / 2 + 1.0 * grid_size);
  pB_chessboard.y = -board_height;

  std::vector<Point2float> img_quad_ground;
  std::vector<Point2float> world_quad_ground;

  // prepare A,B point in the image

  Vec3 pA = homo_mat.inverse() * Vec3(pA_chessboard.x, pA_chessboard.y, 1);
  if (pA(2, 0) == 0) {
    LOGE("wrong param");
    return false;
  }
  pA /= pA(2, 0);
  Vec3 pB = homo_mat.inverse() * Vec3(pB_chessboard.x, pB_chessboard.y, 1);
  if (pB(2, 0) == 0) {
    LOGE("wrong param");
    return false;
  }
  pB /= pB(2, 0);
  img_quad_ground.push_back(Point2float(pA(0, 0), pA(1, 0)));
  img_quad_ground.push_back(Point2float(pB(0, 0), pB(1, 0)));

  world_quad_ground.push_back(
      Point2float(car_center_to_board, -pA_chessboard.x));
  world_quad_ground.push_back(
      Point2float(car_center_to_board, -pB_chessboard.x));

  // prepare midpoints of A-VP and B-VP
  Point2f pA_mid_img((pA(0, 0) + vanishing_point(0, 0)) / 2,
                     (pA(1, 0) + vanishing_point(1, 0)) / 2);
  Point2f pB_mid_img((pB(0, 0) + vanishing_point(0, 0)) / 2,
                     (pB(1, 0) + vanishing_point(1, 0)) / 2);
  img_quad_ground.push_back(Point2float(pA_mid_img.x, pA_mid_img.y));
  img_quad_ground.push_back(Point2float(pB_mid_img.x, pB_mid_img.y));

  Point2f pA_mid_ground(0, -pA_chessboard.x);
  Point2f pB_mid_ground(0, -pB_chessboard.x);
  /*Method 2*/

  float camera_height = calibration_inputs.camera_height;
  if (camera_height == 0) {
    LOGE("wrong param");
    return false;
  }
  float camera_fy_a = (camera_to_board_distance) *
                      (pA(1, 0) - vanishing_point(1, 0)) / camera_height;
  float camera_fy_b = (camera_to_board_distance) *
                      (pB(1, 0) - vanishing_point(1, 0)) / camera_height;
  float camera_fy = (camera_fy_a + camera_fy_b) / 2.0;
  if (std::fabs(pA_mid_img.y - vanishing_point(1, 0)) < 1e-6 ||
      std::fabs(pB_mid_img.y - vanishing_point(1, 0)) < 1e-6) {
    LOGE("wrong param");
    return false;
  }
  pA_mid_ground.x =
      camera_height * camera_fy / (pA_mid_img.y - vanishing_point(1, 0)) +
      camera_vertical_offset;
  pB_mid_ground.x =
      camera_height * camera_fy / (pB_mid_img.y - vanishing_point(1, 0)) +
      camera_vertical_offset;

  world_quad_ground.push_back(Point2float(pA_mid_ground.x, pA_mid_ground.y));
  world_quad_ground.push_back(Point2float(pB_mid_ground.x, pB_mid_ground.y));

  /* solve the hmat of the ground */
  Eigen::Matrix<double, 3, 3> ground_hmat;
  homography_flag =
      CeresSolveHomography(img_quad_ground, world_quad_ground,
                           expected_symmetric_distance, &ground_hmat);
  if (!homography_flag) {
    LOGE("failed to compute homography");
    return false;
  }

  calibration_result->cam_homography = ground_hmat;
  // save fx,fy
  if (fabs(base_input_param_.camera_focus_length_fy - camera_fy) < 50 &&
      fabs(base_input_param_.camera_focus_length_fx - camera_fy) < 50) {
    base_input_param_.camera_focus_length_fy = camera_fy;
    base_input_param_.camera_focus_length_fx = camera_fy;
  }
  calibration_result_ = *calibration_result;
  this->SaveCalibrationResult(calibration_result_, output_json_path);

  return true;
}

bool CalibrationBoardTool::VerticalBoardCalibration(
    const std::vector<std::vector<float>> &gray_img,
    const InputParam &calibration_inputs, const std::string &output_json_path,
    CalibrationOutputParams *calibration_result) {
  if (calibration_result == nullptr) {
    LOGE("wrong param");
    return false;
  }
  // copy input param to base calibration param struct
  base_input_param_.camera_height = calibration_inputs.camera_height;
  base_input_param_.camera_lateral_offset =
      calibration_inputs.camera_lateral_offset;
  base_input_param_.camera_vertical_offset =
      calibration_inputs.camera_vertical_offset;
  base_input_param_.camera_focus_length_fx =
      calibration_inputs.camera_focus_length_fx;
  base_input_param_.camera_focus_length_fy =
      calibration_inputs.camera_focus_length_fy;
  base_input_param_.img_width = gray_img[0].size();
  base_input_param_.img_height = gray_img.size();

  // generate vertical board pattern
  VerticalBoard v_board;
  v_board.set(calibration_inputs.calibration_board_type);
  std::cout << "board type: " << calibration_inputs.calibration_board_type
            << std::endl;
  // check board pattern
  if (!v_board.check()) {
    LOGE("wrong param");
    return false;
  }

  // corner detect
  verticalBoard::CornerDetector vboard_detector;
  verticalBoard::CornerPoints corners;
  vboard_detector.image_ = image_.clone();
  bool is_success = vboard_detector.detect(gray_img, v_board, &corners);

  if (!is_success) {
    LOGE("failed to detect corners");
    return false;
  }
  // display
  if (display_img_) {
    std::vector<cv::Scalar> color_box;
    color_box.emplace_back(cv::Scalar(235, 80, 80));
    color_box.emplace_back(cv::Scalar(205, 200, 0));
    color_box.emplace_back(cv::Scalar(80, 235, 80));
    color_box.emplace_back(cv::Scalar(0, 200, 200));
    color_box.emplace_back(cv::Scalar(80, 80, 235));
    color_box.emplace_back(cv::Scalar(200, 0, 200));
    cv::Mat display_img = image_.clone();
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
    cv::imwrite(output_dir_ + "/verticalboard.png", display_img);
  }

  // compute roll degree
  float board_roll = 0;
  for (auto line : corners.lines_) {
    board_roll += line->getLineSlopeAngle();
  }
  board_roll /= 3.0;
  board_roll -= 90.0;
  float roll_rad = board_roll * M_PI / 180.0;
  double cos_roll = cos(-roll_rad);
  double sin_roll = sin(-roll_rad);

  // prepare corresponding board and img points
  std::vector<Point2float> img_quad;
  std::vector<Point2float> board_quad;
  Mat3d img2board_hmat;
  float grid_size = calibration_inputs.chessboard_grid_size;
  float camera_height = calibration_inputs.camera_height;
  int mid_corner_num = v_board.lines[1].size();
  float top_board_height =
      mid_corner_num * grid_size + calibration_inputs.board_height;
  float board_pt_x, board_pt_y;
  for (int i = 0; i < 3; ++i) {
    auto c_line = corners.lines_[i];
    int pt_num = c_line->m_line_pts.size();
    for (int j = 0; j < pt_num; ++j) {
      board_pt_x = (i - 1) * grid_size;
      board_pt_y = top_board_height - corners.pattern_idxs_[i][j] * grid_size -
                   camera_height;
      float img_x = c_line->m_line_pts[j](0);
      float img_y = c_line->m_line_pts[j](1);
      board_quad.emplace_back(Point2float(board_pt_x, board_pt_y));
      img_quad.emplace_back(Point2float(img_x, img_y));
    }
  }
  // compute image to board homography
  float expected_symmetric_distance = 0.02;
  bool homography_flag = CeresSolveHomography(
      img_quad, board_quad, expected_symmetric_distance, &img2board_hmat);
  if (!homography_flag) {
    LOGE("failed to compute homography");
    return false;
  }

  // compute virtual point
  Vec3d camera_pos(-calibration_inputs.camera_lateral_offset, 0, 1);
  Vec3d vp = img2board_hmat.inverse() * camera_pos;
  vp /= vp(2);
  calibration_result->vanishing_pt(0) = vp(0);
  calibration_result->vanishing_pt(1) = vp(1);

  // prepare virtual ground points and img points
  std::vector<Point2float> ground_img_pts;
  std::vector<Point2float> ground_world_pts;
  Mat3d img2ground_hmat;
  float x_ground_gap = 10;
  float y_ground_gap = 1.5;
  float fx = calibration_inputs.camera_focus_length_fx;
  float fy = calibration_inputs.camera_focus_length_fy;
  float cam_v_offset = calibration_inputs.camera_vertical_offset;
  float cam_l_offset = calibration_inputs.camera_lateral_offset;
  for (int a = 1; a <= 2; ++a) {
    for (int b = -1; b <= 1; b += 2) {
      float gd_x = a * x_ground_gap;
      float gd_y = b * y_ground_gap;
      float img_y = camera_height * fy / (gd_x - cam_v_offset) + vp(1);
      float img_x = vp(0) - fx / (gd_x - cam_v_offset) * (gd_y - cam_l_offset);
      // roll adjustment
      float new_img_x, new_img_y;
      float x2vp_x = img_x - vp(0);
      float y2vp_y = img_y - vp(1);
      new_img_x = x2vp_x * cos_roll - y2vp_y * sin_roll + vp(0);
      new_img_y = y2vp_y * cos_roll + x2vp_x * sin_roll + vp(1);
      ground_img_pts.emplace_back(Point2float(new_img_x, new_img_y));
      ground_world_pts.emplace_back(Point2float(gd_x, gd_y));
    }
  }

  // compute image to ground homography
  homography_flag =
      CeresSolveHomography(ground_img_pts, ground_world_pts,
                           expected_symmetric_distance, &img2ground_hmat);
  if (!homography_flag) {
    LOGE("failed to compute homography");
    return false;
  }

  calibration_result->cam_homography = img2ground_hmat;

  calibration_result_ = *calibration_result;
  this->SaveCalibrationResult(calibration_result_, output_json_path);

  return true;
}

bool CalibrationBoardTool::CircleBoardCalibration(
    const std::vector<std::vector<float>> &gray_img,
    const InputParam &calibration_inputs, const std::string &output_json_path,
    CalibrationOutputParams *calibration_result) {
  if (calibration_result == nullptr) {
    LOGE("wrong param");
    return false;
  }
  // copy input param to base calibration param struct
  base_input_param_.camera_height = calibration_inputs.camera_height;
  base_input_param_.camera_lateral_offset =
      calibration_inputs.camera_lateral_offset;
  base_input_param_.camera_vertical_offset =
      calibration_inputs.camera_vertical_offset;
  base_input_param_.camera_focus_length_fx =
      calibration_inputs.camera_focus_length_fx;
  base_input_param_.camera_focus_length_fy =
      calibration_inputs.camera_focus_length_fy;
  base_input_param_.img_width = gray_img[0].size();
  base_input_param_.img_height = gray_img.size();

  // generate vertical board pattern
  CircleBoard c_board;
  // check board pattern
  if (!c_board.check()) {
    LOGE("wrong param");
    return false;
  }

  // corner detect
  circleBoard::CornerDetector cboard_detector;
  circleBoard::CornerPoints corners;
  bool is_success = cboard_detector.detect(gray_img, c_board, &corners);
  if (!is_success) {
    LOGE("failed to detect corner");
    return false;
  }

  // display
  if (display_img_) {
    std::vector<cv::Scalar> color_box;
    color_box.emplace_back(cv::Scalar(235, 80, 80));
    color_box.emplace_back(cv::Scalar(205, 200, 0));
    color_box.emplace_back(cv::Scalar(80, 235, 80));
    color_box.emplace_back(cv::Scalar(0, 200, 200));
    color_box.emplace_back(cv::Scalar(80, 80, 235));
    color_box.emplace_back(cv::Scalar(200, 0, 200));
    cv::Mat display_img = image_.clone();
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
    cv::imwrite(output_dir_ + "/circleboard.png", display_img);
  }

  // compute roll degree
  float board_roll = 0;
  for (auto lslope : corners.line_slope) {
    board_roll += lslope;
  }
  board_roll /= static_cast<float>(corners.line_slope.size());
  board_roll -= 90.0;
  float roll_rad = board_roll * M_PI / 180.0;
  double cos_roll = cos(-roll_rad);
  double sin_roll = sin(-roll_rad);

  // prepare corresponding board and img points
  std::vector<Point2float> img_quad;
  std::vector<Point2float> board_quad;
  Mat3d img2board_hmat;
  // grid_size is point-to-point distance in vertical direction
  // point-to-point distance in lateral is default to be 2*grid_size
  float vertical_dist = calibration_inputs.chessboard_grid_size;
  float lateral_dist = 2 * vertical_dist;
  float camera_height = calibration_inputs.camera_height;
  float mid_width = (c_board.width - 1) * lateral_dist / 2.0;
  float mid_height = (c_board.height - 1) * vertical_dist / 2.0;
  float board_pt_x, board_pt_y;
  for (int i = 0; i < c_board.height; ++i) {
    for (int j = 0; j < c_board.width; ++j) {
      Point2f pt = corners.points[i][j];
      if (pt.x > 0 && pt.y > 0) {
        board_pt_x = j * lateral_dist - mid_width;
        board_pt_y = mid_height - i * vertical_dist;
        board_quad.emplace_back(Point2float(board_pt_x, board_pt_y));
        img_quad.emplace_back(Point2float(pt.x, pt.y));
      }
    }
  }
  // compute image to board homography
  float expected_symmetric_distance = 0.02;
  bool homography_flag = CeresSolveHomography(
      img_quad, board_quad, expected_symmetric_distance, &img2board_hmat);
  if (!homography_flag) {
    LOGE("failed to compute homography");
    return false;
  }

  float camera_vertical_pos =
      calibration_inputs.camera_height - calibration_inputs.board_height;
  // compute virtual point
  Vec3d camera_pos(-calibration_inputs.camera_lateral_offset,
                   camera_vertical_pos, 1);
  Vec3d vp = img2board_hmat.inverse() * camera_pos;
  vp /= vp(2);
  calibration_result->vanishing_pt(0) = vp(0);
  calibration_result->vanishing_pt(1) = vp(1);

  // prepare virtual ground points and img points
  std::vector<Point2float> ground_img_pts;
  std::vector<Point2float> ground_world_pts;
  Mat3d img2ground_hmat;
  float x_ground_gap = 10;
  float y_ground_gap = 1.5;
  float fx = calibration_inputs.camera_focus_length_fx;
  float fy = calibration_inputs.camera_focus_length_fy;
  float cam_v_offset = calibration_inputs.camera_vertical_offset;
  float cam_l_offset = calibration_inputs.camera_lateral_offset;
  for (int a = 1; a <= 2; ++a) {
    for (int b = -1; b <= 1; b += 2) {
      float gd_x = a * x_ground_gap;
      float gd_y = b * y_ground_gap;
      float img_y = camera_height * fy / (gd_x - cam_v_offset) + vp(1);
      float img_x = vp(0) - fx / (gd_x - cam_v_offset) * (gd_y - cam_l_offset);
      // roll adjustment
      float new_img_x, new_img_y;
      float x2vp_x = img_x - vp(0);
      float y2vp_y = img_y - vp(1);
      new_img_x = x2vp_x * cos_roll - y2vp_y * sin_roll + vp(0);
      new_img_y = y2vp_y * cos_roll + x2vp_x * sin_roll + vp(1);
      ground_img_pts.emplace_back(Point2float(new_img_x, new_img_y));
      ground_world_pts.emplace_back(Point2float(gd_x, gd_y));
    }
  }

  // compute image to ground homography
  homography_flag =
      CeresSolveHomography(ground_img_pts, ground_world_pts,
                           expected_symmetric_distance, &img2ground_hmat);
  if (!homography_flag) {
    LOGE("failed to compute homography");
    return false;
  }

  calibration_result->cam_homography = img2ground_hmat;

  calibration_result_ = *calibration_result;
  this->SaveCalibrationResult(calibration_result_, output_json_path);

  return true;
}

bool CalibrationBoardTool::ArucoMarkerBoardCalibration(
    const std::vector<std::vector<float>> &gray_img,
    const InputParam &calibration_inputs, const std::string &output_json_path,
    CalibrationOutputParams *calibration_result) {
  if (calibration_result == nullptr) {
    LOGE("wrong param");
    return false;
  }
  // copy input param to base calibration param struct
  base_input_param_.camera_height = calibration_inputs.camera_height;
  base_input_param_.camera_lateral_offset =
      calibration_inputs.camera_lateral_offset;
  base_input_param_.camera_vertical_offset =
      calibration_inputs.camera_vertical_offset;
  base_input_param_.camera_focus_length_fx =
      calibration_inputs.camera_focus_length_fx;
  base_input_param_.camera_focus_length_fy =
      calibration_inputs.camera_focus_length_fy;
  base_input_param_.img_width = gray_img[0].size();
  base_input_param_.img_height = gray_img.size();

  // generate vertical board pattern
  ArucoMarker aruco_marker;
  // check board pattern
  if (!aruco_marker.check()) {
    LOGE("wrong param");
    return false;
  }

  // corner detect
  arucomarker::ArucoMarkerDetector board_detector;
  arucomarker::CornerPoints corners;
  bool is_success = board_detector.detect(gray_img, aruco_marker, &corners);
  if (!is_success) {
    LOGE("failed to detect corner");
    return false;
  }
  // display
  if (display_img_) {
    cv::Mat display_img = image_.clone();
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
    cv::imshow("apriltag", display_img);
    cv::waitKey();
    cv::imwrite(output_dir_ + "/apriltag.png", display_img);
  }

  return true;
}

bool CalibrationBoardTool::SaveCalibrationResult(
    const CalibrationOutputParams &calibration_result,
    const std::string &output_json_path) {
  output_dir_ = output_json_path;
  /* check and create output directory of Calibration result */
  if (output_dir_.rfind('/') != output_dir_.size() - 1) {
    output_dir_ = output_dir_ + "/";
  }
  if (opendir(output_dir_.c_str()) == nullptr) {
    char command[1024];
    snprintf(command, sizeof(command), "mkdir -p %s", output_dir_.c_str());
    if (system(command)) {
      printf("Create dir: %s\n", output_dir_.c_str());
    }
  }
  // three calibration json files
  std::string calibration_json_path =
      output_dir_ + "front_center_camera-calibration.json";
  std::string extrinsic_json_path =
      output_dir_ + "front_center_camera-to-car_center-extrinsic.json";
  std::string intrinsic_json_path =
      output_dir_ + "front_center_camera-intrinsic.json";

  if (!this->SaveJsonCalibration(calibration_result, calibration_json_path)) {
    std::cerr << "[ADAS CALIB]Write calibration json file fail!" << std::endl;
  }
  if (!this->SaveJsonExtrinsic(calibration_result, extrinsic_json_path)) {
    std::cerr << "[ADAS CALIB]Write extrinsic json file fail!" << std::endl;
  }
  if (!this->SaveJsonIntrinsic(calibration_result, intrinsic_json_path)) {
    std::cerr << "[ADAS CALIB]Write intrinsic json file fail!" << std::endl;
  }
  return true;
}

bool CalibrationBoardTool::SaveJsonCalibration(
    const CalibrationOutputParams &result,
    const std::string &output_json_file_name) {
  Json::Value front_center_camera_calibration;
  Json::Value data;
  Json::Value param;
  Json::Value vanishing_point;
  Json::Value blinding_area_param;
  Json::Value h_matrix;
  Json::Value h_matrix_data(Json::arrayValue);
  Json::Value h_matrix_data1(Json::arrayValue);
  Json::Value h_matrix_data2(Json::arrayValue);
  Json::Value h_matrix_data3(Json::arrayValue);

  vanishing_point["vanishing_pt_x"] = result.vanishing_pt(0);
  vanishing_point["vanishing_pt_y"] = result.vanishing_pt(1);
  blinding_area_param["blinding_area_distance"] = 4.587542018708203;
  blinding_area_param["blinding_area_pixel_height"] = 561.0;
  h_matrix["rows"] = Json::Value(3);
  h_matrix["cols"] = Json::Value(3);
  h_matrix["type"] = Json::Value(6);
  h_matrix["continuous"] = Json::Value(true);
  h_matrix_data1.append(result.cam_homography(0, 0));
  h_matrix_data1.append(result.cam_homography(0, 1));
  h_matrix_data1.append(result.cam_homography(0, 2));
  h_matrix_data2.append(result.cam_homography(1, 0));
  h_matrix_data2.append(result.cam_homography(1, 1));
  h_matrix_data2.append(result.cam_homography(1, 2));
  h_matrix_data3.append(result.cam_homography(2, 0));
  h_matrix_data3.append(result.cam_homography(2, 1));
  h_matrix_data3.append(result.cam_homography(2, 2));
  h_matrix_data.append(h_matrix_data1);
  h_matrix_data.append(h_matrix_data2);
  h_matrix_data.append(h_matrix_data3);
  // h_matrix["data"] = Json::Value(output_hmat);
  h_matrix["data"] = Json::Value(h_matrix_data);

  param["vanishing_point"] = Json::Value(vanishing_point);
  param["blinding_area_param"] = Json::Value(blinding_area_param);
  param["h_matrix"] = Json::Value(h_matrix);

  // data["sensor_name"] = Json::Value("center_camera_fov30");
  // data["target_sensor_name"] = Json::Value("center_camera_fov30");
  data["sensor_name"] = Json::Value("front_center_camera");
  data["target_sensor_name"] = Json::Value("front_center_camera");
  data["device_type"] = Json::Value("camera");
  data["param_type"] = Json::Value("calibration");
  data["param"] = Json::Value(param);

  front_center_camera_calibration["front_center_camera_calibration"] =
      Json::Value(data);
  std::cout << "Writing File " << output_json_file_name.c_str() << std::endl;

  Json::StreamWriterBuilder write_builder;
  std::unique_ptr<Json::StreamWriter> json_writer(
      write_builder.newStreamWriter());
  std::ofstream output;
  output.open(output_json_file_name);
  json_writer->write(front_center_camera_calibration, &output);
  output.close();
  return true;
}

bool CalibrationBoardTool::SaveJsonExtrinsic(
    const CalibrationOutputParams &result,
    const std::string &output_json_file_name) {
  Json::Value front_center_camera_car_center_calib;
  Json::Value data;
  Json::Value param;
  Json::Value sensor_calib;
  Json::Value sensor_calib_matrix_data(Json::arrayValue);
  Json::Value sensor_calib_matrix_data1(Json::arrayValue);
  Json::Value sensor_calib_matrix_data2(Json::arrayValue);
  Json::Value sensor_calib_matrix_data3(Json::arrayValue);
  Json::Value sensor_calib_matrix_data4(Json::arrayValue);

  sensor_calib["rows"] = 4;
  sensor_calib["cols"] = 4;
  sensor_calib["type"] = 6;
  sensor_calib["continuous"] = Json::Value(true);
  sensor_calib_matrix_data1.append(1.0);
  sensor_calib_matrix_data1.append(0.0);
  sensor_calib_matrix_data1.append(0.0);
  sensor_calib_matrix_data1.append(base_input_param_.camera_vertical_offset);
  sensor_calib_matrix_data2.append(0.0);
  sensor_calib_matrix_data2.append(1.0);
  sensor_calib_matrix_data2.append(0.0);
  sensor_calib_matrix_data2.append(base_input_param_.camera_lateral_offset);
  sensor_calib_matrix_data3.append(0.0);
  sensor_calib_matrix_data3.append(0.0);
  sensor_calib_matrix_data3.append(1.0);
  sensor_calib_matrix_data3.append(base_input_param_.camera_height);
  sensor_calib_matrix_data4.append(0.0);
  sensor_calib_matrix_data4.append(0.0);
  sensor_calib_matrix_data4.append(0.0);
  sensor_calib_matrix_data4.append(1.0);
  sensor_calib_matrix_data.append(sensor_calib_matrix_data1);
  sensor_calib_matrix_data.append(sensor_calib_matrix_data2);
  sensor_calib_matrix_data.append(sensor_calib_matrix_data3);
  sensor_calib_matrix_data.append(sensor_calib_matrix_data4);
  sensor_calib["data"] = Json::Value(sensor_calib_matrix_data);

  param["time_lag"] = 0;
  param["sensor_calib"] = Json::Value(sensor_calib);

  // data["sensor_name"] = Json::Value("center_camera_fov30");
  data["sensor_name"] = Json::Value("front_center_camera");
  data["target_sensor_name"] = Json::Value("car_center");
  data["device_type"] = Json::Value("relational");
  data["param_type"] = Json::Value("extrinsic");
  data["param"] = Json::Value(param);

  front_center_camera_car_center_calib["front_center_camera_car_center_calib"] =
      Json::Value(data);
  std::cout << "Writing File " << output_json_file_name.c_str() << std::endl;

  Json::StreamWriterBuilder write_builder;
  std::unique_ptr<Json::StreamWriter> json_writer(
      write_builder.newStreamWriter());
  std::ofstream output;
  output.open(output_json_file_name);
  json_writer->write(front_center_camera_car_center_calib, &output);
  output.close();
  return true;
}

bool CalibrationBoardTool::SaveJsonIntrinsic(
    const CalibrationOutputParams &result,
    const std::string &output_json_file_name) {
  Json::Value front_center_camera_calib;
  Json::Value data;
  Json::Value param;
  Json::Value cam_K;
  Json::Value cam_K_data(Json::arrayValue);
  Json::Value cam_K_data1(Json::arrayValue);
  Json::Value cam_K_data2(Json::arrayValue);
  Json::Value cam_K_data3(Json::arrayValue);
  Json::Value cam_dist;
  Json::Value cam_dist_data(Json::arrayValue);
  Json::Value cam_dist_data1(Json::arrayValue);

  cam_K["rows"] = 3;
  cam_K["cols"] = 3;
  cam_K["type"] = 6;
  cam_K["continuous"] = Json::Value(true);
  cam_K_data1.append(base_input_param_.camera_focus_length_fx);
  cam_K_data1.append(0.0);
  // cam_K_data1.append(523.49975711);
  cam_K_data1.append(base_input_param_.img_width / 2 + std::rand() % 10 + 1);
  cam_K_data2.append(0.0);
  cam_K_data2.append(base_input_param_.camera_focus_length_fy);
  // cam_K_data2.append(321.25872596);
  cam_K_data2.append(base_input_param_.img_height / 2 + std::rand() % 10 + 1);
  cam_K_data3.append(0.0);
  cam_K_data3.append(0.0);
  cam_K_data3.append(1.0);
  cam_K_data.append(cam_K_data1);
  cam_K_data.append(cam_K_data2);
  cam_K_data.append(cam_K_data3);
  cam_K["data"] = cam_K_data;

  cam_dist["rows"] = 1;
  cam_dist["cols"] = 4;
  cam_dist["type"] = 6;
  cam_dist["continuous"] = Json::Value(true);
  cam_dist_data1.append(0.0);
  cam_dist_data1.append(0.0);
  cam_dist_data1.append(0.0);
  cam_dist_data1.append(0.0);
  // cam_dist_data1.append(0.0);
  cam_dist_data.append(cam_dist_data1);
  cam_dist["data"] = cam_dist_data;

  param["img_dist_w"] = base_input_param_.img_width;
  param["img_dist_h"] = base_input_param_.img_height;
  param["cam_K"] = Json::Value(cam_K);
  param["cam_dist"] = Json::Value(cam_dist);

  data["sensor_name"] = Json::Value("front_center_camera");
  data["target_sensor_name"] = Json::Value("front_center_camera");
  data["device_type"] = Json::Value("camera");
  data["param_type"] = Json::Value("intrinsic");
  data["param"] = Json::Value(param);

  front_center_camera_calib["front_center_camera_calib"] = Json::Value(data);
  std::cout << "Writing File " << output_json_file_name.c_str() << std::endl;

  Json::StreamWriterBuilder write_builder;
  std::unique_ptr<Json::StreamWriter> json_writer(
      write_builder.newStreamWriter());
  std::ofstream output;
  output.open(output_json_file_name);
  json_writer->write(front_center_camera_calib, &output);
  output.close();
  return true;
}

bool CalibrationBoardTool::LoadJsonIntrinsic(
    const std::string &input_json_file_name,
    CameraIntrinsic *camera_intrinsic) {
  if (camera_intrinsic == nullptr) {
    return false;
  }
  Json::CharReaderBuilder builder;
  Json::Value read_data;
  JSONCPP_STRING errs;
  std::ifstream in(input_json_file_name, std::ios::binary);

  if (!in.is_open()) {
    std::cout << "Error open read json:" << input_json_file_name << std::endl;
    return false;
  }

  if (Json::parseFromStream(builder, in, &read_data, &errs)) {
    camera_intrinsic->camera_focus_length_fx =
        read_data["front_center_camera_calib"]["param"]["cam_K"]["data"][0][0]
            .asDouble();
    camera_intrinsic->camera_focus_length_fy =
        read_data["front_center_camera_calib"]["param"]["cam_K"]["data"][1][1]
            .asDouble();
    camera_intrinsic->img_width =
        read_data["front_center_camera_calib"]["param"]["img_dist_w"].asInt();
    camera_intrinsic->img_height =
        read_data["front_center_camera_calib"]["param"]["img_dist_h"].asInt();
  } else {
    return false;
  }

  return true;
}

} // namespace cameracalib
