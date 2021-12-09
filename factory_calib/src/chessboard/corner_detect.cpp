/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */

#include "chessboard/corner_detect.hpp"

double CornerDetect::CalculatePointDis(double x1, double y1, double x2,
                                       double y2) {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

bool CornerDetect::IsRectangle(const std::vector<Point> &approx_contour) {
  Point p1, p2, p3, p4;
  p1 = approx_contour[0];
  p2 = approx_contour[1];
  p3 = approx_contour[2];
  p4 = approx_contour[3];
  double x_c = (p1.x + p2.x + p3.x + p4.x) / 4;
  double y_c = (p1.y + p2.y + p3.y + p4.y) / 4;
  double d1, d2, d3, d4;
  d1 = CalculatePointDis(p1.x, p1.y, x_c, y_c);
  d2 = CalculatePointDis(p2.x, p2.y, x_c, y_c);
  d3 = CalculatePointDis(p3.x, p3.y, x_c, y_c);
  d4 = CalculatePointDis(p4.x, p4.y, x_c, y_c);
  double diff1, diff2, diff3;
  diff1 = std::fabs(d1 - d2);
  diff2 = std::fabs(d1 - d3);
  diff3 = std::fabs(d1 - d4);
  return diff1 < 5 && diff2 < 5 && diff3 < 5;
}

bool CornerDetect::IsSquare(const std::vector<Point> &approx_contour) {
  Point p1, p2, p3;
  p1 = approx_contour[0];
  p2 = approx_contour[1];
  p3 = approx_contour[2];
  double d1, d2, d3;
  d1 = CalculatePointDis(p1.x, p1.y, p2.x, p2.y);
  d2 = CalculatePointDis(p1.x, p1.y, p3.x, p3.y);
  d3 = CalculatePointDis(p2.x, p2.y, p3.x, p3.y);
  double diff1, diff2, diff3;
  diff1 = std::fabs(d1 - d2);
  diff2 = std::fabs(d1 - d3);
  diff3 = std::fabs(d2 - d3);
  return diff1 < 10 || diff2 < 10 || diff3 < 10;
}

bool CornerDetect::IsRightTriangle(const std::vector<Quad> &quads) {
  if (quads.size() != 3) {
    return false;
  }
  Point c1 = quads[0].center;
  Point c2 = quads[1].center;
  Point c3 = quads[2].center;
  double dis1 = (c1.x - c2.x) * (c1.x - c2.x) + (c1.y - c2.y) * (c1.y - c2.y);
  double dis2 = (c1.x - c3.x) * (c1.x - c3.x) + (c1.y - c3.y) * (c1.y - c3.y);
  double dis3 = (c2.x - c3.x) * (c2.x - c3.x) + (c2.y - c3.y) * (c2.y - c3.y);
  return std::fabs(dis1 + dis2 - dis3) < 1000 ||
         std::fabs(dis1 + dis3 - dis2) < 1000 ||
         std::fabs(dis2 + dis3 - dis1) < 1000;
}

bool CornerDetect::IsIsoscelesTriangle(const std::vector<Quad> &quads) {
  if (quads.size() != 3) {
    return false;
  }
  Point center1 = quads[0].center;
  Point center2 = quads[1].center;
  Point center3 = quads[2].center;
  double dis1 = CalculatePointDis(center1.x, center1.y, center2.x, center2.y);
  double dis2 = CalculatePointDis(center1.x, center1.y, center3.x, center3.y);
  double dis3 = CalculatePointDis(center2.x, center2.y, center3.x, center3.y);
  return std::fabs(dis1 - dis2) < 10 || std::fabs(dis1 - dis3) < 10 ||
         std::fabs(dis2 - dis3) < 10;
}

bool CornerDetect::IsQuadEqual(Quad q1, Quad q2) {
  return q1.center.x == q2.center.x && q1.center.y == q2.center.y;
}

void CornerDetect::ClusterByRansac(const std::vector<Quad> &cluster_tmp,
                                   const double &dis_lower_thresh,
                                   const double &dis_upper_thresh,
                                   std::vector<std::vector<Quad>> *sets) {
  if (sets == nullptr)
    return;
  std::vector<Quad> points_set = cluster_tmp;
  sets->clear();
  while (true) {
    int loop_times = points_set.size();
    int maxVoteCnt = 0;
    int initialValue = 0;
    std::vector<Quad> get_points;
    for (int i = 0; i < loop_times; i++) {
      int selectedIndex = initialValue % loop_times;
      initialValue = initialValue + 1;
      Quad select_point = points_set[selectedIndex];
      int voteCnt = 0;
      std::vector<Quad> subPoint;
      subPoint.push_back(select_point);
      for (int k = 0; k < loop_times; k++) {
        if (k == selectedIndex)
          continue;
        double x1 = select_point.center.x;
        double y1 = select_point.center.y;
        double x2 = points_set[k].center.x;
        double y2 = points_set[k].center.y;
        double dis = CalculatePointDis(x1, y1, x2, y2);
        if (dis > dis_lower_thresh && dis < dis_upper_thresh) {
          subPoint.push_back(points_set[k]);
          voteCnt++;
        }
      }
      if (voteCnt > maxVoteCnt) {
        maxVoteCnt = voteCnt;
        get_points.clear();
        for (size_t j = 0; j < subPoint.size(); j++) {
          get_points.push_back(subPoint[j]);
        }
      }
    }
    if (get_points.size() < 1)
      break;
    sets->push_back(get_points);
    for (size_t i = 0; i < get_points.size(); i++) {
      Quad item = get_points[i];
      std::vector<Quad>::iterator ite;
      for (ite = points_set.begin(); ite != points_set.end(); ++ite) {
        if (IsQuadEqual(item, *ite)) {
          ite = points_set.erase(ite);
          break;
        }
      }
    }
  }
}

void CornerDetect::GetSlopPoint(const std::vector<Quad> &chess, Point *point) {
  double x = (chess[0].center.x + chess[1].center.x + chess[2].center.x) / 3;
  double y = (chess[0].center.y + chess[1].center.y + chess[2].center.y) / 3;
  point->x = std::round(x);
  point->y = std::round(y);
}

double CornerDetect::Max(int x1, int x2, int x3, int x4) {
  int max = 0;
  if (max < x1)
    max = x1;
  if (max < x2)
    max = x2;
  if (max < x3)
    max = x3;
  if (max < x4)
    max = x4;
  return max;
}

double CornerDetect::Min(int x1, int x2, int x3, int x4) {
  int min = 1000000;
  if (min > x1)
    min = x1;
  if (min > x2)
    min = x2;
  if (min > x3)
    min = x3;
  if (min > x4)
    min = x4;
  return min;
}

void CornerDetect::GetQuadDis(const Quad quad, int *qx, int *qy) {
  int p1_x = quad.p1.x;
  int p1_y = quad.p1.y;
  int p2_x = quad.p2.x;
  int p2_y = quad.p2.y;
  int p3_x = quad.p3.x;
  int p3_y = quad.p3.y;
  int p4_x = quad.p4.x;
  int p4_y = quad.p4.y;
  int min_x = Min(p1_x, p2_x, p3_x, p4_x);
  int min_y = Min(p1_y, p2_y, p3_y, p4_y);
  int max_x = Max(p1_x, p2_x, p3_x, p4_x);
  int max_y = Max(p1_y, p2_y, p3_y, p4_y);
  *qx = max_x - min_x;
  *qy = max_y - min_y;
}

void CornerDetect::GetGridDis(const std::vector<std::vector<Quad>> chess_sets,
                              int *gx_dis, int *gy_dis) {
  if (chess_sets.size() != 2)
    return;
  if (gx_dis == nullptr || gy_dis == nullptr)
    return;
  std::vector<int> x_dis, y_dis;
  for (size_t i = 0; i < chess_sets[0].size(); i++) {
    Quad quad = chess_sets[0][i];
    int quad_x_dis = 0;
    int quad_y_dis = 0;
    GetQuadDis(quad, &quad_x_dis, &quad_y_dis);
    x_dis.push_back(quad_x_dis);
    y_dis.push_back(quad_y_dis);
  }

  for (size_t i = 0; i < chess_sets[1].size(); i++) {
    Quad quad = chess_sets[1][i];
    int quad_x_dis = 0;
    int quad_y_dis = 0;
    GetQuadDis(quad, &quad_x_dis, &quad_y_dis);
    x_dis.push_back(quad_x_dis);
    y_dis.push_back(quad_y_dis);
  }

  int sum_x = 0;
  int sum_y = 0;
  for (size_t i = 0; i < x_dis.size(); i++)
    sum_x += x_dis[i];
  for (size_t i = 0; i < y_dis.size(); i++)
    sum_y += y_dis[i];
  if (x_dis.size() == 0 || y_dis.size() == 0)
    return;
  *gx_dis = std::round(sum_x / x_dis.size()) + 2;
  *gy_dis = std::round(sum_y / y_dis.size()) + 2;
}

double CornerDetect::CalculateSlope(const std::vector<Quad> &first_chess,
                                    const std::vector<Quad> &second_chess) {
  double x1 = (first_chess[0].center.x + first_chess[1].center.x +
               first_chess[2].center.x) /
              3;
  double y1 = (first_chess[0].center.y + first_chess[1].center.y +
               first_chess[2].center.y) /
              3;
  double x2 = (second_chess[0].center.x + second_chess[1].center.x +
               second_chess[2].center.x) /
              3;
  double y2 = (second_chess[0].center.y + second_chess[1].center.y +
               second_chess[2].center.y) /
              3;
  double slop_value = std::fabs(y2 - y1) / std::fabs(x2 - x1);
  double theta = std::atan(slop_value);
  return theta;
}

double CornerDetect::CalculateArea(const std::vector<Quad> &chess) {
  Point p1 = chess[0].center;
  Point p2 = chess[1].center;
  Point p3 = chess[2].center;
  double a = CalculatePointDis(p1.x, p1.y, p2.x, p2.y);
  double b = CalculatePointDis(p1.x, p1.y, p3.x, p3.y);
  double c = CalculatePointDis(p2.x, p2.y, p3.x, p3.y);
  double s = (a + b + c) / 2;
  double area = std::sqrt(s * (s - a) * (s - b) * (s - c));
  return area;
}

bool CornerDetect::CheckChessBoardArea(const std::vector<Quad> &first_chess,
                                       const std::vector<Quad> &second_chess) {
  double first_area = CalculateArea(first_chess);
  double second_area = CalculateArea(second_chess);
  if (std::fabs(second_area) < 1e-6)
    return false;
  return 0.5 < (first_area / second_area) && (first_area / second_area) < 1.5;
}

bool CornerDetect::CheckChessboard(
    const std::vector<std::vector<Quad>> &valid_sets,
    std::vector<Quad> *valid_Chessboard1,
    std::vector<Quad> *valid_Chessboard2) {
  if (valid_Chessboard1 == nullptr || valid_Chessboard2 == nullptr) {
    return false;
  }
  double min_slop = 1;

  for (size_t i = 0; i < valid_sets.size() - 1; i++) {
    std::vector<Quad> first_chess = valid_sets[i];
    for (size_t j = i + 1; j < valid_sets.size(); j++) {
      std::vector<Quad> second_chess = valid_sets[j];
      double slop = CalculateSlope(first_chess, second_chess);
      if (min_slop > slop) {
        min_slop = slop;
        *valid_Chessboard1 = first_chess;
        *valid_Chessboard2 = second_chess;
      }
    }
  }
  if (min_slop < 0.15 &&
      CheckChessBoardArea(*valid_Chessboard1, *valid_Chessboard2)) {
    return true;
  } else {
    return false;
  }
}

void CornerDetect::ImageThreshold(
    const std::vector<std::vector<float>> &imgGray,
    const double &thresh_threshold, std::vector<std::vector<int>> *thresh_vec,
    float edge_fill_percent) {
  if (thresh_vec == nullptr)
    return;
  if (imgGray.size() == 0)
    return;
  if (edge_fill_percent > 0.29)
    return;
  int height = imgGray.size();
  int width = imgGray[0].size();
  int edge_fill_thickness_x =
      static_cast<int>(edge_fill_percent * height * 1.7);
  int edge_fill_thickness_y = static_cast<int>(edge_fill_percent * width);
  std::vector<std::vector<int>> img_vec(height, std::vector<int>(width, 255));
  for (int i = edge_fill_thickness_x; i < height - edge_fill_thickness_x; i++) {
    for (int j = edge_fill_thickness_y; j < width - edge_fill_thickness_y;
         j++) {
      if (imgGray[i][j] < thresh_threshold) {
        img_vec[i][j] = 0;
      } else {
        img_vec[i][j] = 255;
      }
    }
  }
  *thresh_vec = img_vec;
}

void CornerDetect::EstimateGrayThres(
    const std::vector<std::vector<float>> &imgGray, std::vector<int> *thres_box,
    float edge_fill_percent) {
  if (thres_box == nullptr)
    return;
  if (imgGray.size() == 0)
    return;
  if (edge_fill_percent > 0.29)
    return;
  int height = imgGray.size();
  int width = imgGray[0].size();
  int edge_fill_thickness_x =
      static_cast<int>(edge_fill_percent * height * 1.7);
  int edge_fill_thickness_y = static_cast<int>(edge_fill_percent * width);
  float aver_gray = 0;
  float mid_gray = 0;

  std::vector<int> gray_pixel_num(256);
  std::vector<float> gray_prob(256);
  std::vector<float> gray_distribution(256);
  int pixel_num = 0;

  for (int i = edge_fill_thickness_x; i < height - edge_fill_thickness_x; i++) {
    for (int j = edge_fill_thickness_y; j < width - edge_fill_thickness_y;
         j++) {
      aver_gray = (aver_gray * pixel_num + imgGray[i][j]) /
                  static_cast<float>(pixel_num + 1);
      gray_pixel_num[static_cast<int>(imgGray[i][j])]++;
      pixel_num++;
    }
  }
  for (int i = 0; i < 256; i++) {
    gray_prob[i] = gray_pixel_num[i] / static_cast<float>(pixel_num);
  }
  gray_distribution[0] = gray_prob[0];
  for (int i = 1; i < 256; i++) {
    gray_distribution[i] = gray_distribution[i - 1] + gray_prob[i];
    if (gray_distribution[i - 1] <= 0.5 && gray_distribution[i] >= 0.5)
      mid_gray = i;
  }

  std::vector<int> possible_box;
  int start_thres = static_cast<int>(aver_gray);
  if (aver_gray > 127.5)
    start_thres = static_cast<int>(mid_gray + mid_gray - aver_gray);
  else
    start_thres = static_cast<int>(aver_gray + aver_gray - mid_gray);
  int stride = static_cast<int>((127.5 - abs(127.5 - aver_gray)) *
                                (1.0 -
                                 (127.5 - aver_gray) * (127.5 - mid_gray) /
                                     static_cast<float>(127.5 * 127.5)) /
                                6.0);
  if (aver_gray > 127.5 && (mid_gray - aver_gray) < 0)
    stride = stride * (-1);
  if (aver_gray < 127.5 && (mid_gray - aver_gray) > 0)
    stride = stride * (-1);

  int iter = static_cast<int>(256.0 / static_cast<float>(abs(stride)));
  possible_box.push_back((start_thres - stride + 256) % 256);
  possible_box.push_back(start_thres);
  for (int i = 2; i <= iter; i++) {
    if (size_t(i - 1) >= possible_box.size())
      break;
    if (possible_box[i - 1] + stride >= 255 ||
        possible_box[i - 1] + stride < 0) {
      possible_box.push_back(start_thres - 2 * stride);
      stride *= -1;
    } else {
      possible_box.push_back(possible_box[i - 1] + stride);
    }
  }

  std::cout << "possible box:\n";
  for (size_t i = 0; i < possible_box.size(); i++) {
    std::cout << possible_box[i] << " ";
  }
  std::cout << std::endl;

  *thres_box = possible_box;
}

void CornerDetect::ImageDilate(const std::vector<std::vector<int>> thresh_vec,
                               int kernal[3][3],
                               std::vector<std::vector<int>> *dilation_vec,
                               float edge_fill_percent) {
  if (dilation_vec == nullptr)
    return;
  if (edge_fill_percent > 0.29)
    return;
  *dilation_vec = thresh_vec;
  int nTemp;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3 - i; j++) {
      nTemp = kernal[i][j];
      kernal[i][j] = kernal[2 - i][2 - j];
      kernal[2 - i][2 - j] = nTemp;
    }
  }

  int edge_fill_thickness_x =
      static_cast<int>(edge_fill_percent * thresh_vec.size() * 1.7);
  int edge_fill_thickness_y =
      static_cast<int>(edge_fill_percent * thresh_vec[0].size());
  for (size_t i = edge_fill_thickness_x + 1;
       i < thresh_vec.size() - 1 - edge_fill_thickness_x; i++) {
    for (size_t j = edge_fill_thickness_y + 1;
         j < thresh_vec[i].size() - 1 - edge_fill_thickness_y; j++) {
      if (thresh_vec[i][j] != 0) {
        for (int k = 0; k < 3; k++) {
          for (int l = 0; l < 3; l++) {
            if (kernal[k][l] == -1)
              continue;
            if (kernal[k][l] == 1) {
              (*dilation_vec)[i - 1 + k][j - 1 + l] = 255;
            } else {
              return;
            }
          }
        }
      }
    }
  }
}

double CornerDetect::CalculateContourArea(std::vector<Point> contour) {
  if (contour.size() == 0)
    return 0;
  double a00 = 0;
  int contour_size = contour.size();
  Point2f prev = Point2f(static_cast<float>(contour[contour_size - 1].x),
                         static_cast<float>(contour[contour_size - 1].y));
  for (size_t i = 0; i < contour.size(); i++) {
    Point2f p = Point2f(static_cast<float>(contour[i].x),
                        static_cast<float>(contour[i].y));
    a00 +=
        static_cast<double>(prev.x * p.y) - static_cast<double>(prev.y * p.x);
    prev = p;
  }
  a00 *= 0.5;
  a00 = std::fabs(a00);
  return a00;
}

std::vector<Point>
CornerDetect::DepthFirstSearch(std::vector<std::vector<int>> *img, int i,
                               int j) {
  std::vector<Point> temp;
  if (img == nullptr)
    return temp;
  std::stack<Point> S;
  S.push(Point(i, j));
  (*img)[i][j] = 0;

  temp.push_back(Point(j, i));
  while (!S.empty()) {
    Point node = S.top();
    i = node.x;
    j = node.y;
    S.pop();
    if ((*img)[i + 1][j] == 0) {
      (*img)[i + 1][j] = 255;
      S.push(Point(i + 1, j));
      temp.push_back(Point(j, i + 1));
    }
    if ((*img)[i][j + 1] == 0) {
      (*img)[i][j + 1] = 255;
      S.push(Point(i, j + 1));
      temp.push_back(Point(j + 1, i));
    }
    if ((*img)[i - 1][j] == 0) {
      (*img)[i - 1][j] = 255;
      S.push(Point(i - 1, j));
      temp.push_back(Point(j, i - 1));
    }
    if ((*img)[i][j - 1] == 0) {
      (*img)[i][j - 1] = 255;
      S.push(Point(i, j - 1));
      temp.push_back(Point(j - 1, i));
    }
  }
  return temp;
}

void CornerDetect::FindContours(
    const std::vector<std::vector<int>> dilation_vec,
    std::vector<std::vector<Point>> *rt_contours, float edge_fill_percent) {
  if (rt_contours == nullptr) {
    return;
  }
  if (edge_fill_percent > 0.29)
    return;
  std::vector<std::vector<int>> img = dilation_vec;
  int edge_fill_thickness_x =
      static_cast<int>(edge_fill_percent * dilation_vec.size() * 1.7);
  int edge_fill_thickness_y =
      static_cast<int>(edge_fill_percent * dilation_vec[0].size());

  for (size_t i = edge_fill_thickness_x;
       i < dilation_vec.size() - edge_fill_thickness_x; i++)
    for (size_t j = edge_fill_thickness_y;
         j < dilation_vec[i].size() - edge_fill_thickness_y; j++) {
      int tmp = dilation_vec[i][j];
      if (tmp > 120)
        img[i][j] = 255;
      else
        img[i][j] = 0;
    }

  std::vector<std::vector<int>> img_tmp = img;
  for (size_t i = 1; i < img.size() - 1; i++) {
    for (size_t j = 1; j < img[i].size() - 1; j++) {
      if (img[i + 1][j] == 0 && img[i + 1][j - 1] == 0 && img[i][j - 1] == 0 &&
          img[i - 1][j - 1] == 0 && img[i - 1][j] == 0 &&
          img[i - 1][j + 1] == 0 && img[i][j + 1] == 0 &&
          img[i + 1][j + 1] == 0) {
        img_tmp[i][j] = 255;
      }
    }
  }
  std::vector<Point> temp;
  for (size_t i = 1; i < img_tmp.size() - 1; i++) {
    for (size_t j = 1; j < img_tmp[i].size() - 1; j++) {
      if (img_tmp[i][j] == 0) {
        temp = DepthFirstSearch(&img_tmp, i, j);
        if (temp.size() < 1000 && temp.size() > 20)
          rt_contours->push_back(temp);
      }
    }
  }
}

void CornerDetect::ApproxPolyDP(const std::vector<Point> src_contour,
                                double eps, std::vector<Point> *rt_contour) {
  if (rt_contour == nullptr)
    return;

#define READ_DST_PT(pt, pos)                                                   \
  pt = dst_contour[pos];                                                       \
  if (++pos >= count)                                                          \
  pos = 0
#define READ_PT(pt, pos)                                                       \
  pt = src_contour[pos];                                                       \
  if (++pos >= count)                                                          \
  pos = 0
  std::vector<Point> dst_contour;
  Point slice(0, 0);
  Point right_slice(0, 0);
  int count = src_contour.size();
  int init_iters = 3;
  int i = 0, j, pos = 0, wpos, new_count = 0;
  bool le_eps = false;
  Point start_pt(-1000000, -1000000);
  Point end_pt(0, 0);
  Point pt(0, 0);
  std::stack<Point> s;
  eps *= eps;
  right_slice.x = 0;
  for (i = 0; i < init_iters; i++) {
    double max_dist = 0;
    pos = (pos + right_slice.x) % count;

    READ_PT(start_pt, pos);
    for (j = 1; j < count; j++) {
      double dx, dy;
      READ_PT(pt, pos);
      dx = pt.x - start_pt.x;
      dy = pt.y - start_pt.y;
      double dist = dx * dx + dy * dy;
      if (dist > max_dist) {
        max_dist = dist;
        right_slice.x = j;
      }
    }
    le_eps = max_dist <= eps;
  }
  if (!le_eps) {
    right_slice.y = slice.x = pos % count;
    slice.y = right_slice.x = (right_slice.x + slice.x) % count;
    s.push(right_slice);
    s.push(slice);

  } else {
    dst_contour.push_back(start_pt);
    new_count++;
  }

  while (!s.empty()) {
    slice = s.top();
    s.pop();
    end_pt = src_contour[slice.y];
    pos = slice.x;
    READ_PT(start_pt, pos);
    if (pos != slice.y) {
      double dx, dy, max_dist = 0;

      dx = end_pt.x - start_pt.x;
      dy = end_pt.y - start_pt.y;
      while (pos != slice.y) {
        READ_PT(pt, pos);
        double dist =
            std::fabs((pt.y - start_pt.y) * dx - (pt.x - start_pt.x) * dy);
        if (dist > max_dist) {
          max_dist = dist;
          right_slice.x = (pos + count - 1) % count;
        }
      }
      le_eps = max_dist * max_dist <= eps * (dx * dx + dy * dy);
    } else {
      le_eps = true;
      start_pt = src_contour[slice.x];
    }
    if (le_eps) {
      dst_contour.push_back(start_pt);
      new_count++;
    } else {
      right_slice.y = slice.y;
      slice.y = right_slice.x;
      s.push(right_slice);
      s.push(slice);
    }
  }
  count = new_count;
  pos = count - 1;
  READ_DST_PT(start_pt, pos);
  wpos = pos;
  READ_DST_PT(pt, pos);
  bool is_closed = true;
  for (i = !is_closed; i < count - !is_closed && new_count > 2; i++) {
    double dx, dy, dist, successive_inner_product;
    READ_DST_PT(end_pt, pos);
    dx = end_pt.x - start_pt.x;
    dy = end_pt.y - start_pt.y;
    dist = fabs((pt.x - start_pt.x) * dy - (pt.y - start_pt.y) * dx);
    successive_inner_product = (pt.x - start_pt.x) * (end_pt.x - pt.x) +
                               (pt.y - start_pt.y) * (end_pt.y - pt.y);
    if (dist * dist <= 0.5 * eps * (dx * dx + dy * dy) && dx != 0 && dy != 0 &&
        successive_inner_product >= 0) {
      new_count--;
      dst_contour[wpos] = start_pt = end_pt;
      if (++wpos >= count)
        wpos = 0;
      READ_DST_PT(pt, pos);
      i++;
      continue;
    }
    dst_contour[wpos] = start_pt = pt;
    if (++wpos >= count)
      wpos = 0;
    pt = end_pt;
  }

  std::vector<Point> tmp_contour;
  for (int i = 0; i < new_count; i++)
    tmp_contour.push_back(dst_contour[i]);
  *rt_contour = tmp_contour;
}

bool CornerDetect::CornerDetection(
    const std::vector<std::vector<float>> &imgGray, int *vp_x, int *vp_y,
    Point *sp1, Point *sp2, int *grid_x_dis, int *grid_y_dis,
    std::vector<Point> *grid_center_points) {
  if (vp_x == nullptr || vp_y == nullptr)
    return false;
  if (sp1 == nullptr || sp2 == nullptr)
    return false;
  if (grid_x_dis == nullptr || grid_y_dis == nullptr)
    return false;
  if (grid_center_points == nullptr)
    return false;

  clock_t startTime, endTime;
  startTime = clock();
  std::vector<std::vector<int>> thresh_vec;
  std::vector<std::vector<int>> dilation_vec;
  int kernal[3][3] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
  std::vector<std::vector<Quad>> final_chess_sets;
  // int edge_fill_thickness = 300;
  float edge_fill_percent = 0.05;
  float grid_area_thres =
      static_cast<float>(imgGray.size() * imgGray[0].size()) / 4000;
  std::vector<int> thres_box;
  EstimateGrayThres(imgGray, &thres_box, edge_fill_percent);

  for (size_t thres_idx = 0; thres_idx < thres_box.size(); thres_idx += 1) {
    int thresh_threshold = thres_box[thres_idx];
    ImageThreshold(imgGray, thresh_threshold, &thresh_vec, edge_fill_percent);
    ImageDilate(thresh_vec, kernal, &dilation_vec, edge_fill_percent);
    std::vector<std::vector<Point>> contours;
    FindContours(dilation_vec, &contours, edge_fill_percent);
    std::vector<Quad> approx_contour_points;
    float area_sum = 0;
    for (size_t i = 0; i < contours.size(); i++) {
      std::vector<Point> contour = contours[i];
      std::vector<Point> approx_contour;
      for (int approx_level = 1; approx_level < 8; approx_level++) {
        ApproxPolyDP(contour, static_cast<double>(approx_level),
                     &approx_contour);
        if (approx_contour.size() == 4)
          break;
        std::vector<Point> approx_contour_tmp;
        std::swap(approx_contour, approx_contour_tmp);
        ApproxPolyDP(approx_contour_tmp, static_cast<double>(approx_level),
                     &approx_contour);
        if (approx_contour.size() == 4)
          break;
      }
      if (approx_contour.size() != 4)
        continue;
      double area = CalculateContourArea(approx_contour);
      // if (area < 400) continue;
      if (area < grid_area_thres)
        continue;
      if (!IsRectangle(approx_contour))
        continue;
      if (!IsSquare(approx_contour))
        continue;

      Point p1, p2, p3, p4;
      p1 = approx_contour[0];
      p2 = approx_contour[1];
      p3 = approx_contour[2];
      p4 = approx_contour[3];
      double x_c = (p1.x + p2.x + p3.x + p4.x) / 4;
      double y_c = (p1.y + p2.y + p3.y + p4.y) / 4;
      Point center(x_c, y_c);
      Quad quad;
      quad.center = center;
      quad.p1 = p1;
      quad.p2 = p2;
      quad.p3 = p3;
      quad.p4 = p4;
      area_sum += area;
      approx_contour_points.push_back(quad);
    }

    if (approx_contour_points.size() == 0)
      continue;
    double approx_dis_thre =
        sqrt(area_sum / static_cast<double>(approx_contour_points.size()));
    for (int base_value = 0; base_value < 50; base_value += 2) {
      std::vector<std::vector<Quad>> sets;
      double cluster_dis_threshold =
          static_cast<double>((approx_dis_thre)*1.4 + base_value); // 20
      ClusterByRansac(approx_contour_points, approx_dis_thre * 0.8,
                      cluster_dis_threshold, &sets);
      std::vector<std::vector<Quad>> valid_sets;
      for (size_t i = 0; i < sets.size(); i++) {
        if (sets[i].size() != 3)
          continue;
        if (!IsIsoscelesTriangle(sets[i]))
          continue;
        if (!IsRightTriangle(sets[i]))
          continue;
        valid_sets.push_back(sets[i]);
      }
      if (valid_sets.size() < 2)
        continue;
      std::vector<Quad> valid_Chessboard1;
      std::vector<Quad> valid_Chessboard2;
      bool check_success =
          CheckChessboard(valid_sets, &valid_Chessboard1, &valid_Chessboard2);
      if (check_success) {
        final_chess_sets.clear();
        final_chess_sets.push_back(valid_Chessboard1);
        final_chess_sets.push_back(valid_Chessboard2);
        break;
      }
    }
    if (final_chess_sets.size() == 2) {
      std::cout << "Success thresh_threshold: " << thresh_threshold
                << std::endl;
      break;
    }
  }

  endTime = clock();
  std::cout << "Totle Time: "
            << static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC
            << std::endl;
  bool is_detection_success = true;
  if (final_chess_sets.size() == 2) {
    is_detection_success = true;
    std::cout << "detection success!" << std::endl;
  } else {
    is_detection_success = false;
    std::cout << "detection fail!" << std::endl;
  }

  double corner_point_x = 0;
  double corner_point_y = 0;
  int corner_count = 0;
  for (size_t i = 0; i < final_chess_sets.size(); i++) {
    for (size_t j = 0; j < final_chess_sets[i].size(); j++) {
      Quad quad = final_chess_sets[i][j];
      Point center = quad.center;
      Point p1 = quad.p1;
      Point p2 = quad.p2;
      Point p3 = quad.p3;
      Point p4 = quad.p4;

      grid_center_points->push_back(center);
      corner_point_x += p1.x;
      corner_point_y += p1.y;
      corner_count++;
      corner_point_x += p2.x;
      corner_point_y += p2.y;
      corner_count++;
      corner_point_x += p3.x;
      corner_point_y += p3.y;
      corner_count++;
      corner_point_x += p4.x;
      corner_point_y += p4.y;
      corner_count++;
    }
  }
  if (is_detection_success) {
    int vanishing_point_x = std::round(corner_point_x / corner_count);
    int vanishing_point_y = std::round(corner_point_y / corner_count);
    *vp_x = vanishing_point_x;
    *vp_y = vanishing_point_y;
    // slope
    Point slope_point1(0, 0), slope_point2(0, 0);
    GetSlopPoint(final_chess_sets[0], &slope_point1);
    GetSlopPoint(final_chess_sets[1], &slope_point2);
    *sp1 = slope_point1;
    *sp2 = slope_point2;
    // grid
    int gx_dis = 0;
    int gy_dis = 0;
    GetGridDis(final_chess_sets, &gx_dis, &gy_dis);
    *grid_x_dis = gx_dis;
    *grid_y_dis = gy_dis;
  }
  return is_detection_success;
}
