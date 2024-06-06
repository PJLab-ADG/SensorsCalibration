/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#include "CalibrationHarp.hpp"

#define HAHAHA 5
#define UNDIST 1

struct Undistort
{
    static const int N = 20, NUM_INTRINSIC = 3;
    std::vector<std::vector<cv::Point2d>> points, points_undistort;
    double cx, cy;// center of distortion
    double f;// focal length

    int nk, iter;
    double klist[N] = { 1, };// list of coefficients for distortion correction polynomial and cx, cy, f
    double hessian[N][N], grad[N];
    double lambda = 0;
    double fxy[N][N], fx[N], fx_[N], fy[N], dk[N];

    Undistort(std::vector<std::vector<cv::Point2d>> points_, double cx_, double cy_, double f, int nk_ = 6, int iter_ = 500)
        :points(points_), points_undistort(points_), cx(cx_), cy(cy_), nk(nk_), iter(iter_)
    {
        klist[nk++] = cx, klist[nk++] = cy, klist[nk++] = f;
        calcGrad();
        calcHessian();
        for (int i = 0; i < nk; ++i)
        {
            lambda += hessian[i][i];
        }
        lambda *= 0.001;
    }

    // Compute the least squares fitting error for minimum current undistorted result for each line.
    double calcErr()
    {
        cx = klist[nk - NUM_INTRINSIC];
        cy = klist[nk - NUM_INTRINSIC + 1];
        f = klist[nk - NUM_INTRINSIC + 2];

        double res = 0, r, rk, fr;
        double a, b, c, sin, cos, sx, sy, alpha, beta;
        double xd, yd;
        int nl = points.size(), np;
        for (int i = 0; i < nl; ++i)
        {
            auto& line = points[i];
            auto& line_undistort = points_undistort[i];
            np = line.size();
            if (np <= 2) continue;

            for (int i = 0; i < np; ++i)
            {
                xd = line[i].x, yd = line[i].y;
                r = (xd - cx) * (xd - cx) + (yd - cy) * (yd - cy), r /= (f * f);
                rk = r, fr = 1;
                for (int j = 1; j < nk - NUM_INTRINSIC; ++j) fr += klist[j] * rk, rk *= r;
                line_undistort[i].x = fr * (xd - cx) + cx;
                line_undistort[i].y = fr * (yd - cy) + cy;
            }

            a = b = c = sx = sy = 0;
            for (auto p : line_undistort)
            {
                a += p.x * p.x, b += p.x * p.y, c += p.y * p.y;
                sx += p.x, sy += p.y;
            }
            a -= sx * sx / np, b -= sx * sy / np, c -= sy * sy / np;
            alpha = a - c, beta = alpha / (2 * sqrt(alpha * alpha + 4 * b * b));
            sin = sqrt(0.5 - beta);
            cos = sqrt(0.5 + beta);
            res += a * sin * sin - 2 * std::abs(b) * sin * cos + c * cos * cos;
        }
        return res;
    }

    void calcGrad()
    {
        double f0 = calcErr();
        for (int i = 0; i < nk; ++i)
        {
            dk[i] = std::max(klist[i] * 1e-4, 1e-6);
            klist[i] += dk[i], fx[i] = calcErr();
            klist[i] -= 2 * dk[i], fx_[i] = calcErr();
            klist[i] += dk[i];
            grad[i] = (fx[i] - f0) / dk[i];
        }
        for (int i = 0; i < nk; ++i)
        {
            for (int j = 0; j < nk; ++j)
            {
                klist[i] += dk[i], klist[j] += dk[j];
                fxy[i][j] = calcErr();
                klist[i] -= dk[i], klist[j] -= dk[j];
            }
        }
    }

    void calcHessian()
    {
        double f0 = calcErr();
        for (int i = 0; i < nk; ++i)
        {
            for (int j = 0; j < nk; ++j)
            {
                if (i > j) hessian[i][j] = hessian[j][i];
                else if (i == j)
                {
                    hessian[i][i] = (fx[i] + fx_[i] - 2 * f0) / dk[i];
                }
                else
                {
                    hessian[i][j] = (fxy[i][j] + f0 - fx[i] - fx[j]) / (dk[i] * dk[j]);
                }
            }
        }
    }

    void proc()
    {
        while (iter--)
        {
            double errPre = calcErr();
            calcGrad();
            calcHessian();
            for (int i = 0; i < nk; ++i) hessian[i][i] += lambda;

            cv::Mat A(nk, nk, CV_64FC1);
            for (int i = 0; i < nk; ++i)
            {
                for (int j = 0; j < nk; ++j)
                {
                    A.at<double>(i, j) = hessian[i][j];
                }
            }
            cv::invert(A, A);
            cv::Mat b(nk, 1, CV_64FC1, grad);
            cv::Mat x = -A * b;
            for (int i = 0; i < nk; ++i)
            {
                klist[i] += x.at<double>(i);
            }
            if (calcErr() > errPre)
            {
                for (int i = 0; i < nk; ++i)
                {
                    klist[i] -= x.at<double>(i);
                }
                lambda *= 10;
            }
            else lambda = std::max(lambda / 10, 1e-16);

#if 0
            printf("cx:%f, cy:%f, error:%f, lambda:%f\n", cx, cy, errPre, lambda);
#endif
        }
    }
};


CalibrationHarp::CalibrationHarp() { return; }

// void maskImage(cv::Mat &img, std::vector)

bool CalibrationHarp::measure(const std::string &image_path, double &_d,
                              double &_d_max) {
  // get image
  cv::Mat image = cv::imread(image_path, 0);
  image.copyTo(origin_image_);

  // get line edge points
  // get line support region
  LineSegmentDetector line_detector(0);
  std::vector<Vector4f> lines;
  std::vector<std::vector<Point2i>> line_points;
  std::vector<std::vector<int>> line_point_map;
  std::vector<double> width;
  std::vector<double> prec;
  std::vector<double> nfas;
  line_detector.detect(image, lines, line_points, line_point_map, width, prec,
                       nfas);
  // canny edge detect
  cv::Mat blur_image;
  cv::Mat edge_image;
  cv::GaussianBlur(image, blur_image, cv::Size(3, 3), 0);
  cv::Canny(blur_image, edge_image, 20, 40);
  // cv::imwrite("canny.jpg", edge_image);
  // cv::imshow("canny", edge_image);
  // cv::waitKey();
  // get edge line point in line support region
  int line_num = lines.size();
  std::vector<std::vector<Point2i>> edge_line_points(line_num);
  cv::Mat edge_point_image(edge_image.size(), edge_image.type(), cv::Scalar(0));
  for (int i = 0; i < line_point_map.size(); i++) {
    for (int j = 0; j < line_point_map[0].size(); j++) {
      if (edge_image.at<uchar>(i, j) == 255 && line_point_map[i][j] != 0) {
        edge_point_image.at<uchar>(i, j) = 255;
        edge_line_points[line_point_map[i][j] - 1].push_back(Point2i(j, i));
      }
    }
  }

  // cv::imwrite("edge_point_image.jpg", edge_point_image);
  // cv::imshow("edge_point_image", edge_point_image);
  // cv::waitKey();

  cv::Mat drawn_img;
  origin_image_.copyTo(drawn_img);
  // line_detector.drawSegments(drawn_img, lines);
  line_detector.drawSegments(drawn_img, edge_line_points);
  // cv::imwrite("edge_line_points.jpg", drawn_img);
  // cv::imshow("edge_line_points", drawn_img);
  // cv::waitKey();

  // interpolate line points
  std::vector<std::vector<Point2i>> interpolated_edge_points;
  interpolate_edge_points(lines, edge_line_points, interpolated_edge_points);
  // gaussion blur and subsample points
  std::vector<std::vector<Point2i>> subsampled_edge_points;
  gaussion_subsample_points(interpolated_edge_points, subsampled_edge_points,
                            20);

  origin_image_.copyTo(drawn_img);
  line_detector.drawSegments(drawn_img, subsampled_edge_points);
  // cv::imwrite("subsampled_edge_points.jpg", drawn_img);
  cv::imshow("subsampled_edge_points", drawn_img);
  cv::waitKey();
  cv::destroyAllWindows();
  // calculate error
  double d, d_max, d_c_median;
  calculate_error(subsampled_edge_points, d, d_max, d_c_median);

  std::cout << "d: " << d << " pixels" << std::endl;
  std::cout << "d_max: " << d_max << " pixels" << std::endl;
  _d = d;
  _d_max = d_max;
  // std::cout << "d_c_median: " << d_c_median << " pixels" << std::endl;

  return true;
}

bool CalibrationHarp::interpolate_edge_points(
    const std::vector<Vector4f> &lines,
    const std::vector<std::vector<Point2i>> &edge_points,
    std::vector<std::vector<Point2i>> &interpolated_edge_points) {
  std::vector<std::vector<Point2i>> sorted_edge_points(edge_points);
  int line_num = lines.size();

  // sort line points

  for (int i = 0; i < line_num; i++) {
    double rough_angle = abs(static_cast<double>(lines[0].x - lines[0].z) /
                             static_cast<double>(lines[0].y - lines[0].w));
    if (rough_angle < 1) {
      std::sort(sorted_edge_points[i].begin(), sorted_edge_points[i].end(),
                sortLineByY_function);
    } else {
      std::sort(sorted_edge_points[i].begin(), sorted_edge_points[i].end(),
                sortLineByX_function);
    }
  }

  // interpolate line points
  interpolated_edge_points.resize(line_num);
  std::vector<std::vector<double>> line_point_distance(line_num);
  for (int i = 0; i < line_num; i++) {
    if (edge_points[i].size() == 0)
      continue;
    // double line_length = (lines[i].x - lines[i].z) * (lines[i].x -
    // lines[i].z) +
    //                      (lines[i].y - lines[i].w) * (lines[i].y -
    //                      lines[i].w);
    // line_length = sqrt(line_length);
    line_point_distance[i].push_back(0);
    for (int idx = 0; idx < sorted_edge_points[i].size() - 1; idx++) {
      double dx =
          sorted_edge_points[i][idx].x - sorted_edge_points[i][idx + 1].x;
      double dy =
          sorted_edge_points[i][idx].y - sorted_edge_points[i][idx + 1].y;
      double dist = sqrt(dx * dx + dy * dy);
      line_point_distance[i].push_back(line_point_distance[i][idx] + dist);
    }
  }

  for (int i = 0; i < line_num; i++) {
    if (sorted_edge_points[i].size() == 0)
      continue;

    double L = line_point_distance[i][line_point_distance[i].size() - 1];
    double N = sorted_edge_points[i].size();
    double d = L / N;
    double L_pos = 0;
    double p_pos = 1;

    // std::cout << "\nd: " << d << " pixels\n";

    interpolated_edge_points[i].push_back(sorted_edge_points[i][0]);

    while (p_pos < N) {
      L_pos += d;
      while (p_pos < N && L_pos > line_point_distance[i][p_pos]) {
        p_pos++;
      }
      if (p_pos >= N)
        break;
      double a = L_pos - line_point_distance[i][p_pos - 1];
      double b = line_point_distance[i][p_pos] - L_pos;
      double x1 = sorted_edge_points[i][p_pos - 1].x;
      double y1 = sorted_edge_points[i][p_pos - 1].y;
      double x2 = sorted_edge_points[i][p_pos].x;
      double y2 = sorted_edge_points[i][p_pos].y;
      double new_x = b / (a + b) * x1 + a / (a + b) * x2;
      double new_y = b / (a + b) * y1 + a / (a + b) * y2;
      // interpolated_edge_points[i].push_back(Point2i(int(new_x + 0.5),
      // int(new_y + 0.5)));
      interpolated_edge_points[i].push_back(Point2i(int(new_x), int(new_y)));
    }
  }

#if UNDIST
  {
      std::vector<std::vector<cv::Point2d>> points;
      std::vector<std::vector<Point2i>> points_undistort;
      for (auto& v : subsampled_edge_points)
      {
          std::vector<cv::Point2d> t;
          for (auto& p : v)
          {
              t.push_back(cv::Point2d(p.x, p.y));
          }
          points.push_back(t);
      }
      Undistort udt(points, image.cols / 2, image.rows / 2, image.cols / 2);
      udt.proc();
      for (auto& v : udt.points_undistort)
      {
          std::vector<Point2i> t;
          for (auto& p : v)
          {
              t.push_back(Point2i(p.x, p.y));
          }
          points_undistort.push_back(t);
      }

      calculate_error(points_undistort, d, d_max, d_c_median);
      std::cout << "after undistort:" << std::endl;
      std::cout << "d: " << d << " pixels" << std::endl;
      std::cout << "d_max: " << d_max << " pixels" << std::endl;

      cv::Mat show = image.clone();
      line_detector.drawSegments(show, points_undistort);
      cv::resize(show, show, cv::Size(800, 600));
      cv::imshow("undistort", show);
      cv::waitKey();
  }
#endif

  return true;
}

bool CalibrationHarp::gaussion_subsample_points(
    const std::vector<std::vector<Point2i>> &interpolated_edge_points,
    std::vector<std::vector<Point2i>> &subsampled_edge_points, const int t) {
  int line_num = interpolated_edge_points.size();
  subsampled_edge_points.resize(line_num);

  const double sigma = 0.8 * sqrt(t * t - 1);
  const double deno = 1.0 / (sigma * sqrt(2.0 * M_PI));
  const double nume = -1.0 / (2.0 * sigma * sigma);

  // generate gauss matrix
  std::vector<double> gauss_matrix;
  double gauss_matrix_sum = 0;
  for (int i = 0, x = -t; x <= t; i++, x++) {
    double g = deno * exp(nume * x * x);
    gauss_matrix.push_back(g);
    gauss_matrix_sum += g;
  }
  // normalize
  for (int i = 0; i < gauss_matrix.size(); i++) {
    gauss_matrix[i] /= gauss_matrix_sum;
  }

  // gaussian blur
  for (int line_idx = 0; line_idx < line_num; line_idx++) {
    int point_size = interpolated_edge_points[line_idx].size();
    if (point_size == 0)
      continue;

    for (int p = t; p < point_size - t; p += t) {
      double gauss_x_sum = 0;
      double gauss_y_sum = 0;
      double gauss_sum = 0;
      for (int i = -t; i <= t; i++) {
        int k = p + i;
        if (k >= 0 && k < point_size) {
          double x = interpolated_edge_points[line_idx][k].x;
          double y = interpolated_edge_points[line_idx][k].y;
          gauss_x_sum += x * gauss_matrix[i + t];
          gauss_y_sum += y * gauss_matrix[i + t];
          gauss_sum += gauss_matrix[i + t];
        }
      }
      if (gauss_sum == 0)
        continue;
      int selected_x = static_cast<int>(gauss_x_sum / gauss_sum + 0.5);
      int selected_y = static_cast<int>(gauss_y_sum / gauss_sum + 0.5);
      // std::cout << selected_x << "\t" << selected_y << std::endl;

      subsampled_edge_points[line_idx].push_back(
          Point2i(selected_x, selected_y));
    }
  }

  return true;
}

bool CalibrationHarp::calculate_error(
    // const std::vector<Vector4f> &lines,
    // const std::vector< std::vector<Point2i> > &edge_points,
    const std::vector<std::vector<Point2i>> &subsampled_edge_point, double &d,
    double &d_max, double &d_c_median) {
  int real_line_num = 0;
  int total_point_num = 0;
  double S = 0;
  double S_max_total = 0;
  double S_d_max = 0;

  for (int i = 0; i < subsampled_edge_point.size(); i++) {
    if (subsampled_edge_point[i].size() == 0)
      continue;
    // std::cout << std::endl;
    int point_num = subsampled_edge_point[i].size();
    real_line_num++;
    total_point_num += point_num;

    double alpha, beta, gama;
    get_line_param(subsampled_edge_point[i], alpha, beta, gama);
    // std::cout << alpha << "\t" << beta << "\t" << gama << std::endl;

    double Si_max = -10000;
    double Si_min = 10000;
    for (int j = 0; j < point_num; j++) {
      double x = subsampled_edge_point[i][j].x;
      double y = subsampled_edge_point[i][j].y;
      double Si = (alpha * x + beta * y - gama);
      double Si_square = Si * Si;
      S += Si_square;
      if (Si > Si_max)
        Si_max = Si;
      if (Si < Si_min)
        Si_min = Si;

      // std::cout << "x=" << x << "   " << "y=" << y
      //           << "  Si=" << Si << std::endl;
    }
    // std::cout << "Si_max: " << Si_max << std::endl;
    // std::cout << "Si_min: " << Si_min << std::endl;
    S_max_total += (Si_max - Si_min) * (Si_max - Si_min);
    double max_current = Si_max - Si_min;
    if (max_current > S_d_max)
      S_d_max = max_current;
  }
  d = sqrt(S / static_cast<double>(total_point_num));
  d_max = sqrt(S_max_total / static_cast<double>(real_line_num));
  d_c_median = S_d_max;

  std::cout << "Detected " << real_line_num << " lines. " << total_point_num
            << " edge points.\n";
  return true;
}

bool CalibrationHarp::get_line_param(const std::vector<Point2i> &edge_points,
                                     double &alpha, double &beta,
                                     double &gama) {
  double Ax = 0, Ay = 0;
  int point_num = edge_points.size();

  for (int i = 0; i < point_num; i++) {
    Ax += edge_points[i].x;
    Ay += edge_points[i].y;
  }
  Ax /= static_cast<double>(point_num);
  Ay /= static_cast<double>(point_num);

  double theta = get_theta(edge_points, Ax, Ay);
  alpha = sin(theta);
  beta = -cos(theta);
  gama = Ax * sin(theta) - Ay * cos(theta);
}

double CalibrationHarp::get_theta(const std::vector<Point2i> &edge_points,
                                  const double &Ax, const double &Ay) {
  double Vxx = 0, Vxy = 0, Vyy = 0;
  int point_num = edge_points.size();
  for (int i = 0; i < point_num; i++) {
    double x = edge_points[i].x;
    double y = edge_points[i].y;
    Vxx += (x - Ax) * (x - Ax);
    Vxy += (x - Ax) * (y - Ay);
    Vyy += (y - Ay) * (y - Ay);
  }

  Vxx /= static_cast<double>(point_num);
  Vxy /= static_cast<double>(point_num);
  Vyy /= static_cast<double>(point_num);

  double theta = atan2(2 * Vxy, Vxx - Vyy) / 2.0;
  // std::cout << "theta = " << theta * 180 / M_PI << std::endl;
  return theta;
}