/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#include "LSD.hpp"

LineSegmentDetector::LineSegmentDetector(int _refine, double _scale,
                                         double _sigma_scale, double _quant,
                                         double _ang_th, double _log_eps,
                                         double _density_th, int _n_bins)
    : img_width(0), img_height(0), LOG_NT(0), w_needed(false), p_needed(false),
      n_needed(false), SCALE(_scale), doRefine(_refine),
      SIGMA_SCALE(_sigma_scale), QUANT(_quant), ANG_TH(_ang_th),
      LOG_EPS(_log_eps), DENSITY_TH(_density_th), N_BINS(_n_bins) {
  if (!(_scale > 0 && _sigma_scale > 0 && _quant >= 0 && _ang_th > 0 &&
        _ang_th < 180 && _density_th >= 0 && _density_th < 1 && _n_bins > 0)) {
    std::cerr << "[ERROR] WRONG INPUT PARAM" << std::endl;
  }
}

void LineSegmentDetector::detect(
    cv::Mat _image, std::vector<Vector4f> &_lines,
    std::vector<std::vector<Point2i>> &_line_points,
    std::vector<std::vector<int>> &_line_point_map, std::vector<double> &_width,
    std::vector<double> &_prec, std::vector<double> &_nfa) {
  // image = _image.getMat();
  image = _image;
  if (image.empty() && !image.type() == CV_8UC1) {
    std::cerr << "[ERROR] WRONG INPUT IMAGE PARAM" << std::endl;
    return;
  }

  std::vector<double> w, p, n;
  // w_needed = _width.needed();
  // p_needed = _prec.needed();
  w_needed = true;
  p_needed = true;
  if (doRefine < LSD_REFINE_ADV)
    n_needed = false;
  else
    // n_needed = _nfa.needed();
    n_needed = false;

  _line_point_map.resize(image.rows, std::vector<int>(image.cols));
  flsd(_lines, _line_points, _line_point_map, w, p, n);

  if (w_needed)
    _width = w;
  if (p_needed)
    _prec = p;
  if (n_needed)
    _nfa = n;

  // Clear used structures
  ordered_points.clear();
}

void LineSegmentDetector::flsd(std::vector<Vector4f> &lines,
                               std::vector<std::vector<Point2i>> &line_points,
                               std::vector<std::vector<int>> &line_point_map,
                               std::vector<double> &widths,
                               std::vector<double> &precisions,
                               std::vector<double> &nfas) {
  // Angle tolerance
  const double prec = M_PI * ANG_TH / 180;
  const double p = ANG_TH / 180;
  const double rho = QUANT / sin(prec); // gradient magnitude threshold

  if (SCALE != 1) {
    cv::Mat gaussian_img;
    const double sigma = (SCALE < 1) ? (SIGMA_SCALE / SCALE) : (SIGMA_SCALE);
    const double sprec = 3;
    const unsigned int h =
        (unsigned int)(ceil(sigma * sqrt(2 * sprec * log(10.0))));
    cv::Size ksize(1 + 2 * h, 1 + 2 * h); // kernel size
    cv::GaussianBlur(image, gaussian_img, ksize, sigma);
    // Scale image to needed size
    cv::resize(gaussian_img, scaled_image, cv::Size(), SCALE, SCALE,
               cv::INTER_LINEAR);
    // cv::resize(gaussian_img, scaled_image, cv::Size(), SCALE, SCALE,
    // cv::INTER_CUBIC);
    ll_angle(rho, N_BINS);
  } else {
    scaled_image = image;
    ll_angle(rho, N_BINS);
  }

  LOG_NT = 5 * (log10(double(img_width)) + log10(double(img_height))) / 2 +
           log10(11.0);
  const size_t min_reg_size =
      size_t(-LOG_NT / log10(p)) *
      3; // minimal number of points in region that can give a meaningful event
  // const size_t min_reg_size = 1500;
  // // Initialize region only when needed
  // Mat region = Mat::zeros(scaled_image.size(), CV_8UC1);
  used = cv::Mat_<uchar>::zeros(scaled_image.size()); // zeros = NOTUSED
  std::vector<RegionPoint> reg;

  // Search for line segments
  for (size_t i = 0, points_size = ordered_points.size(); i < points_size;
       ++i) {
    const Point2i &point = ordered_points[i].p;
    // std::cout << point.x << "  " << point.y;
    // if((used.at<uchar>(point) == NOTUSED) && (angles.at<double>(point) !=
    // NOTDEF))
    if ((used.at<uchar>(point.y, point.x) == NOTUSED) &&
        (angles.at<double>(point.y, point.x) != NOTDEF)) {
      double reg_angle;
      region_grow(ordered_points[i].p, reg, reg_angle, prec);

      // Ignore small regions
      if (reg.size() < min_reg_size) {
        continue;
      }

      // Construct rectangular approximation for the region
      rect rec;
      region2rect(reg, reg_angle, prec, p, rec);

      double log_nfa = -1;
      if (doRefine > LSD_REFINE_NONE) {
        // At least REFINE_STANDARD lvl.
        if (!refine(reg, reg_angle, prec, p, rec, DENSITY_TH)) {
          continue;
        }

        if (doRefine >= LSD_REFINE_ADV) {
          // Compute NFA
          log_nfa = rect_improve(rec);
          if (log_nfa <= LOG_EPS) {
            continue;
          }
        }
      }
      // Found new line

      // Add the offset
      rec.x1 += 0.5;
      rec.y1 += 0.5;
      rec.x2 += 0.5;
      rec.y2 += 0.5;

      // scale the result values if a sub-sampling was performed
      if (SCALE != 1) {
        rec.x1 /= SCALE;
        rec.y1 /= SCALE;
        rec.x2 /= SCALE;
        rec.y2 /= SCALE;
        rec.width /= SCALE;
      }

      // Store the relevant data
      // std::cout << "\nFind Line " << line_points.size() << "("<< reg.size()
      // << ")!\n";
      lines.push_back(
          Vector4f(float(rec.x1), float(rec.y1), float(rec.x2), float(rec.y2)));
      line_points.push_back(std::vector<Point2i>());
      int line_index = line_points.size();
      // get_edge_point(reg, reg_angle, line_points[line_points.size() - 1]);
      for (size_t p_idx = 0; p_idx < reg.size(); p_idx++) {
        int x = int((reg[p_idx].x + 0.5) / SCALE);
        int y = int((reg[p_idx].y + 0.5) / SCALE);
        line_points[line_points.size() - 1].push_back(Point2i(x, y));
        if (y < line_point_map.size() && x < line_point_map[0].size()) {
          line_point_map[y][x] = line_index;
        }
      }
      if (w_needed)
        widths.push_back(rec.width);
      if (p_needed)
        precisions.push_back(rec.p);
      if (n_needed && doRefine >= LSD_REFINE_ADV)
        nfas.push_back(log_nfa);
    }
  }
}

void LineSegmentDetector::get_edge_point(std::vector<RegionPoint> &reg_points,
                                         const double &reg_angle,
                                         std::vector<Point2i> &edge_point) {
  edge_point.clear();
  std::sort(reg_points.begin(), reg_points.end(), sortRegionFunction);

  // Angle tolerance
  const double prec = M_PI * ANG_TH / 180;
  const double p = ANG_TH / 180;
  const double rho = QUANT / sin(prec); // gradient magnitude threshold

  int gap = 20;
  for (int i = 0; i < reg_points.size() - gap; i += gap) {
    rect rec;
    region2rect(std::vector<RegionPoint>(reg_points.begin() + i,
                                         reg_points.begin() + i + gap),
                reg_angle, prec, p, rec);
    rec.x1 += 0.5;
    rec.y1 += 0.5;
    rec.x2 += 0.5;
    rec.y2 += 0.5;
    edge_point.push_back(Point2i(int(rec.x1 / SCALE), int(rec.y1 / SCALE)));
    // edge_point.push_back(Point2i(int(rec.x / SCALE), int(rec.y / SCALE)));
    edge_point.push_back(Point2i(int(rec.x2 / SCALE), int(rec.y2 / SCALE)));
    // std::cout << int(rec.x1 / SCALE) << "\t" << int(rec.y1 / SCALE) <<
    // std::endl;
    // std::cout << int(rec.x / SCALE) << "\t" << int(rec.y / SCALE) <<
    // std::endl;
    // std::cout << int(rec.x2 / SCALE) << "\t" << int(rec.y2 / SCALE) <<
    // std::endl;
  }
}

void LineSegmentDetector::ll_angle(const double &threshold,
                                   const unsigned int &n_bins) {
  // Initialize data
  angles = cv::Mat_<double>(scaled_image.size());
  modgrad = cv::Mat_<double>(scaled_image.size());

  img_width = scaled_image.cols;
  img_height = scaled_image.rows;

  // Undefined the down and right boundaries
  angles.row(img_height - 1).setTo(NOTDEF);
  angles.col(img_width - 1).setTo(NOTDEF);

  // Computing gradient for remaining pixels
  double max_grad = -1;
  for (int y = 0; y < img_height - 1; ++y) {
    const uchar *scaled_image_row = scaled_image.ptr<uchar>(y);
    const uchar *next_scaled_image_row = scaled_image.ptr<uchar>(y + 1);
    double *angles_row = angles.ptr<double>(y);
    double *modgrad_row = modgrad.ptr<double>(y);
    for (int x = 0; x < img_width - 1; ++x) {
      int DA = next_scaled_image_row[x + 1] - scaled_image_row[x];
      int BC = scaled_image_row[x + 1] - next_scaled_image_row[x];
      int gx = DA + BC; // gradient x component
      int gy = DA - BC; // gradient y component
      double norm = std::sqrt((gx * gx + gy * gy) / 4.0); // gradient norm

      modgrad_row[x] = norm; // store gradient

      if (norm <= threshold) // norm too small, gradient no defined
      {
        angles_row[x] = NOTDEF;
      } else {
        angles_row[x] = cv::fastAtan2(float(gx), float(-gy)) *
                        DEG_TO_RADS; // gradient angle computation
        if (norm > max_grad) {
          max_grad = norm;
        }
      }
    }
  }

  // Compute histogram of gradient values
  double bin_coef = (max_grad > 0) ? double(n_bins - 1) / max_grad
                                   : 0; // If all image is smooth, max_grad <= 0
  for (int y = 0; y < img_height - 1; ++y) {
    const double *modgrad_row = modgrad.ptr<double>(y);
    for (int x = 0; x < img_width - 1; ++x) {
      normPoint _point;
      int i = int(modgrad_row[x] * bin_coef);
      _point.p = Point2i(x, y);
      _point.norm = i;
      ordered_points.push_back(_point);
    }
  }

  // Sort
  std::sort(ordered_points.begin(), ordered_points.end(), compare_norm);
}

void LineSegmentDetector::region_grow(const Point2i &s,
                                      std::vector<RegionPoint> &reg,
                                      double &reg_angle, const double &prec) {
  reg.clear();

  // Point to this region
  RegionPoint seed;
  seed.x = s.x;
  seed.y = s.y;
  seed.used = &used.at<uchar>(s.y, s.x);
  reg_angle = angles.at<double>(s.y, s.x);
  seed.angle = reg_angle;
  seed.modgrad = modgrad.at<double>(s.y, s.x);
  reg.push_back(seed);

  float sumdx = float(std::cos(reg_angle));
  float sumdy = float(std::sin(reg_angle));
  *seed.used = USED;

  // Try neighboring regions
  for (size_t i = 0; i < reg.size(); i++) {
    const RegionPoint &rpoint = reg[i];
    int xx_min = std::max(rpoint.x - 1, 0),
        xx_max = std::min(rpoint.x + 1, img_width - 1);
    int yy_min = std::max(rpoint.y - 1, 0),
        yy_max = std::min(rpoint.y + 1, img_height - 1);
    for (int yy = yy_min; yy <= yy_max; ++yy) {
      uchar *used_row = used.ptr<uchar>(yy);
      const double *angles_row = angles.ptr<double>(yy);
      const double *modgrad_row = modgrad.ptr<double>(yy);
      for (int xx = xx_min; xx <= xx_max; ++xx) {
        uchar &is_used = used_row[xx];
        if (is_used != USED && (isAligned(xx, yy, reg_angle, prec))) {
          const double &angle = angles_row[xx];
          // Add point
          is_used = USED;
          RegionPoint region_point;
          region_point.x = xx;
          region_point.y = yy;
          region_point.used = &is_used;
          region_point.modgrad = modgrad_row[xx];
          region_point.angle = angle;
          reg.push_back(region_point);

          // Update region's angle
          sumdx += cos(float(angle));
          sumdy += sin(float(angle));
          // reg_angle is used in the isAligned, so it needs to be updates?
          reg_angle = cv::fastAtan2(sumdy, sumdx) * DEG_TO_RADS;
        }
      }
    }
  }
}

void LineSegmentDetector::region2rect(const std::vector<RegionPoint> &reg,
                                      const double reg_angle, const double prec,
                                      const double p, rect &rec) const {
  double x = 0, y = 0, sum = 0;
  for (size_t i = 0; i < reg.size(); ++i) {
    const RegionPoint &pnt = reg[i];
    const double &weight = pnt.modgrad;
    x += double(pnt.x) * weight;
    y += double(pnt.y) * weight;
    sum += weight;
  }

  // Weighted sum must differ from 0
  if (sum <= 0) {
    std::cerr << "[ERROR] WEIGHTED SUM ERROR IN region2rect" << std::endl;
    return;
  }

  x /= sum;
  y /= sum;

  double theta = get_theta(reg, x, y, reg_angle, prec);

  // Find length and width
  double dx = cos(theta);
  double dy = sin(theta);
  double l_min = 0, l_max = 0, w_min = 0, w_max = 0;

  for (size_t i = 0; i < reg.size(); ++i) {
    double regdx = double(reg[i].x) - x;
    double regdy = double(reg[i].y) - y;

    double l = regdx * dx + regdy * dy;
    double w = -regdx * dy + regdy * dx;

    if (l > l_max)
      l_max = l;
    else if (l < l_min)
      l_min = l;
    if (w > w_max)
      w_max = w;
    else if (w < w_min)
      w_min = w;
  }

  // Store values
  rec.x1 = x + l_min * dx;
  rec.y1 = y + l_min * dy;
  rec.x2 = x + l_max * dx;
  rec.y2 = y + l_max * dy;
  rec.width = w_max - w_min;
  rec.x = x;
  rec.y = y;
  rec.theta = theta;
  rec.dx = dx;
  rec.dy = dy;
  rec.prec = prec;
  rec.p = p;

  // Min width of 1 pixel
  if (rec.width < 1.0)
    rec.width = 1.0;
}

double LineSegmentDetector::get_theta(const std::vector<RegionPoint> &reg,
                                      const double &x, const double &y,
                                      const double &reg_angle,
                                      const double &prec) const {
  double Ixx = 0.0;
  double Iyy = 0.0;
  double Ixy = 0.0;

  // Compute inertia matrix
  for (size_t i = 0; i < reg.size(); ++i) {
    const double &regx = reg[i].x;
    const double &regy = reg[i].y;
    const double &weight = reg[i].modgrad;
    double dx = regx - x;
    double dy = regy - y;
    Ixx += dy * dy * weight;
    Iyy += dx * dx * weight;
    Ixy -= dx * dy * weight;
  }

  // Check if inertia matrix is null
  if (double_equal(Ixx, 0) && double_equal(Iyy, 0) && double_equal(Ixy, 0)) {
    std::cerr << "[ERROR] PARAM ERROR IN get_theta" << std::endl;
    return 0;
  };

  // Compute smallest eigenvalue
  double lambda =
      0.5 * (Ixx + Iyy - sqrt((Ixx - Iyy) * (Ixx - Iyy) + 4.0 * Ixy * Ixy));

  // Compute angle
  double theta =
      (fabs(Ixx) > fabs(Iyy))
          ? double(cv::fastAtan2(float(lambda - Ixx), float(Ixy)))
          : double(cv::fastAtan2(float(Ixy), float(lambda - Iyy))); // in degs
  theta *= DEG_TO_RADS;

  // Correct angle by 180 deg if necessary
  if (angle_diff(theta, reg_angle) > prec) {
    theta += M_PI;
  }

  return theta;
}

bool LineSegmentDetector::refine(std::vector<RegionPoint> &reg,
                                 double reg_angle, const double prec, double p,
                                 rect &rec, const double &density_th) {
  double density =
      double(reg.size()) / (dist(rec.x1, rec.y1, rec.x2, rec.y2) * rec.width);

  if (density >= density_th) {
    return true;
  }

  // Try to reduce angle tolerance
  double xc = double(reg[0].x);
  double yc = double(reg[0].y);
  const double &ang_c = reg[0].angle;
  double sum = 0, s_sum = 0;
  int n = 0;

  for (size_t i = 0; i < reg.size(); ++i) {
    *(reg[i].used) = NOTUSED;
    if (dist(xc, yc, reg[i].x, reg[i].y) < rec.width) {
      const double &angle = reg[i].angle;
      double ang_d = angle_diff_signed(angle, ang_c);
      sum += ang_d;
      s_sum += ang_d * ang_d;
      ++n;
    }
  }
  if (n <= 0) {
    std::cerr << "[ERROR] PARAM ERROR IN refine" << std::endl;
    return false;
  }
  double mean_angle = sum / double(n);
  // 2 * standard deviation
  double tau = 2.0 * sqrt((s_sum - 2.0 * mean_angle * sum) / double(n) +
                          mean_angle * mean_angle);

  // Try new region
  region_grow(Point2i(reg[0].x, reg[0].y), reg, reg_angle, tau);

  if (reg.size() < 2) {
    return false;
  }

  region2rect(reg, reg_angle, prec, p, rec);
  density =
      double(reg.size()) / (dist(rec.x1, rec.y1, rec.x2, rec.y2) * rec.width);

  if (density < density_th) {
    return reduce_region_radius(reg, reg_angle, prec, p, rec, density,
                                density_th);
  } else {
    return true;
  }
}

bool LineSegmentDetector::reduce_region_radius(std::vector<RegionPoint> &reg,
                                               double reg_angle,
                                               const double prec, double p,
                                               rect &rec, double density,
                                               const double &density_th) {
  // Compute region's radius
  double xc = double(reg[0].x);
  double yc = double(reg[0].y);
  double radSq1 = distSq(xc, yc, rec.x1, rec.y1);
  double radSq2 = distSq(xc, yc, rec.x2, rec.y2);
  double radSq = radSq1 > radSq2 ? radSq1 : radSq2;

  while (density < density_th) {
    radSq *= 0.75 * 0.75; // Reduce region's radius to 75% of its value
    // Remove points from the region and update 'used' map
    for (size_t i = 0; i < reg.size(); ++i) {
      if (distSq(xc, yc, double(reg[i].x), double(reg[i].y)) > radSq) {
        // Remove point from the region
        *(reg[i].used) = NOTUSED;
        std::swap(reg[i], reg[reg.size() - 1]);
        reg.pop_back();
        --i; // To avoid skipping one point
      }
    }

    if (reg.size() < 2) {
      return false;
    }

    // Re-compute rectangle
    region2rect(reg, reg_angle, prec, p, rec);

    // Re-compute region points density
    density =
        double(reg.size()) / (dist(rec.x1, rec.y1, rec.x2, rec.y2) * rec.width);
  }

  return true;
}

double LineSegmentDetector::rect_improve(rect &rec) const {
  double delta = 0.5;
  double delta_2 = delta / 2.0;

  double log_nfa = rect_nfa(rec);

  if (log_nfa > LOG_EPS)
    return log_nfa; // Good rectangle

  // Try to improve
  // Finer precision
  rect r = rect(rec); // Copy
  for (int n = 0; n < 5; ++n) {
    r.p /= 2;
    r.prec = r.p * M_PI;
    double log_nfa_new = rect_nfa(r);
    if (log_nfa_new > log_nfa) {
      log_nfa = log_nfa_new;
      rec = rect(r);
    }
  }
  if (log_nfa > LOG_EPS)
    return log_nfa;

  // Try to reduce width
  r = rect(rec);
  for (unsigned int n = 0; n < 5; ++n) {
    if ((r.width - delta) >= 0.5) {
      r.width -= delta;
      double log_nfa_new = rect_nfa(r);
      if (log_nfa_new > log_nfa) {
        rec = rect(r);
        log_nfa = log_nfa_new;
      }
    }
  }
  if (log_nfa > LOG_EPS)
    return log_nfa;

  // Try to reduce one side of rectangle
  r = rect(rec);
  for (unsigned int n = 0; n < 5; ++n) {
    if ((r.width - delta) >= 0.5) {
      r.x1 += -r.dy * delta_2;
      r.y1 += r.dx * delta_2;
      r.x2 += -r.dy * delta_2;
      r.y2 += r.dx * delta_2;
      r.width -= delta;
      double log_nfa_new = rect_nfa(r);
      if (log_nfa_new > log_nfa) {
        rec = rect(r);
        log_nfa = log_nfa_new;
      }
    }
  }
  if (log_nfa > LOG_EPS)
    return log_nfa;

  // Try to reduce other side of rectangle
  r = rect(rec);
  for (unsigned int n = 0; n < 5; ++n) {
    if ((r.width - delta) >= 0.5) {
      r.x1 -= -r.dy * delta_2;
      r.y1 -= r.dx * delta_2;
      r.x2 -= -r.dy * delta_2;
      r.y2 -= r.dx * delta_2;
      r.width -= delta;
      double log_nfa_new = rect_nfa(r);
      if (log_nfa_new > log_nfa) {
        rec = rect(r);
        log_nfa = log_nfa_new;
      }
    }
  }
  if (log_nfa > LOG_EPS)
    return log_nfa;

  // Try finer precision
  r = rect(rec);
  for (unsigned int n = 0; n < 5; ++n) {
    if ((r.width - delta) >= 0.5) {
      r.p /= 2;
      r.prec = r.p * M_PI;
      double log_nfa_new = rect_nfa(r);
      if (log_nfa_new > log_nfa) {
        rec = rect(r);
        log_nfa = log_nfa_new;
      }
    }
  }

  return log_nfa;
}

double LineSegmentDetector::rect_nfa(const rect &rec) const {
  int total_pts = 0, alg_pts = 0;
  double half_width = rec.width / 2.0;
  double dyhw = rec.dy * half_width;
  double dxhw = rec.dx * half_width;

  edge ordered_x[4];
  edge *min_y = &ordered_x[0];
  edge *max_y = &ordered_x[0]; // Will be used for loop range

  ordered_x[0].p.x = int(rec.x1 - dyhw);
  ordered_x[0].p.y = int(rec.y1 + dxhw);
  ordered_x[0].taken = false;
  ordered_x[1].p.x = int(rec.x2 - dyhw);
  ordered_x[1].p.y = int(rec.y2 + dxhw);
  ordered_x[1].taken = false;
  ordered_x[2].p.x = int(rec.x2 + dyhw);
  ordered_x[2].p.y = int(rec.y2 - dxhw);
  ordered_x[2].taken = false;
  ordered_x[3].p.x = int(rec.x1 + dyhw);
  ordered_x[3].p.y = int(rec.y1 - dxhw);
  ordered_x[3].taken = false;

  std::sort(ordered_x, ordered_x + 4, AsmallerB_XoverY);

  // Find min y. And mark as taken. find max y.
  for (unsigned int i = 1; i < 4; ++i) {
    if (min_y->p.y > ordered_x[i].p.y) {
      min_y = &ordered_x[i];
    }
    if (max_y->p.y < ordered_x[i].p.y) {
      max_y = &ordered_x[i];
    }
  }
  min_y->taken = true;

  // Find leftmost untaken point;
  edge *leftmost = 0;
  for (unsigned int i = 0; i < 4; ++i) {
    if (!ordered_x[i].taken) {
      if (!leftmost) // if uninitialized
      {
        leftmost = &ordered_x[i];
      } else if (leftmost->p.x > ordered_x[i].p.x) {
        leftmost = &ordered_x[i];
      }
    }
  }
  if (leftmost == NULL) {
    std::cerr << "[ERROR] INVALID PARAM IN rect_nfa" << std::endl;
    return 0;
  }
  leftmost->taken = true;

  // Find rightmost untaken point;
  edge *rightmost = 0;
  for (unsigned int i = 0; i < 4; ++i) {
    if (!ordered_x[i].taken) {
      if (!rightmost) // if uninitialized
      {
        rightmost = &ordered_x[i];
      } else if (rightmost->p.x < ordered_x[i].p.x) {
        rightmost = &ordered_x[i];
      }
    }
  }
  if (rightmost == NULL) {
    std::cerr << "[ERROR] INVALID PARAM IN rect_nfa" << std::endl;
    return 0;
  }
  rightmost->taken = true;

  // Find last untaken point;
  edge *tailp = 0;
  for (unsigned int i = 0; i < 4; ++i) {
    if (!ordered_x[i].taken) {
      if (!tailp) // if uninitialized
      {
        tailp = &ordered_x[i];
      } else if (tailp->p.x > ordered_x[i].p.x) {
        tailp = &ordered_x[i];
      }
    }
  }
  if (tailp == NULL) {
    std::cerr << "[ERROR] INVALID PARAM IN rect_nfa" << std::endl;
    return 0;
  }
  tailp->taken = true;

  double flstep =
      (min_y->p.y != leftmost->p.y)
          ? (min_y->p.x - leftmost->p.x) / (min_y->p.y - leftmost->p.y)
          : 0; // first left step
  double slstep =
      (leftmost->p.y != tailp->p.x)
          ? (leftmost->p.x - tailp->p.x) / (leftmost->p.y - tailp->p.x)
          : 0; // second left step

  double frstep =
      (min_y->p.y != rightmost->p.y)
          ? (min_y->p.x - rightmost->p.x) / (min_y->p.y - rightmost->p.y)
          : 0; // first right step
  double srstep =
      (rightmost->p.y != tailp->p.x)
          ? (rightmost->p.x - tailp->p.x) / (rightmost->p.y - tailp->p.x)
          : 0; // second right step

  double lstep = flstep, rstep = frstep;

  double left_x = min_y->p.x, right_x = min_y->p.x;

  // Loop around all points in the region and count those that are aligned.
  int min_iter = min_y->p.y;
  int max_iter = max_y->p.y;
  for (int y = min_iter; y <= max_iter; ++y) {
    if (y < 0 || y >= img_height)
      continue;

    for (int x = int(left_x); x <= int(right_x); ++x) {
      if (x < 0 || x >= img_width)
        continue;

      ++total_pts;
      if (isAligned(x, y, rec.theta, rec.prec)) {
        ++alg_pts;
      }
    }

    if (y >= leftmost->p.y) {
      lstep = slstep;
    }
    if (y >= rightmost->p.y) {
      rstep = srstep;
    }

    left_x += lstep;
    right_x += rstep;
  }

  return nfa(total_pts, alg_pts, rec.p);
}

double LineSegmentDetector::nfa(const int &n, const int &k,
                                const double &p) const {
  // Trivial cases
  if (n == 0 || k == 0) {
    return -LOG_NT;
  }
  if (n == k) {
    return -LOG_NT - double(n) * log10(p);
  }

  double p_term = p / (1 - p);

  double log1term = (double(n) + 1) - log_gamma(double(k) + 1) -
                    log_gamma(double(n - k) + 1) + double(k) * log(p) +
                    double(n - k) * log(1.0 - p);
  double term = exp(log1term);

  if (double_equal(term, 0)) {
    if (k > n * p)
      return -log1term / M_LN10 - LOG_NT;
    else
      return -LOG_NT;
  }

  // Compute more terms if needed
  double bin_tail = term;
  double tolerance = 0.1; // an error of 10% in the result is accepted
  for (int i = k + 1; i <= n; ++i) {
    double bin_term = double(n - i + 1) / double(i);
    double mult_term = bin_term * p_term;
    term *= mult_term;
    bin_tail += term;
    if (bin_term < 1) {
      double err =
          term *
          ((1 - pow(mult_term, double(n - i + 1))) / (1 - mult_term) - 1);
      if (err < tolerance * std::fabs(-log10(bin_tail) - LOG_NT) * bin_tail)
        break;
    }
  }
  return -log10(bin_tail) - LOG_NT;
}

inline bool LineSegmentDetector::isAligned(int x, int y, const double &theta,
                                           const double &prec) const {
  if (x < 0 || y < 0 || x >= angles.cols || y >= angles.rows) {
    return false;
  }
  const double &a = angles.at<double>(y, x);
  if (a == NOTDEF) {
    return false;
  }

  // It is assumed that 'theta' and 'a' are in the range [-pi,pi]
  double n_theta = theta - a;
  if (n_theta < 0) {
    n_theta = -n_theta;
  }
  if (n_theta > M_3_2_PI) {
    n_theta -= M_2__PI;
    if (n_theta < 0)
      n_theta = -n_theta;
  }

  return n_theta <= prec;
}

void LineSegmentDetector::drawSegments(
    cv::Mat &_image, const std::vector<std::vector<Point2i>> line_points) {
  if (_image.empty()) {
    std::cerr << "[ERROR] WRONG INPUT IMAGE" << std::endl;
  }

  if (_image.channels() == 1) {
    cv::cvtColor(_image, _image, cv::COLOR_GRAY2BGR);
  }

  const int line_num = line_points.size();
  // cv::Scalar color(0, 0, 255);
  // // for(int line_idx = 0; line_idx < line_num; line_idx ++){
  // for(int line_idx = 0; line_idx < line_num - 1; line_idx ++){
  //     if (line_idx % 2 == 0){
  //         color = cv::Scalar(0, 0, 255);
  //     }
  //     else{
  //         color = cv::Scalar(255, 0, 0);
  //     }
  //     for(size_t p_idx = 0; p_idx < line_points[line_idx].size(); p_idx+=2){
  //         const cv::Point p(line_points[line_idx][p_idx].x,
  //                           line_points[line_idx][p_idx].y);
  //         // cv::circle(_image, p, 1, color, 0);
  //         const cv::Point p2(line_points[line_idx][p_idx+ 1].x,
  //                 line_points[line_idx][p_idx + 1].y);
  //         cv::line(_image, p, p2, color, 1);
  //     }
  // }

  cv::Scalar color(0, 0, 255);
  // for(int line_idx = 0; line_idx < line_num; line_idx ++){
  for (int line_idx = 0; line_idx < line_num; line_idx++) {
    if (line_idx % 2 == 0) {
      color = cv::Scalar(0, 55, 255);
    } else {
      color = cv::Scalar(155, 255, 50);
    }
    for (size_t p_idx = 0; p_idx < line_points[line_idx].size(); p_idx++) {
      const cv::Point p(line_points[line_idx][p_idx].x,
                        line_points[line_idx][p_idx].y);
      // cv::circle(_image, p, 1, color, 0);
      cv::circle(_image, p, 2, color, 2);
    }
  }
}

void LineSegmentDetector::drawSegments(cv::Mat &_image,
                                       const std::vector<Vector4f> &lines) {
  if (_image.empty()) {
    std::cerr << "[ERROR] WRONG INPUT IMAGE" << std::endl;
  }

  if (_image.channels() == 1) {
    cv::cvtColor(_image, _image, cv::COLOR_GRAY2BGR);
  }

  const int N = lines.size();
  cv::Scalar color(0, 0, 255);

  // Draw segments
  for (int i = 0; i < N; ++i) {
    if (i % 2 == 0) {
      color = cv::Scalar(0, 0, 255);
    } else {
      color = cv::Scalar(255, 0, 0);
    }
    const Vector4f &v = lines[i];
    const cv::Point2f b(v.x, v.y);
    const cv::Point2f e(v.z, v.w);
    cv::line(_image, b, e, color, 1);
  }
}
