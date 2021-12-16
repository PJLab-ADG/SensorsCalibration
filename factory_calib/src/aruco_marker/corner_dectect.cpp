/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#include "aruco_marker/contours.c"
#include "aruco_marker/corner_detect.hpp"

namespace cameracalib {
namespace arucomarker {

bool ArucoMarkerDetector::detect(const std::vector<std::vector<float>> &imgGray,
                              const ArucoMarker &board_pattern,
                              CornerPoints *corner_pts) {
  std::vector<int> thres_box;
  float edge_fill_percent = 0.05;
  int kernal[3][3] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
  std::vector<bool> row_thresh(imgGray[0].size(), 0);
  std::vector<std::vector<bool>> thresh(imgGray.size(), row_thresh);
  std::vector<std::vector<bool>> dilate(imgGray.size(), row_thresh);
  CornerPoints pts;
  this->EstimateGrayThres(imgGray, &thres_box, edge_fill_percent);

  for (int i = 0; i < thres_box.size(); ++i) {
    pts.clear();
    std::vector<std::vector<Point2f>> candidates;
    std::vector<std::vector<std::vector<Point2f>>> candidatesSet;
    std::vector<std::vector<PointRDP>> contours;
    std::vector<std::vector<std::vector<PointRDP>>> contoursSet;

    int thresh_threshold = thres_box[i];
    this->ImageThreshold(imgGray, thresh_threshold, &thresh, edge_fill_percent);
    this->GetContours(thresh, &dilate, &contours, edge_fill_percent);
    this->findMarkerContours(dilate, candidates, contours, minPerimeterRate_,
                             maxPerimeterRate_, ApproxAccuracyRate_,
                             minCornerDistanceRate_, minDistanceToBorder_);
    if (contours.size() < 1)
      continue;

    // sort corners
    for (unsigned int i = 0; i < candidates.size(); i++) {
      double dx1 = candidates[i][1].x - candidates[i][0].x;
      double dy1 = candidates[i][1].y - candidates[i][0].y;
      double dx2 = candidates[i][2].x - candidates[i][0].x;
      double dy2 = candidates[i][2].y - candidates[i][0].y;
      double crossProduct = (dx1 * dy2) - (dy1 * dx2);
      if (crossProduct < 0.0) {
        std::swap(candidates[i][1], candidates[i][3]);
      }
    }

    filterTooCloseCandidates(candidates, candidatesSet, contours, contoursSet,
                             minMarkerDistanceRate_, detectInvertedMarker_);
    int ncandidates = candidatesSet[0].size();
    candidates = candidatesSet[0];

    for (int i = 0; i < candidates.size(); i++) {
      int ID = this->identifyOneCandidate(thresh, candidates[i], board_pattern);
      if (ID == -1)
        continue;
      pts.points.emplace_back(candidates[i]);
      pts.tag_ids.emplace_back(ID);
      if (pts.points.size() == board_pattern.id_box.size())
        break;
    }
    if (pts.points.size() >= min_detected_marker_num_) {
      *corner_pts = pts;
      return true;
    }
  }
  return false;
}

void ArucoMarkerDetector::filterTooCloseCandidates(
    const std::vector<std::vector<Point2f>> &candidatesIn,
    std::vector<std::vector<std::vector<Point2f>>> &candidatesSetOut,
    const std::vector<std::vector<PointRDP>> &contoursIn,
    std::vector<std::vector<std::vector<PointRDP>>> &contoursSetOut,
    double minMarkerDistanceRate, bool detectInvertedMarker) {
  std::vector<int> candGroup;
  candGroup.resize(candidatesIn.size(), -1);
  std::vector<std::vector<unsigned int>> groupedCandidates;
  for (unsigned int i = 0; i < candidatesIn.size(); i++) {
    for (unsigned int j = i + 1; j < candidatesIn.size(); j++) {

      int minimumPerimeter =
          std::min((int)contoursIn[i].size(), (int)contoursIn[j].size());

      // fc is the first corner considered on one of the markers, 4 combinations
      // are possible
      for (int fc = 0; fc < 4; fc++) {
        double distSq = 0;
        for (int c = 0; c < 4; c++) {
          // modC is the corner considering first corner is fc
          int modC = (c + fc) % 4;
          distSq += (candidatesIn[i][modC].x - candidatesIn[j][c].x) *
                        (candidatesIn[i][modC].x - candidatesIn[j][c].x) +
                    (candidatesIn[i][modC].y - candidatesIn[j][c].y) *
                        (candidatesIn[i][modC].y - candidatesIn[j][c].y);
        }
        distSq /= 4.;

        // if mean square distance is too low, remove the smaller one of the two
        // markers
        double minMarkerDistancePixels =
            double(minimumPerimeter) * minMarkerDistanceRate;
        if (distSq < minMarkerDistancePixels * minMarkerDistancePixels) {

          // i and j are not related to a group
          if (candGroup[i] < 0 && candGroup[j] < 0) {
            // mark candidates with their corresponding group number
            candGroup[i] = candGroup[j] = (int)groupedCandidates.size();

            // create group
            std::vector<unsigned int> grouped;
            grouped.emplace_back(i);
            grouped.emplace_back(j);
            groupedCandidates.emplace_back(grouped);
          }
          // i is related to a group
          else if (candGroup[i] > -1 && candGroup[j] == -1) {
            int group = candGroup[i];
            candGroup[j] = group;

            // add to group
            groupedCandidates[group].emplace_back(j);
          }
          // j is related to a group
          else if (candGroup[j] > -1 && candGroup[i] == -1) {
            int group = candGroup[j];
            candGroup[i] = group;

            // add to group
            groupedCandidates[group].emplace_back(i);
          }
        }
      }
    }
  }

  // save possible candidates
  candidatesSetOut.clear();
  contoursSetOut.clear();

  std::vector<std::vector<Point2f>> biggerCandidates;
  std::vector<std::vector<PointRDP>> biggerContours;
  std::vector<std::vector<Point2f>> smallerCandidates;
  std::vector<std::vector<PointRDP>> smallerContours;

  // save possible candidates
  for (unsigned int i = 0; i < groupedCandidates.size(); i++) {
    int smallerIdx = groupedCandidates[i][0];
    int biggerIdx = -1;

    // evaluate group elements
    for (unsigned int j = 1; j < groupedCandidates[i].size(); j++) {
      size_t currPerim = contoursIn[groupedCandidates[i][j]].size();

      // check if current contour is bigger
      if (biggerIdx < 0)
        biggerIdx = groupedCandidates[i][j];
      else if (currPerim >= contoursIn[biggerIdx].size())
        biggerIdx = groupedCandidates[i][j];

      // check if current contour is smaller
      if (currPerim < contoursIn[smallerIdx].size() && detectInvertedMarker)
        smallerIdx = groupedCandidates[i][j];
    }
    // add contours und candidates
    if (biggerIdx > -1) {

      biggerCandidates.emplace_back(candidatesIn[biggerIdx]);
      biggerContours.emplace_back(contoursIn[biggerIdx]);

      if (detectInvertedMarker) {
        smallerCandidates.emplace_back(alignContourOrder(
            candidatesIn[biggerIdx][0], candidatesIn[smallerIdx]));
        smallerContours.emplace_back(contoursIn[smallerIdx]);
      }
    }
  }
  for (int i = 0; i < candGroup.size(); i++) {
    if (candGroup[i] == -1) {
      biggerCandidates.emplace_back(candidatesIn[i]);
      biggerContours.emplace_back(contoursIn[i]);
    }
  }
  // to preserve the structure :: candidateSet< defaultCandidates,
  // whiteCandidates >
  // default candidates
  candidatesSetOut.emplace_back(biggerCandidates);
  contoursSetOut.emplace_back(biggerContours);
  // white candidates
  candidatesSetOut.emplace_back(smallerCandidates);
  contoursSetOut.emplace_back(smallerContours);
}

std::vector<Point2f>
ArucoMarkerDetector::alignContourOrder(Point2f corner,
                                    std::vector<Point2f> candidate) {
  uint8_t r = 0;
  double min =
      std::sqrt((corner.x - candidate[0].x) * (corner.x - candidate[0].x) +
                (corner.y - candidate[0].y) * (corner.y - candidate[0].y));
  for (uint8_t pos = 1; pos < 4; pos++) {
    double nDiff = std::sqrt(
        (corner.x - candidate[pos].x) * (corner.x - candidate[pos].x) +
        (corner.y - candidate[pos].y) * (corner.y - candidate[pos].y));
    if (nDiff < min) {
      r = pos;
      min = nDiff;
    }
  }
  std::rotate(candidate.begin(), candidate.begin() + r, candidate.end());
  return candidate;
}

Eigen::MatrixXf
ArucoMarkerDetector::getPerspectiveTransform(std::vector<Point2f> src,
                                          std::vector<Point2f> dst) {
  const int N_inv = 8;
  Eigen::MatrixXf xx(N_inv, N_inv);
  xx << src[0].x, src[0].y, 1, 0, 0, 0, -src[0].x * dst[0].x,
      -src[0].y * dst[0].x, src[1].x, src[1].y, 1, 0, 0, 0,
      -src[1].x * dst[1].x, -src[1].y * dst[1].x, src[2].x, src[2].y, 1, 0, 0,
      0, -src[2].x * dst[2].x, -src[2].y * dst[2].x, src[3].x, src[3].y, 1, 0,
      0, 0, -src[3].x * dst[3].x, -src[3].y * dst[3].x, 0, 0, 0, src[0].x,
      src[0].y, 1, -src[0].x * dst[0].y, -src[0].y * dst[0].y, 0, 0, 0,
      src[1].x, src[1].y, 1, -src[1].x * dst[1].y, -src[1].y * dst[1].y, 0, 0,
      0, src[2].x, src[2].y, 1, -src[2].x * dst[2].y, -src[2].y * dst[2].y, 0,
      0, 0, src[3].x, src[3].y, 1, -src[3].x * dst[3].y, -src[3].y * dst[3].y;
  Eigen::MatrixXf xx_inv = xx.inverse();
  Eigen::MatrixXf input(N_inv, 1);
  input << dst[0].x, dst[1].x, dst[2].x, dst[3].x, dst[0].y, dst[1].y, dst[2].y,
      dst[3].y;
  Eigen::MatrixXf output = xx_inv * input;
  // std::cout<<"test:"<<std::endl;
  // std::cout<<xx*output-input<<std::endl;
  output.conservativeResize(output.rows() + 1, output.cols());
  output(8, 0) = 1;
  return output;
}

void ArucoMarkerDetector::EstimateGrayThres(
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
    start_thres = static_cast<int>(mid_gray + mid_gray - aver_gray) % 256;
  else
    start_thres = static_cast<int>(aver_gray + aver_gray - mid_gray) % 256;
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

  std::cout << "aver_gray:" << aver_gray << std::endl;
  std::cout << "start_thres:" << start_thres << std::endl;
  std::cout << "stride:" << stride << std::endl;
  std::cout << "possible box:\n";
  for (size_t i = 0; i < possible_box.size(); i++) {
    std::cout << possible_box[i] << " ";
  }
  std::cout << std::endl;

  *thres_box = possible_box;
}

void ArucoMarkerDetector::ImageThreshold(
    const std::vector<std::vector<float>> &imgGray,
    const double &thresh_threshold, std::vector<std::vector<bool>> *thresh_vec,
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
  std::vector<std::vector<bool>> img_vec(height, std::vector<bool>(width));
  for (int i = edge_fill_thickness_x; i < height - edge_fill_thickness_x; i++) {
    for (int j = edge_fill_thickness_y; j < width - edge_fill_thickness_y;
         j++) {
      if (imgGray[i][j] < thresh_threshold) {
        img_vec[i][j] = 0;
      } else {
        img_vec[i][j] = 1;
      }
    }
  }
  *thresh_vec = img_vec;
}

void ArucoMarkerDetector::GetContours(
    const std::vector<std::vector<bool>> &dilation_vec,
    std::vector<std::vector<bool>> *contour_img,
    std::vector<std::vector<PointRDP>> *contours, float edge_fill_percent) {
  if (contours == nullptr) {
    return;
  }
  if (edge_fill_percent > 0.29)
    return;
  std::vector<std::vector<bool>> img = dilation_vec;
  int edge_fill_thickness_x =
      static_cast<int>(edge_fill_percent * dilation_vec.size() * 1.7);
  int edge_fill_thickness_y =
      static_cast<int>(edge_fill_percent * dilation_vec[0].size());

  for (size_t i = edge_fill_thickness_x;
       i < dilation_vec.size() - edge_fill_thickness_x; i++)
    for (size_t j = edge_fill_thickness_y;
         j < dilation_vec[i].size() - edge_fill_thickness_y; j++) {
      bool tmp = dilation_vec[i][j];
      if (tmp)
        img[i][j] = 0;
      else
        img[i][j] = 1;
    }

  *contour_img = img;
  for (size_t i = 1; i < img.size() - 1; i++) {
    for (size_t j = 1; j < img[i].size() - 1; j++) {
      if (img[i + 1][j] == 1 && img[i + 1][j - 1] == 1 && img[i][j - 1] == 1 &&
          img[i - 1][j - 1] == 1 && img[i - 1][j] == 1 &&
          img[i - 1][j + 1] == 1 && img[i][j + 1] == 1 &&
          img[i + 1][j + 1] == 1) {
        (*contour_img)[i][j] = 0;
      }
    }
  }
}

void ArucoMarkerDetector::findMarkerContours(
    const std::vector<std::vector<bool>> &_in,
    std::vector<std::vector<Point2f>> &candidates,
    std::vector<std::vector<PointRDP>> &contoursOut, double minPerimeterRate,
    double maxPerimeterRate, double accuracyRate, double minCornerDistanceRate,
    int minDistanceToBorder) {
  // calculate maximum and minimum sizes in pixels
  unsigned int minPerimeterPixels =
      (unsigned int)(minPerimeterRate * std::max(_in.size(), _in[0].size()));
  unsigned int maxPerimeterPixels =
      (unsigned int)(maxPerimeterRate * std::max(_in.size(), _in[0].size()));

  std::vector<std::vector<PointRDP>> contours;
  findContours(_in, contours);

  // now filter list of contours
  for (unsigned int i = 0; i < contours.size(); i++) {
    // check perimeter
    if (contours[i].size() < minPerimeterPixels ||
        contours[i].size() > maxPerimeterPixels)
      continue;

    // check is square and is convex
    std::vector<PointRDP> approxCurve;
    for (int approx_level = 5; approx_level < 8; approx_level++) {
      ApproxPolyDP(contours[i], static_cast<double>(approx_level),
                   &approxCurve);
      if (approxCurve.size() == 4)
        break;
    }
    if (approxCurve.size() != 4)
      continue;
    if (!isContourConvex(approxCurve))
      continue;

    // check min distance between corners
    double minDistSq = std::max(_in.size(), _in[0].size()) *
                       std::max(_in.size(), _in[0].size());
    for (int j = 0; j < 4; j++) {
      double d = (double)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) *
                     (double)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) +
                 (double)(approxCurve[j].y - approxCurve[(j + 1) % 4].y) *
                     (double)(approxCurve[j].y - approxCurve[(j + 1) % 4].y);
      minDistSq = std::min(minDistSq, d);
    }
    double minCornerDistancePixels =
        static_cast<double>(contours[i].size()) * minCornerDistanceRate;
    if (minDistSq < minCornerDistancePixels * minCornerDistancePixels)
      continue;

    // // check if it is too near to the image border
    bool tooNearBorder = false;
    for (int j = 0; j < 4; j++) {
      if (approxCurve[j].x < minDistanceToBorder ||
          approxCurve[j].y < minDistanceToBorder ||
          approxCurve[j].x > _in[0].size() - 1 - minDistanceToBorder ||
          approxCurve[j].y > _in.size() - 1 - minDistanceToBorder)
        tooNearBorder = true;
    }
    if (tooNearBorder)
      continue;

    // check if the contour coincides
    if (approxCurve[3].x - approxCurve[1].x == 0 &&
        approxCurve[3].y - approxCurve[1].y == 0)
      continue;

    // if it passes all the test, add to candidates vector
    std::vector<Point2f> currentCandidate;
    currentCandidate.resize(4);
    for (int j = 0; j < 4; j++) {
      Point2f tmp_pf;
      tmp_pf.x = (float)approxCurve[j].x;
      tmp_pf.y = (float)approxCurve[j].y;
      currentCandidate[j] = tmp_pf;
    }
    candidates.emplace_back(currentCandidate);
    contoursOut.emplace_back(contours[i]);
  }
}

void ArucoMarkerDetector::ApproxPolyDP(const std::vector<PointRDP> src_contour,
                                    double eps,
                                    std::vector<PointRDP> *rt_contour) {
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
  std::vector<PointRDP> dst_contour;
  PointRDP slice(0, 0);
  PointRDP right_slice(0, 0);
  int count = src_contour.size();
  int init_iters = 3;
  int i = 0, j, pos = 0, wpos, new_count = 0;
  bool le_eps = false;
  PointRDP start_pt(-1000000, -1000000);
  PointRDP end_pt(0, 0);
  PointRDP pt(0, 0);
  std::stack<PointRDP> s;
  eps *= eps;
  right_slice.x = 0;
  for (i = 0; i < init_iters; i++) {
    double max_dist = 0;
    pos = int(pos + right_slice.x) % count;

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
    slice.y = right_slice.x = int(right_slice.x + slice.x) % count;
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

  std::vector<PointRDP> tmp_contour;
  for (int i = 0; i < new_count; i++)
    tmp_contour.push_back(dst_contour[i]);
  *rt_contour = tmp_contour;
}

void ArucoMarkerDetector::findContours(
    const std::vector<std::vector<bool>> &_in,
    std::vector<std::vector<PointRDP>> &contours) {
  int **image;
  image = create2dArray(_in.size(), _in[0].size());
  for (int ii = 0; ii < _in.size(); ii++) {
    for (int jj = 0; jj < _in[0].size(); jj++) {
      if (_in[ii][jj])
        image[ii][jj] = 1;
      else
        image[ii][jj] = 0;
    }
  }
  int numrows = _in.size();
  int numcols = _in[0].size();
  struct Border NBD;
  struct Border LNBD;
  LNBD.border_type = HOLE_BORDER;
  NBD.border_type = HOLE_BORDER;
  NBD.seq_num = 1;

  struct nodeVector hierarchy_vector;
  initNodeVector(&hierarchy_vector);
  struct Node temp_node;
  resetNode(&temp_node);
  temp_node.border = NBD;
  addNodeVector(&hierarchy_vector, temp_node);

  // add in padding for both contour and hierarchy have the same offset.
  struct point2dVector contour_vector;
  initPoint2dVector(&contour_vector);
  struct PointRDP border;
  setPoint(&border, -1, -1);
  struct PointRDP *padding;
  padding = (struct PointRDP *)malloc(sizeof(struct PointRDP));
  padding[0] = border;
  addPoint2dVector(&contour_vector, padding);

  struct intVector contour_counter;
  initIntVector(&contour_counter);
  addIntVector(&contour_counter, 1);

  struct PointRDP p2;
  bool border_start_found;

  for (int r = 0; r < numrows; r++) {
    LNBD.seq_num = 1;
    LNBD.border_type = HOLE_BORDER;
    for (int c = 0; c < numcols; c++) {
      border_start_found = false;
      // Phase 1: Find border
      // If fij = 1 and fi, j-1 = 0, then decide that the pixel (i, j) is the
      // border following starting point
      // of an outer border, increment NBD, and (i2, j2) <- (i, j - 1).
      if ((image[r][c] == 1 && c - 1 < 0) ||
          (image[r][c] == 1 && image[r][c - 1] == 0)) {
        NBD.border_type = OUTER_BORDER;
        NBD.seq_num += 1;
        setPoint(&p2, r, c - 1);
        border_start_found = true;
      }

      // Else if fij >= 1 and fi,j+1 = 0, then decide that the pixel (i, j) is
      // the border following
      // starting point of a hole border, increment NBD, (i2, j2) ←(i, j + 1),
      // and LNBD ← fij in case fij > 1.
      else if (c + 1 < numcols && (image[r][c] >= 1 && image[r][c + 1] == 0)) {
        NBD.border_type = HOLE_BORDER;
        NBD.seq_num += 1;
        if (image[r][c] > 1) {
          LNBD.seq_num = image[r][c];
          LNBD.border_type =
              hierarchy_vector.vector[LNBD.seq_num - 1].border.border_type;
        }
        setPoint(&p2, r, c + 1);
        border_start_found = true;
      }

      if (border_start_found) {
        // Phase 2: Store Parent

        // current = new TreeNode(NBD);
        resetNode(&temp_node);
        if (NBD.border_type == LNBD.border_type) {
          temp_node.parent = hierarchy_vector.vector[LNBD.seq_num - 1].parent;
          temp_node.next_sibling =
              hierarchy_vector.vector[temp_node.parent - 1].first_child;
          hierarchy_vector.vector[temp_node.parent - 1].first_child =
              NBD.seq_num;
          temp_node.border = NBD;
          addNodeVector(&hierarchy_vector, temp_node);

          // cout << "indirect: " << NBD.seq_num << "  parent: " << LNBD.seq_num
          // <<endl;
        } else {
          if (hierarchy_vector.vector[LNBD.seq_num - 1].first_child != -1) {
            temp_node.next_sibling =
                hierarchy_vector.vector[LNBD.seq_num - 1].first_child;
          }

          temp_node.parent = LNBD.seq_num;
          hierarchy_vector.vector[LNBD.seq_num - 1].first_child = NBD.seq_num;
          temp_node.border = NBD;
          addNodeVector(&hierarchy_vector, temp_node);
          // cout << "direct: " << NBD.seq_num << "  parent: " << LNBD.seq_num
          // << endl;
        }

        // Phase 3: Follow border
        followBorder(image, numrows, numcols, r, c, p2, NBD, &contour_vector,
                     &contour_counter);
      }

      // Phase 4: Continue to next border
      // If fij != 1, then LNBD <- abs( fij ) and resume the raster scan from
      // the pixel(i, j + 1).
      // The algorithm terminates when the scan reaches the lower right corner
      // of the picture.
      if (abs(image[r][c]) > 1) {
        LNBD.seq_num = abs(image[r][c]);
        LNBD.border_type =
            hierarchy_vector.vector[LNBD.seq_num - 1].border.border_type;
      }
    }
  }

  int hierarchy_size;
  int contour_size;
  int contour_index_size;
  struct PointRDP **contours_p =
      trimPoint2dVector(&contour_vector, &contour_size);
  int *contours_index = trimIntVector(&contour_counter, &contour_index_size);
  for (int i = 0; i < contour_size; i++) {
    std::vector<PointRDP> row;

    if (contours_index[i] < 20 || contours_index[i] > 1500)
      continue;

    for (int j = 0; j < contours_index[i]; j++) {
      PointRDP tmp;
      tmp = contours_p[i][j];
      row.emplace_back(tmp);
    }
    contours.emplace_back(row);
  }
}

bool ArucoMarkerDetector::isContourConvex(const std::vector<PointRDP> &_contour) {
  double k = (_contour[3].y - _contour[2].y) / (_contour[3].x - _contour[2].x);
  if (k > 20) {
    if ((_contour[0].x - _contour[2].x) * (_contour[1].x - _contour[2].x) < 0)
      return false;
  } else {
    double b = _contour[2].y - k * _contour[2].x;
    double p0 = k * _contour[0].x + b - _contour[0].y;
    double p1 = k * _contour[1].x + b - _contour[1].y;
    if (p0 * p1 < 0)
      return false;
  }
  k = (_contour[2].y - _contour[1].y) / (_contour[2].x - _contour[1].x);
  if (k > 20) {
    return (_contour[0].x - _contour[1].x) * (_contour[3].x - _contour[1].x);
  }
  double b = _contour[2].y - k * _contour[2].x;
  double p0 = k * _contour[0].x + b - _contour[0].y;
  double p1 = k * _contour[3].x + b - _contour[3].y;
  return p0 * p1 >= 0;
}

void ArucoMarkerDetector::warpPerspective(
    const std::vector<std::vector<bool>> &_src,
    std::vector<std::vector<bool>> &_dst, Eigen::MatrixXf transformation,
    int dstSize) {
  Eigen::MatrixXf warp_trans(3, 3);
  warp_trans << transformation(0, 0), transformation(1, 0),
      transformation(2, 0), transformation(3, 0), transformation(4, 0),
      transformation(5, 0), transformation(6, 0), transformation(7, 0),
      transformation(8, 0);
  Eigen::MatrixXf inv_trans = warp_trans.inverse();
  for (int i = 0; i < dstSize; i++) {
    std::vector<bool> dst_row;
    for (int j = 0; j < dstSize; j++) {
      float org_x =
          (inv_trans(0, 0) * j + inv_trans(0, 1) * i + inv_trans(0, 2)) /
          (inv_trans(2, 0) * j + inv_trans(2, 1) * i + inv_trans(2, 2));
      float org_y =
          (inv_trans(1, 0) * j + inv_trans(1, 1) * i + inv_trans(1, 2)) /
          (inv_trans(2, 0) * j + inv_trans(2, 1) * i + inv_trans(2, 2));
      bool pixel = getPixel(_src, org_x, org_y);
      dst_row.emplace_back(pixel);
    }
    _dst.emplace_back(dst_row);
  }
}

bool ArucoMarkerDetector::getPixel(const std::vector<std::vector<bool>> &_src,
                                float org_x, float org_y) {
  int low_x = int(org_x);
  int high_x = low_x + 1;
  int low_y = int(org_y);
  int high_y = low_y + 1;
  if (low_x >= 0 && low_y >= 0 && high_x < _src[0].size() &&
      high_y < _src.size()) {
    double mid_low =
        (_src[low_y][high_x] - _src[low_y][low_x]) * (org_x - low_x) +
        _src[low_y][low_x];
    double mid_high =
        (_src[high_y][high_x] - _src[high_y][low_x]) * (org_x - low_x) +
        _src[high_y][low_x];
    bool output = static_cast<bool>(
        std::round((mid_high - mid_low) * (org_y - low_y) + mid_low));
    return output;
  }
  return 0;
}

int ArucoMarkerDetector::findID(std::vector<bool> ID_detect,
                             std::vector<std::vector<bool>> ID_org) {
  int max_cnt = 0;
  int ID = -1;
  for (int i = 0; i < ID_org.size(); i++) {
    int cnt = 0;
    for (int j = 0; j < 16; j++) {
      if (ID_detect[j] == ID_org[i][j])
        cnt++;
    }
    if (cnt > max_cnt) {
      max_cnt = cnt;
      ID = i;
    }
    cnt = 0;
    for (int j = 0; j < 16; j++) {
      if (ID_detect[j] == ID_org[i][3 - j / 4 + 4 * (j % 4)])
        cnt++;
    }
    if (cnt > max_cnt) {
      max_cnt = cnt;
      ID = i;
    }
    cnt = 0;
    for (int j = 0; j < 16; j++) {
      if (ID_detect[j] == ID_org[i][15 - j])
        cnt++;
    }
    if (cnt > max_cnt) {
      max_cnt = cnt;
      ID = i;
    }
    cnt = 0;
    for (int j = 0; j < 16; j++) {
      if (ID_detect[j] == ID_org[i][12 + j / 4 - 4 * (j % 4)])
        cnt++;
    }
    if (cnt > max_cnt) {
      max_cnt = cnt;
      ID = i;
    }
  }
  if (max_cnt < 12)
    return -1;
  return ID;
}

int ArucoMarkerDetector::extractBits(
    const std::vector<std::vector<bool>> &_image,
    const std::vector<Point2f> &_corners, int markerSize, int markerBorderBits,
    int cellSize, double cellMarginRate, double minStdDevOtsu,
    const std::vector<std::vector<bool>> &ID_org) {
  // number of bits in the marker
  int markerSizeWithBorders = markerSize + 2 * markerBorderBits;

  std::vector<std::vector<uchar>>
      resultImg; // marker image after removing perspective
  int resultImgSize = markerSizeWithBorders * cellSize;
  // Mat resultImgCorners(4, 1, CV_32FC2);
  std::vector<Point2f> resultImgCorners;
  Point2f tmp_pf;
  tmp_pf.x = 0;
  tmp_pf.y = 0;
  resultImgCorners.emplace_back(tmp_pf);
  tmp_pf.x = (float)resultImgSize - 1;
  resultImgCorners.emplace_back(tmp_pf);
  tmp_pf.y = tmp_pf.x;
  resultImgCorners.emplace_back(tmp_pf);
  tmp_pf.x = 0;
  resultImgCorners.emplace_back(tmp_pf);

  // remove perspective
  clock_t start, end;
  start = clock();
  float left = 99999, right = 0, top = 99999, bottom = 0;
  for (int i = 0; i < _corners.size(); i++) {
    if (_corners[i].x < left)
      left = _corners[i].x;
    if (_corners[i].x > right)
      right = _corners[i].x;
    if (_corners[i].y < top)
      top = _corners[i].y;
    if (_corners[i].y > bottom)
      bottom = _corners[i].y;
  }
  if (right - left < resultImgSize || bottom - top < resultImgSize)
    return -1;

  Eigen::MatrixXf transformation =
      getPerspectiveTransform(_corners, resultImgCorners);

  std::vector<std::vector<bool>> threImg;

  warpPerspective(_image, threImg, transformation, resultImgSize);
  end = clock();
  double endtime = (double)(end - start) / CLOCKS_PER_SEC;

  start = clock();

  int edgeBits = markerBorderBits * cellSize;
  int cellSize_col = cellSize * threImg[0].size() / resultImgSize;
  int cellSize_row = cellSize * threImg.size() / resultImgSize;
  int edgeBits_col = markerBorderBits * cellSize_col;
  int edgeBits_row = markerBorderBits * cellSize_row;
  float cellSize_row_f = float((threImg.size() - 2 * edgeBits_row)) / 4;
  float cellSize_col_f = float((threImg[0].size() - 2 * edgeBits_col)) / 4;
  int fault = 0;
  int all_white = 0, all_black = 0;
  std::vector<bool> ID_detect;
  for (int row = 0; row < markerSize; row++) {
    int i = edgeBits_row + cellSize_row_f * row;
    for (int col = 0; col < markerSize; col++) {
      int j = edgeBits_col + cellSize_col_f * col;
      int white = 0;
      int black = 0;
      for (int ii = i; ii - i < cellSize_row; ii++) {
        for (int jj = j; jj - j < cellSize_col; jj++) {
          if (threImg[ii][jj])
            white++;
          else
            black++;
        }
      }
      if (white > black) {
        all_white++;
        ID_detect.emplace_back(1);
      } else {
        all_black++;
        ID_detect.emplace_back(0);
      }
      if (float(white) / (cellSize_row) / (cellSize_col) > 0.3 &&
          float(white) / (cellSize_row) / (cellSize_col) < 0.7) {
        fault++;
      }
    }
  }
  int ID = findID(ID_detect, ID_org);
  if (ID == -1)
    return ID;

  float judge = float(fault) / markerSize / markerSize;
  end = clock();
  endtime = (double)(end - start) / CLOCKS_PER_SEC;

  if (!(judge < 0.4 && all_black != 0 && all_white != 0 &&
        std::abs((float)all_white / all_black - 1) < 1.3 &&
        std::abs((float)all_black / all_white - 1) < 1.3))
    return -1;

  return ID;
}

} // namespace arucomarker
} // namespace cameracalib