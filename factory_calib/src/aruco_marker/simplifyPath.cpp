/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#include "aruco_marker/simplifyPath.hpp"
#include <string>

// using std::vector;
// Given an array of points, "findMaximumDistance" calculates the GPS PointRDP
// which have largest distance from the line formed by first and last points in
// RDP algorithm. Returns the index of the PointRDP in the array and the
// distance.

const std::pair<int, double>
simplifyPath::findMaximumDistance(const std::vector<PointRDP> &Points) const {
  PointRDP firstpoint = Points[0];
  PointRDP lastpoint = Points[Points.size() - 1];
  int index = 0;     // index to be returned
  double Mdist = -1; // the Maximum distance to be returned

  // distance calculation
  PointRDP p = lastpoint - firstpoint;
  for (int i = 1; i < Points.size() - 1;
       i++) { // traverse through second PointRDP to second last PointRDP
    PointRDP pp = Points[i] - firstpoint;
    double Dist =
        fabs(pp * p) / p.Norm(); // formula for PointRDP-to-line distance
    if (Dist > Mdist) {
      Mdist = Dist;
      index = i;
    }
  }
  return std::make_pair(index, Mdist);
}

std::vector<PointRDP>
simplifyPath::simplifyWithRDP(std::vector<PointRDP> &Points,
                              double epsilon) const {
  if (Points.size() < 3) { // base case 1
    return Points;
  }
  std::pair<int, double> maxDistance = findMaximumDistance(Points);
  if (maxDistance.second >= epsilon) {
    int index = maxDistance.first;
    std::vector<PointRDP>::iterator it = Points.begin();
    std::vector<PointRDP> path1(Points.begin(),
                                it + index + 1); // new path l1 from 0 to index
    std::vector<PointRDP> path2(it + index,
                                Points.end()); // new path l2 from index to last

    std::vector<PointRDP> r1 = simplifyWithRDP(path1, epsilon);
    std::vector<PointRDP> r2 = simplifyWithRDP(path2, epsilon);

    // Concat simplified path1 and path2 together
    std::vector<PointRDP> rs(r1);
    rs.pop_back();
    rs.insert(rs.end(), r2.begin(), r2.end());
    return rs;
  } else { // base case 2, all points between are to be removed.
    std::vector<PointRDP> r(1, Points[0]);
    r.emplace_back(Points[Points.size() - 1]);
    return r;
  }
}