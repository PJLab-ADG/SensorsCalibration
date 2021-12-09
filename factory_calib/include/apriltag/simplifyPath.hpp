/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#pragma once

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

typedef struct PointRDP {
  double x;
  double y;

  PointRDP() {}
  PointRDP(double i, double j) {
    x = i;
    y = j;
  }

  PointRDP(int i, int j) {
    x = i;
    y = j;
  }
  double operator*(PointRDP rhs) const {
    return (this->x * rhs.y - rhs.x * this->y);
  }

  PointRDP operator-(PointRDP rhs) const {
    PointRDP p;
    p.x = this->x - rhs.x;
    p.y = this->y - rhs.y;
    return p;
  }

  double Norm() const { return sqrt(this->x * this->x + this->y * this->y); }

} PointRDP;

class simplifyPath {
  //"findMaximumDistance" used as part of implementation for RDP algorithm.
private:
  const std::pair<int, double>
  findMaximumDistance(const std::vector<PointRDP> &Points) const;

  //"simplifyWithRDP" returns the simplified path with a PointRDP vector. The
  //function takes in the paths to be simplified and a customerized thresholds
  //for the simplication.
public:
  std::vector<PointRDP> simplifyWithRDP(std::vector<PointRDP> &Points,
                                        double epsilon) const;
};
