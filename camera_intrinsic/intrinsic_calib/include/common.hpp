/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#pragma once

struct Point2i {
  Point2i() {}
  Point2i(int x, int y) {
    this->x = x;
    this->y = y;
  }
  int x;
  int y;
};

struct Point2f {
  Point2f() {}
  Point2f(float x, float y) {
    this->x = x;
    this->y = y;
  }
  float x;
  float y;
};

struct Vector4f {
  Vector4f() {}
  Vector4f(float x, float y, float z, float w) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->w = w;
  }
  float x;
  float y;
  float z;
  float w;
};

struct Vector4i {
  Vector4i() {}
  Vector4i(int x, int y, int z, int w) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->w = w;
  }
  int x;
  int y;
  int z;
  int w;
};