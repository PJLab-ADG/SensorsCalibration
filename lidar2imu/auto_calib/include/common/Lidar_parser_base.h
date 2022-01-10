#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// constexpr float deg2rad = M_PI / 180.0f;

struct LaserCorrection {
  float rot_correction = 0.f;
  float vert_correction = 0.f;
  float dist_correction = 0.f;
  bool two_pt_correction_available = false;
  float dist_correction_x = 0.f;
  float dist_correction_y = 0.f;
  float vert_offset_correction = 0.f;
  float horiz_offset_correction = 0.f;
  int max_intensity = 255;
  int min_intensity = 0;
  float focal_distance = 0.f;
  float focal_slope = 0.f;

  /* cached values calculated when the calibration file is read */
  float cos_rot_correction = 1.0f;  ///< cosine of rot_correction
  float sin_rot_correction = 0.f;   ///< sine of rot_correction
  float cos_vert_correction = 0.f;  ///< cosine of vert_correction
  float sin_vert_correction = 1.0f; ///< sine of vert_correction

  int laser_ring = 0; ///< ring number for this laser
};

struct LidarPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    LidarPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint16_t, ring, ring)(double, timestamp, timestamp))
