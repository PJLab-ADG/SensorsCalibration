#ifndef REPROJECTION_HPP_
#define REPROJECTION_HPP_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>

struct ReprojectionError {
  ReprojectionError(const Eigen::Vector2d &img_pts_,
                    const Eigen::Vector2d &board_pts_)
      : _img_pts(img_pts_), _board_pts(board_pts_) {}

  template <typename T>
  bool operator()(const T *const instrinsics_, const T *const k_,
                  const T *const rt_, // 6 : angle axis and translation
                  T *residuls) const {
    //	Eigen::Vector3d hom_w(_board_pts(0), _board_pts(1), T(1.));
    T hom_w_t[3];
    hom_w_t[0] = T(_board_pts(0));
    hom_w_t[1] = T(_board_pts(1));
    hom_w_t[2] = T(1.);
    T hom_w_trans[3];
    ceres::AngleAxisRotatePoint(rt_, hom_w_t, hom_w_trans);
    hom_w_trans[0] += rt_[3];
    hom_w_trans[1] += rt_[4];
    hom_w_trans[2] += rt_[5];

    T c_x = hom_w_trans[0] / hom_w_trans[2];
    T c_y = hom_w_trans[1] / hom_w_trans[2];

    // distortion
    T r2 = c_x * c_x + c_y * c_y;
    T r4 = r2 * r2;
    T r_coeff = (T(1) + k_[0] * r2 + k_[1] * r4);
    T xd = c_x * r_coeff;
    T yd = c_y * r_coeff;

    // camera coord => image coord
    T predict_x = instrinsics_[0] * xd + instrinsics_[1] * yd + instrinsics_[2];
    T predict_y = instrinsics_[3] * yd + instrinsics_[4];

    // residus

    residuls[0] = _img_pts(0) - predict_x;
    residuls[1] = _img_pts(1) - predict_y;

    return true;
  }
  const Eigen::Vector2d _img_pts;
  const Eigen::Vector2d _board_pts;
};

#endif // REPROJECTION_HPP_
