/*
 * Copyright (C) 2018-2019 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@sensetime.com>
 */
#ifndef LIDAR2CAR_UTILS_TRANSFORM_UTIL_HPP_
#define LIDAR2CAR_UTILS_TRANSFORM_UTIL_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

namespace autoCalib {
namespace calibration {

inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }
inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

class TransformUtil {
 public:
    TransformUtil() = default;
    ~TransformUtil() = default;

    static Eigen::Matrix4d GetMatrix(
        double x, double y, double z, double roll, double pitch, double yaw) {
        Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
        Eigen::Vector3d translation = GetTranslation(x, y, z);
        Eigen::Matrix3d rotation = GetRotation(roll, pitch, yaw);
        ret.block<3, 1>(0, 3) = translation;
        ret.block<3, 3>(0, 0) = rotation;
        return ret;
    }
    static Eigen::Matrix4d GetMatrix(const Eigen::Vector3d& translation,
                                     const Eigen::Matrix3d& rotation) {
        Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
        ret.block<3, 1>(0, 3) = translation;
        ret.block<3, 3>(0, 0) = rotation;
        return ret;
    }
    static Eigen::Vector3d GetTranslation(const Eigen::Matrix4d& matrix) {
        return Eigen::Vector3d(matrix(0, 3), matrix(1, 3), matrix(2, 3));
    }
    static Eigen::Vector3d GetTranslation(double x, double y, double z) {
        return Eigen::Vector3d(x, y, z);
    }
    static Eigen::Matrix3d GetRotation(const Eigen::Matrix4d& matrix) {
        return matrix.block<3, 3>(0, 0);
    }
    static Eigen::Matrix3d GetRotation(double roll, double pitch, double yaw) {
        Eigen::Matrix3d rotation;
        Eigen::Matrix3d Rz;
        Rz << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
        Eigen::Matrix3d Ry;
        Ry << cos(pitch), 0., sin(pitch), 0., 1., 0., -sin(pitch), 0.,
            cos(pitch);

        Eigen::Matrix3d Rx;
        Rx << 1., 0., 0., 0., cos(roll), -sin(roll), 0., sin(roll), cos(roll);
        rotation = Rz * Ry * Rx;
        return rotation;
    }
    static double GetX(const Eigen::Matrix4d& matrix) { return matrix(0, 3); }
    static double GetY(const Eigen::Matrix4d& matrix) { return matrix(1, 3); }
    static double GetZ(const Eigen::Matrix4d& matrix) { return matrix(2, 3); }
    /* avoid using it: matrix.block<3, 3>(0, 0).eulerAngles(0, 1, 2)[0];*/
    static double GetRoll(const Eigen::Matrix4d& matrix) {
        Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);
        double y = atan2(n(1), n(0));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y),
                         -o(0) * sin(y) + o(1) * cos(y));

        return r;
    }
    static double GetPitch(const Eigen::Matrix4d& matrix) {
        Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
        Eigen::Vector3d n = R.col(0);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));

        return p;
    }
    static double GetYaw(const Eigen::Matrix4d& matrix) {
        Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
        Eigen::Vector3d n = R.col(0);
        double y = atan2(n(1), n(0));

        return y;
    }

    static Eigen::Vector3d GetRPY(const Eigen::Matrix3d& matrix)
    {
        Eigen::Vector3d n = matrix.col(0);
        Eigen::Vector3d o = matrix.col(1);
        Eigen::Vector3d a = matrix.col(2);

        Eigen::Vector3d rpy(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        rpy(0) = r;
        rpy(1) = p;
        rpy(2) = y;

        return rpy;
    }

    // static Eigen::Vector3d GetRPY(const Eigen::Matrix4d& matrix)
    // {
    //     return GetRPY(matrix.block<3, 3>(0, 0));
    // }



    static void QV2Matrix(const Eigen::Quaterniond& quat,
                          const Eigen::Vector3d& vect,
                          Eigen::Matrix4d* mat) {
        *mat = Eigen::Matrix4d::Identity();
        mat->block<3, 3>(0, 0) = quat.matrix();
        mat->block<3, 1>(0, 3) = vect;
    }
    static void Matrix2QV(const Eigen::Matrix4d& mat,
                          Eigen::Quaterniond* quat,
                          Eigen::Vector3d* vect) {
        *quat = Eigen::Quaterniond(mat.block<3, 3>(0, 0));
        quat->normalize();
        *vect = mat.block<3, 1>(0, 3);
    }
    static void Matrix2Isometry(const Eigen::Matrix4d& mat,
                                Eigen::Isometry3d* iso) {
        iso->setIdentity();
        iso->linear() = mat.block<3, 3>(0, 0);
        iso->translation() = mat.block<3, 1>(0, 3);
    }
    static void Isometry2Matrix(const Eigen::Isometry3d& iso,
                                Eigen::Matrix4d* mat) {
        *mat = Eigen::Matrix4d::Identity();
        mat->block<3, 3>(0, 0) = iso.rotation();
        mat->block<3, 1>(0, 3) = iso.translation();
    }
    static void QV2Isometry(const Eigen::Quaterniond& quat,
                            const Eigen::Vector3d& vect,
                            Eigen::Isometry3d* iso) {
        iso->setIdentity();
        iso->linear() = quat.toRotationMatrix();
        iso->translation() = vect;
    }
    static void Isometry2QV(const Eigen::Isometry3d& iso,
                            Eigen::Quaterniond* quat,
                            Eigen::Vector3d* vect) {
        *quat = Eigen::Quaterniond(iso.rotation());
        *vect = iso.translation();
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }
};

}  // namespace calibration
}  // namespace autoCalib

#endif  //  LIDAR2CAR_UTILS_TRANSFORM_UTIL_HPP_
