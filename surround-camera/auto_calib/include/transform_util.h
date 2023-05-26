#ifndef TRANSFORM_UTIL_H_
#define TRANSFORM_UTIL_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }
inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

class TransformUtil {
public:
  TransformUtil() = default;
  ~TransformUtil() = default;

  static cv::Mat eigen2mat(Eigen::Matrix3d A) {
    cv::Mat B;
    cv::eigen2cv(A, B);

    return B;
  }

  static Eigen::Matrix4d GetMatrix(const Eigen::Vector3d &translation,
                                   const Eigen::Matrix3d &rotation) {
    Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
    ret.block<3, 1>(0, 3) = translation;
    ret.block<3, 3>(0, 0) = rotation;
    return ret;
  }

  static Eigen::Matrix4d Matrix4doubleToDouble(const Eigen::Matrix4f &matrix) {
    Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
    ret << matrix(0), matrix(4), matrix(8), matrix(12), matrix(1), matrix(5),
        matrix(9), matrix(13), matrix(2), matrix(6), matrix(10), matrix(14),
        matrix(3), matrix(7), matrix(11), matrix(15);
    return ret;
  }

  static Eigen::Matrix4d GetDeltaT(const vector<double> var) {
    auto deltaR = Eigen::Matrix3d(
        Eigen::AngleAxisd(deg2rad(var[2]), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(deg2rad(var[1]), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(deg2rad(var[0]), Eigen::Vector3d::UnitX()));
    Eigen::Matrix4d deltaT = Eigen::Matrix4d::Identity();
    deltaT.block<3, 3>(0, 0) = deltaR;
    deltaT(0, 3) = var[3];
    deltaT(1, 3) = var[4];
    deltaT(2, 3) = var[5];
    return deltaT;
  }

  static Eigen::Vector3d Rotation2Eul(Eigen::Matrix3d rot) {
    double sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));

    bool singular = sy < 1e-6; // If

    double x, y, z;
    if (!singular) {
      x = atan2(rot(2, 1), rot(2, 2));
      y = atan2(-rot(2, 0), sy);
      z = atan2(rot(1, 0), rot(0, 0));
    } else {
      x = atan2(-rot(1, 2), rot(1, 1));
      y = atan2(-rot(2, 0), sy);
      z = 0;
    }

    Eigen::Vector3d eul;
    eul(0) = z * 180 / M_PI;
    eul(1) = y * 180 / M_PI;
    eul(2) = x * 180 / M_PI;
    return eul;
  }

  static Eigen::Matrix3d
  eulerAnglesToRotationMatrix(Vec3f &theta) // theta为角度制
  {

    //转化为弧度制
    double th0 = M_PI * theta[0] / 180;
    double th1 = M_PI * theta[1] / 180;
    double th2 = M_PI * theta[2] / 180;
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3, 3) << 1, 0, 0, 0, cos(th0), -sin(th0), 0,
               sin(th0), cos(th0));

    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3, 3) << cos(th1), 0, sin(th1), 0, 1, 0, -sin(th1),
               0, cos(th1));

    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3, 3) << cos(th2), -sin(th2), 0, sin(th2), cos(th2),
               0, 0, 0, 1);

    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;
    Eigen::Matrix3d R_;
    cv2eigen(R, R_);
    return R_;
  }

  static Eigen::Matrix4d R_T2RT(Mat R, Mat T) {
    Mat RT;
    Mat_<double> R1 =
        (cv::Mat_<double>(4, 3) << R.at<double>(0, 0), R.at<double>(0, 1),
         R.at<double>(0, 2), R.at<double>(1, 0), R.at<double>(1, 1),
         R.at<double>(1, 2), R.at<double>(2, 0), R.at<double>(2, 1),
         R.at<double>(2, 2), 0.0, 0.0, 0.0);

    cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) << T.at<double>(0, 0),
                           T.at<double>(1, 0), T.at<double>(2, 0), 1.0);

    cv::hconcat(R1, T1, RT);
    Eigen::Matrix4d RT_;
    cv2eigen(RT, RT_);
    return RT_;
  }
};

#endif // TRANSFORM_UTIL_H_