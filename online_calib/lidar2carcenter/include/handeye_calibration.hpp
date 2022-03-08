#pragma once 

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace autoCalib {
namespace calibration {

class HandEyeCalibration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	HandEyeCalibration();
    // in enu coordinate
    bool calibrate(const Eigen::Matrix3d &lidar_rot, 
                   const Eigen::Matrix3d &car_rot,   
                   Eigen::Matrix3d &calib_r_result);
    bool calibrate(const Eigen::Matrix3d &lidar_rot, const Eigen::Vector3d &lidar_pos,
                    const Eigen::Matrix3d &car_rot, const Eigen::Vector3d & car_pos,
                    const Eigen::Matrix3d &calib_r_result, Eigen::Vector3d &calib_tic_result);
private:
    const int window_size = 10;

    int frame_count;
    int frame_countT;
    Eigen::Vector3d tic;
    std::vector< Eigen::Matrix3d > Rlidar;
    std::vector< Eigen::Vector3d > Plidar;
    std::vector< Eigen::Matrix3d > Rcar;
    std::vector< Eigen::Vector3d > Pcar;
    std::vector< Eigen::Matrix3d > Rlidar_g;
    bool cflag;
public:
    Eigen::Matrix3d r_lidar2car;
};

} // namespace calibration
} // namespace autoCalib
