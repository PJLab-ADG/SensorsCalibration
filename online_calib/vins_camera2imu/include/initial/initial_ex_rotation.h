/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once 

#include <vector>
#include "../estimator/parameters.h"
using namespace std;

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>
using namespace Eigen;
// #include <ros/console.h>

/* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
class initialExtrinsic
{
public:
	initialExtrinsic();
    bool CalibrationTime(const double frame_time, vector<pair<Vector3d, Vector3d>> &corres,vector<Eigen::Vector3d>& angular_velocity_buf, double &tdtime);
    bool CalibrationExPosition(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Vector3d delta_p_imu, Matrix3d &calib_ric_result,Vector3d &calib_tic_result);
    bool CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result);
    void setCovThresh(const double &cov_val) {cov_thresh = cov_val;}

private:
	Matrix3d solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres);

    Matrix4d solveRelativeT(const vector<pair<Vector3d, Vector3d>> &corres);

    double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
    int frameR_count;
    int frameT_count;
    int frameRT_count;
    double cov_thresh;

    vector< Vector3d > Wc_t;
    vector< Vector3d > Wimu;
    vector< Vector3d > Wimu_t;
    vector<vector<Vector3d> > Wimu_t_buf;
    vector< double > Frame_time;
    vector< Matrix3d > Rc;
    vector< Matrix3d > Rimu;
    vector< Matrix3d > Rc_g;
    vector< Vector3d > tc;
    vector< Vector3d > timu;

public:
    Matrix3d ric;
    Vector3d tic;
    Matrix3d ideal_ric;
    bool cflag;
};


