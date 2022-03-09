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

#include "initial/initial_ex_rotation.h"
#include "logging.hpp"




initialExtrinsic::initialExtrinsic(){
    frameR_count = 0;
    frameT_count = 0;
    frameRT_count = 0;
    cflag = true;
    ric = Matrix3d::Identity();
    tic = Vector3d::Zero();
    ideal_ric << 1, 0, 0,
                 0, 0, 1,
                 0, -1, 0;

    Rc.push_back(Matrix3d::Identity());
    Rc_g.push_back(Matrix3d::Identity());
    Rimu.push_back(ideal_ric.inverse() * ideal_ric);
    cov_thresh = 0.7;
}
bool initialExtrinsic::CalibrationTime(const double frame_time, vector<pair<Vector3d, Vector3d>> &corres,vector<Eigen::Vector3d>& angular_velocity_buf, double &tdtime){
    frameT_count++;
    if(angular_velocity_buf.size()>30||angular_velocity_buf.size()<10) return false;
    Eigen::Vector3d Rc_ypr = Utility::R2ypr(solveRelativeR(corres));
    Rc_ypr(0) = 0;
    // Rc.push_back(solveRelativeR(corres));
    Eigen::Vector3d wc_t =Vector3d(Rc_ypr(2)/frame_time,Rc_ypr(1)/frame_time,Rc_ypr(0)/frame_time);
    Wc_t.push_back(wc_t);
    Frame_time.push_back(frame_time);
    Wimu_t_buf.push_back(angular_velocity_buf); // TODO: not fixed length   1   26  51 
    if(frameT_count>210){
        Eigen::Vector3d wc_t_mean;
        wc_t_mean = std::accumulate(Wc_t.begin()+10,Wc_t.end()-1,Vector3d(0,0,0));
        wc_t_mean = wc_t_mean/(Wc_t.size()-11);
        for(auto it = Wimu_t_buf.begin()+9;it!=Wimu_t_buf.end()-2;it++){
                int sz = it->size();
                auto itnext = it+1; 
                Wimu.push_back(((*itnext)[sz-1]+(*it)[sz-1])/2);
            }
        Eigen::Vector3d wimu_mean = std::accumulate(Wimu.begin(),Wimu.end(),Vector3d(0,0,0));
        wimu_mean /= Wimu.size();
        double frame_time_mean = std::accumulate(Frame_time.begin(),Frame_time.end(),0.0);
        frame_time_mean /= Frame_time.size();
        double trmax=0;
        for(int i=-25;i<25;++i){
            if(i<=0){
                for(auto it = Wimu_t_buf.begin()+9;it!=Wimu_t_buf.end()-2;it++){
                    int sz = it->size();
                    auto itnext = it+1; 
                    Wimu_t.push_back(((*itnext)[sz-1+i]+(*it)[sz-1+i])/2);
                }
            }
            else{
                for(auto it = Wimu_t_buf.begin()+10;it!=Wimu_t_buf.end()-1;it++){
                    int sz = it->size();
                    auto itnext = it+1; 
                    Wimu_t.push_back(((*itnext)[i]+(*it)[i])/2);
                }
            }
            Eigen::Vector3d wimu_t_mean=accumulate(Wimu_t.begin(),Wimu_t.end(),Vector3d(0,0,0));
            wimu_t_mean/=(Wimu_t.size());
            Eigen::Matrix3d sigmaci(Eigen::Matrix3d::Zero()),sigmacc(Eigen::Matrix3d::Zero()),
                        sigmaii(Eigen::Matrix3d::Zero()),sigmaic(Eigen::Matrix3d::Zero());
            for(int j=0;j<Wimu_t.size();++j){
                sigmacc += (Wc_t[j]-wc_t_mean)*(Wc_t[j]-wc_t_mean).transpose();
                sigmaci += (Wc_t[j]-wc_t_mean)*(Wimu_t[j]-wimu_t_mean).transpose();
                sigmaii += (Wimu[j]-wimu_mean)*(Wimu_t[j]-wimu_t_mean).transpose();
                sigmaic += (Wimu[j]-wimu_mean)*(Wc_t[j]-wc_t_mean).transpose();
            }
            Eigen::Matrix3d target = sigmacc.inverse()*sigmaci*sigmaic*sigmaii.inverse();
            if(target.trace()>trmax){
                tdtime = i*frame_time_mean/25;
                trmax = target.trace();
            }
        }
        return true;
    }
    return false;

}

bool initialExtrinsic::CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result)
{
    frameR_count++;
    Eigen::Vector3d Rimu_ypr = Utility::R2ypr(delta_q_imu.toRotationMatrix());
    Eigen::Vector3d Rc_ypr = Utility::R2ypr(solveRelativeR(corres));
    Eigen::Vector3d Rc_ideal_ypr = Utility::R2ypr(ideal_ric.inverse() * delta_q_imu * ideal_ric);
    std::cout << "Rimu_ypr: " << Rimu_ypr(0) << ", " << Rimu_ypr(1)
              << ", " << Rimu_ypr(2) << std::endl;
    std::cout << "Rc_ypr: " << Rc_ypr(0) << ", " << Rc_ypr(1)
              << ", " << Rc_ypr(2) << std::endl;
    std::cout << "Rc_ideal_ypr: " << Rc_ideal_ypr(0) << ", " << Rc_ideal_ypr(1)
              << ", " << Rc_ideal_ypr(2) << std::endl;

    Rimu_ypr(1) = 0;
    Rc_ypr(0) = 0;
    // Rc.push_back(solveRelativeR(corres));
    Rc.push_back(Utility::ypr2R(Rc_ypr));
    Rimu.push_back(Utility::ypr2R(Rimu_ypr));
    // Rc_g.push_back(ric.inverse() * Utility::ypr2R(Rimu_ypr) * ric);

    Eigen::MatrixXf A = MatrixXf::Random(frameR_count * 4, 4);
    int sum_ok = 0;
    std::cout << "frame " << frameR_count << std::endl;
    for (int i = 1; i <= frameR_count; i++)
    {
        Quaterniond r1(Rc[i]);
        Eigen::Matrix3d Rcg_c = ric.inverse() * Rimu[i] * ric;
        // Quaterniond r2(Rc_g[i]);
        Quaterniond r2(Rcg_c);

        double angular_distance = 180 / M_PI * r1.angularDistance(r2);

        double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;
        ++sum_ok;
        Matrix4d L, R;

        double w = Quaterniond(Rc[i]).w();
        Vector3d q = Quaterniond(Rc[i]).vec();
        L.block<3, 3>(0, 0) = w * Matrix3d::Identity() + Utility::skewSymmetric(q);
        L.block<3, 1>(0, 3) = q;
        L.block<1, 3>(3, 0) = -q.transpose();
        L(3, 3) = w;

        Quaterniond R_ij(Rimu[i]);
        w = R_ij.w();
        q = R_ij.vec();
        R.block<3, 3>(0, 0) = w * Matrix3d::Identity() - Utility::skewSymmetric(q);
        R.block<3, 1>(0, 3) = q;
        R.block<1, 3>(3, 0) = -q.transpose();
        R(3, 3) = w;

        Eigen::Matrix4d mid = huber * (L - R);

        for (size_t a = (i - 1) * 4; a < (i - 1) * 4 + 4; ++a) {
            for (size_t b = 0; b < 4; ++b) {
                A(a, b) = mid(a-(i - 1) * 4, b);
            }
        }
        // A.block<4, 4>((i - 1) * 4, 0) = huber * (L - R);
    }
    JacobiSVD<Eigen::MatrixXf> svd(A, ComputeFullU | ComputeFullV);

    Matrix<float, 4, 1> xf = svd.matrixV().col(3);
    // Matrix<double, 4, 1> x = svd.matrixV().col(3);
    Matrix<double, 4, 1> x;
    x(0, 0) = xf(0, 0);
    x(1, 0) = xf(1, 0);
    x(2, 0) = xf(2, 0);
    x(3, 0) = xf(3, 0);

    Quaterniond estimated_R(x);
    ric = estimated_R.toRotationMatrix().inverse();
    // cout << svd.singularValues().transpose() << endl;
    Vector3f ric_cov;
    ric_cov = svd.singularValues().tail<3>();
    std::cout << "cov: " << ric_cov(1) <<std::endl;
    if (frameR_count >= WINDOW_SIZE && ric_cov(1) > cov_thresh)
    {
        calib_ric_result = ric;
        return true;
    }
    else {
        calib_ric_result = ric;
        return false;
    }
}

bool initialExtrinsic::CalibrationExPosition(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu,Vector3d delta_p_imu,  Matrix3d &calib_ric_result, Vector3d &calib_tic_result)
{
    frameRT_count++;
    Eigen::Vector3d Rimu_ypr = Utility::R2ypr(delta_q_imu.toRotationMatrix());
    Eigen::Matrix4d RelativeT = solveRelativeT(corres);
    Eigen::Vector3d Rc_ypr = Utility::R2ypr(RelativeT.block<3,3>(0,0));
    Eigen::Vector3d Rc_ideal_ypr = Utility::R2ypr(ideal_ric.inverse() * delta_q_imu * ideal_ric);
    std::cout << "Rimu_ypr: " << Rimu_ypr(0) << ", " << Rimu_ypr(1)
              << ", " << Rimu_ypr(2) << std::endl;
    std::cout << "Rc_ypr: " << Rc_ypr(0) << ", " << Rc_ypr(1)
              << ", " << Rc_ypr(2) << std::endl;
    std::cout << "Rc_ideal_ypr: " << Rc_ideal_ypr(0) << ", " << Rc_ideal_ypr(1)
              << ", " << Rc_ideal_ypr(2) << std::endl;
    
    Rimu_ypr(1) = 0;
    Rc_ypr(0) = 0;
    // Rc.push_back(solveRelativeR(corres));
    if(cflag) {
        Rc.clear(); Rimu.clear(); tc.clear(); timu.clear(); cflag = false;
        Eigen::Matrix3d ideal_ric;
        ideal_ric << 1, 0, 0,0, 0, 1,0, -1, 0;
        Rc.push_back(Matrix3d::Identity());
        Rimu.push_back(ideal_ric.inverse() * ideal_ric);
        tc.push_back(Vector3d::Zero());timu.push_back(Vector3d::Zero());
        }
    tc.push_back(RelativeT.block<3,1>(0,3));
    timu.push_back(delta_p_imu);
    Rc.push_back(Utility::ypr2R(Rc_ypr));
    Rimu.push_back(Utility::ypr2R(Rimu_ypr));
    // Rc_g.push_back(ric.inverse() * Utility::ypr2R(Rimu_ypr) * ric);

    Eigen::MatrixXd A = MatrixXd::Random(frameRT_count * 3, 3);
    Eigen::VectorXd B = VectorXd::Random(frameRT_count * 3, 1);

    std::cout << "frame " << frameRT_count << std::endl;
    for (int i = 0; i <= frameRT_count; i++)
    {
        Eigen::Matrix3d A1 = Rc[i] - Matrix3d::Identity();  
        Eigen::Vector3d bstep = A1*tic;
        Eigen::Vector3d b1 = calib_ric_result*timu[i] - tc[i];
        double distance = (bstep - b1).norm();
        double huber = distance > 10 ? 10/distance: 1;
        A1 *= huber;
        b1 *= huber;
        for (size_t a = (i - 1) * 3; a < (i - 1) * 3 + 3; ++a) {
            B(a, 0) = b1(a-(i - 1) * 3, 0);
            for (size_t b = 0; b < 3; ++b) {
                A(a, b) = A1(a-(i - 1) * 3, b);
            }
        }
        // A.block<4, 4>((i - 1) * 4, 0) = huber * (L - R);
    }
    // JacobiSVD<Eigen::MatrixXf> svd(A, ComputeFullU | ComputeFullV);

    Matrix<double, 3, 1> xf = A.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(B);
    // Matrix<double, 4, 1> x = svd.matrixV().col(3);

    // cout << svd.singularValues().transpose() << endl;
    // Vector3f tic_cov;
    // tic_cov = svd.singularValues().tail<3>();
    // std::cout << "cov: " << tic_cov(1) <<std::endl;
    if (frameRT_count >= WINDOW_SIZE*10&& (tic-xf).norm()<1e-2)
    {
        tic(0, 0) = xf(0, 0);
        tic(1, 0) = xf(1, 0);
        tic(2, 0) = xf(2, 0);
        calib_tic_result = tic;
        return true;
    }
    else {
        tic(0, 0) = xf(0, 0);
        tic(1, 0) = xf(1, 0);
        tic(2, 0) = xf(2, 0);
        calib_tic_result = tic;
        return false;
    }
}

Matrix3d initialExtrinsic::solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres)
{
    if (corres.size() >= 9)
    {
        vector<cv::Point2f> ll, rr;
        for (int i = 0; i < int(corres.size()); i++)
        {
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
            rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
        }
        cv::Mat E = cv::findFundamentalMat(ll, rr);
        cv::Mat_<double> R1, R2, t1, t2;
        decomposeE(E, R1, R2, t1, t2);

        if (determinant(R1) + 1.0 < 1e-09)
        {
            E = -E;
            decomposeE(E, R1, R2, t1, t2);
        }
        double ratio1 = max(testTriangulation(ll, rr, R1, t1), testTriangulation(ll, rr, R1, t2));
        double ratio2 = max(testTriangulation(ll, rr, R2, t1), testTriangulation(ll, rr, R2, t2));
        cv::Mat_<double> ans_R_cv = ratio1 > ratio2 ? R1 : R2;

        Matrix3d ans_R_eigen;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ans_R_eigen(j, i) = ans_R_cv(i, j);
        return ans_R_eigen;
    }
    return Matrix3d::Identity();
}
Matrix4d initialExtrinsic::solveRelativeT(const vector<pair<Vector3d, Vector3d>> &corres)
{
    if (corres.size() >= 9)
    {
        vector<cv::Point2f> ll, rr;
        for (int i = 0; i < int(corres.size()); i++)
        {
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
            rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
        }
        cv::Mat E = cv::findFundamentalMat(ll, rr);
        cv::Mat_<double> R1, R2, t1, t2;
        decomposeE(E, R1, R2, t1, t2);

        if (determinant(R1) + 1.0 < 1e-09)
        {
            E = -E;
            decomposeE(E, R1, R2, t1, t2);
        }
        double ratio1 = max(testTriangulation(ll, rr, R1, t1), testTriangulation(ll, rr, R1, t2));
        double ratio2 = max(testTriangulation(ll, rr, R2, t1), testTriangulation(ll, rr, R2, t2));
        cv::Mat_<double> ans_R_cv = ratio1 > ratio2 ? R1 : R2;
        cv::Mat_<double> ans_t_cv = ratio1 > ratio2 ? t1 : t2;

        Matrix3d ans_R_eigen;
        Vector3d ans_t_eigen;
        Matrix4d ans;
        for (int i = 0; i < 3; i++){
            ans_t_eigen(i, 0) = ans_t_cv(0, i);
            for (int j = 0; j < 3; j++)
                ans_R_eigen(j, i) = ans_R_cv(i, j);
        }
        ans.block<3,3>(0,0) = ans_R_eigen;
        ans.block<3,1>(0,3) = ans_t_eigen;
        ans.block<1,4>(3,0) = Vector4d(0,0,0,1);
        return ans;
    }
    return Matrix4d::Identity();
}
double initialExtrinsic::testTriangulation(const vector<cv::Point2f> &l,
                                          const vector<cv::Point2f> &r,
                                          cv::Mat_<double> R, cv::Mat_<double> t)
{
    cv::Mat pointcloud;
    cv::Matx34f P = cv::Matx34f(1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0);
    cv::Matx34f P1 = cv::Matx34f(R(0, 0), R(0, 1), R(0, 2), t(0),
                                 R(1, 0), R(1, 1), R(1, 2), t(1),
                                 R(2, 0), R(2, 1), R(2, 2), t(2));
    cv::triangulatePoints(P, P1, l, r, pointcloud);
    int front_count = 0;
    for (int i = 0; i < pointcloud.cols; i++)
    {
        double normal_factor = pointcloud.col(i).at<float>(3);

        cv::Mat_<double> p_3d_l = cv::Mat(P) * (pointcloud.col(i) / normal_factor);
        cv::Mat_<double> p_3d_r = cv::Mat(P1) * (pointcloud.col(i) / normal_factor);
        if (p_3d_l(2) > 0 && p_3d_r(2) > 0)
            front_count++;
    }
    LOGI("MotionEstimator: %f", 1.0 * front_count / pointcloud.cols);
    return 1.0 * front_count / pointcloud.cols;
}

void initialExtrinsic::decomposeE(cv::Mat E,
                                 cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                                 cv::Mat_<double> &t1, cv::Mat_<double> &t2)
{
    cv::SVD svd(E, cv::SVD::MODIFY_A);
    cv::Matx33d W(0, -1, 0,
                  1, 0, 0,
                  0, 0, 1);
    cv::Matx33d Wt(0, 1, 0,
                   -1, 0, 0,
                   0, 0, 1);
    R1 = svd.u * cv::Mat(W) * svd.vt;
    R2 = svd.u * cv::Mat(Wt) * svd.vt;
    t1 = svd.u.col(2);
    t2 = -svd.u.col(2);
}
