#include <iostream>
#include <chrono>
#include <fstream>

#include "yawCalib.h"
#include "util.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

YawCalib::YawCalib(const std::string output_dir) 
{
    output_dir_ = output_dir;
    // plt::backend("Agg"); // plt run in background
}

bool YawCalib::LoadData(const std::vector<Eigen::Matrix4d> &lidar_pose){
    pose_num_ = lidar_pose.size();
    for (int i = 0; i < pose_num_; i++)
    {
        Eigen::Matrix4d T = lidar_pose[i];
        lidar_pose_x_.push_back(T(0, 3));
        lidar_pose_y_.push_back(T(1, 3));
        double yaw = Util::GetYaw(T);
        if (lidar_pose_yaw_.size()!=0){
            if(yaw - lidar_pose_yaw_.back() > M_PI) {
                yaw -= 2 * M_PI;
            }
            else if(yaw - lidar_pose_yaw_.back() < -M_PI) {
                yaw += 2 * M_PI;
            }
        }
        lidar_pose_yaw_.push_back(yaw);
    }

    if(save_trajectory_xy){
        plt::plot(lidar_pose_x_, lidar_pose_y_);
        plt::savefig(output_dir_ + "trajectory.png");
        // plt::show();
        plt::close();
    }
    return true;
}

bool YawCalib::Calibrate(){
    // bspline pose_x and pose_y
    std::cout << "---------------------------------------------------" << std::endl;
    std::cout << "start calibrating lidar yaw" << std::endl;

    DataTable sample_x, sample_y;
    DenseVector t(1);
    for(int i = 0; i < pose_num_; i += time_gap_)
    {
        t(0) = i;
        sample_x.addSample(t, lidar_pose_x_[i]);
        sample_y.addSample(t, lidar_pose_y_[i]);
    }

    // get yaw from dx and dy
    std::vector<DataTable> samples_yaw;
    GetYawSegs(sample_x, sample_y, samples_yaw);

    std::vector<double> offset_yaws, weights;
    // calibrate each segment yaw

    for (unsigned int i = 0; i < samples_yaw.size(); i++)
    {
        DataTable sample_yaw = samples_yaw[i];
        plt::plot(sample_yaw.getTableX()[0], sample_yaw.getVectorY(), "k--", {{"label", "trajectory yaw"}});
        double yaw;
        if(CalibrateSingle(sample_yaw, yaw)){
            offset_yaws.push_back(yaw);
            weights.push_back(sample_yaw.getNumSamples());
            std::cout << "Segment" << i << ": yaw =" << rad2deg(yaw) << " degree  weight = " << sample_yaw.getNumSamples() << std::endl;
        }
    }

    std::vector<double> times;
    for (unsigned int i = 0; i < lidar_pose_yaw_.size();i++)
    {
        times.push_back(i);
    }
    
    plt::plot(times, lidar_pose_yaw_, "k-", {{"label", "lidar pose yaw"}});
    plt::legend();
    plt::savefig(output_dir_ + "compared_yaw.png");
    plt::close();
    if (offset_yaws.size() == 0)
        return false;
    final_yaw_ = Util::WeightMean(offset_yaws, weights);
    // std::cout << "Average yaw = " << rad2deg(final_yaw_) << " degree" << std::endl;

    return true;
}

bool YawCalib::GetYawSegs(const DataTable &sample_x, const DataTable &sample_y, std::vector<DataTable> &samples_yaw){
    
    BSpline bspline_x = BSpline::Builder(sample_x).degree(bspine_degree_).smoothing(BSpline::Smoothing::PSPLINE).alpha(0.03).build();
    BSpline bspline_y = BSpline::Builder(sample_y).degree(bspine_degree_).smoothing(BSpline::Smoothing::PSPLINE).alpha(0.03).build();

    int discarded_nums = int(pose_num_ * 0.05);
    DataTable tmp_yaw;
    int last_t = 0;
    double last_yaw = 0;

    for (int i = discarded_nums; i < pose_num_ - discarded_nums; i += time_gap_)
    {
        DenseVector t(1);
        t(0) = i;
        double dx = bspline_x.evalJacobian(t)(0, 0);
        double dy = bspline_y.evalJacobian(t)(0, 0);
        double ddx = bspline_x.evalHessian(t)(0, 0);
        double ddy = bspline_y.evalHessian(t)(0, 0);
        double cur_x = fabs(ddx) / pow(1 + dx * dx, 1.5);
        double cur_y = fabs(ddy) / pow(1 + dy * dy, 1.5);

        // delete unmoving points
        if (dx * dx + dy * dy < 1e-3)
            continue;
        // detele points with large curvature
        if (cur_x > 0.015 || cur_y > 0.015)
            continue;

        double yaw = atan2(dy, dx);
        t(0) = i + bspine_degree_;
        if(tmp_yaw.getNumSamples() != 0 && i - last_t > time_gap_ * 5)
        {
            if(tmp_yaw.getNumSamples() > 20) 
            {
                samples_yaw.push_back(tmp_yaw);
            }
            tmp_yaw = DataTable();
        }
        if(tmp_yaw.getNumSamples() != 0 && fabs(yaw - last_yaw) > M_PI){
            if(yaw - last_yaw > M_PI) {
                yaw -= 2 * M_PI;
            }
            else if(yaw - last_yaw < -M_PI) {
                yaw += 2 * M_PI;
            }
        }
        tmp_yaw.addSample(t, yaw);
        last_yaw = yaw;
        last_t = i;
    }
    if(tmp_yaw.getNumSamples() > 20) 
    {
        samples_yaw.push_back(tmp_yaw);
    }


    return true;
}

bool YawCalib::CalibrateSingle(const DataTable & sample_yaw, double & estimate_yaw)
{
    BSpline bspline_yaw = BSpline::Builder(sample_yaw).degree(bspine_degree_).smoothing(BSpline::Smoothing::PSPLINE).alpha(0.06).build();
    std::vector<double> timestamp = sample_yaw.getTableX()[0];
    std::vector<double> offset_yaws;
    int discarded_nums = int(timestamp.size() * 0.05);
    for (unsigned int i = discarded_nums; i < timestamp.size() - discarded_nums; i++)
    {
        DenseVector t(1);
        t(0) = timestamp[i];
        double yaw = bspline_yaw.eval(t);
        // double dyaw = bspline_yaw.evalJacobian(t)(0, 0);
        // double ddyaw = bspline_yaw.evalHessian(t)(0, 0);
        double offset_yaw = lidar_pose_yaw_[timestamp[i]] - yaw;
        if (offset_yaw > M_PI)
            offset_yaw -= 2 * M_PI;
        else if(offset_yaw < - M_PI)
            offset_yaw += 2 * M_PI;
        offset_yaws.push_back(offset_yaw);
    }

    std::vector<double> new_offset_yaws;
    Util::DeleteOutliers(offset_yaws, new_offset_yaws, 3);
    estimate_yaw = Util::Mean(new_offset_yaws);
    return true;
}

double YawCalib::GetFinalYaw()
{
    return final_yaw_;
}
