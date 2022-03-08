/*
 * Copyright (C) 2020-2020 by SenseTime Group Limited. All rights reserved.
 * ZhaoMing<zhaoming@sensetime.com>
 */
#ifndef LIDAR2CAR_REGISTRATION_REGISTRATION_HPP_
#define LIDAR2CAR_REGISTRATION_REGISTRATION_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "common/common.hpp"
#include "common/logging.hpp"

namespace autoCalib {
namespace calibration {

struct PointCloudFeatures {
PointCloudFeatures() {
    corner_sharp_pts.reset(new PointCloudLidar);
    corner_less_sharp_pts.reset(new PointCloudLidar);
    surf_flat_pts.reset(new PointCloudLidar);
    surf_less_flat_pts.reset(new PointCloudLidar);
    remove_pts.reset(new PointCloudLidar);
}

PointCloudLidar::Ptr corner_sharp_pts;
PointCloudLidar::Ptr corner_less_sharp_pts;
PointCloudLidar::Ptr surf_flat_pts;
PointCloudLidar::Ptr surf_less_flat_pts;
PointCloudLidar::Ptr remove_pts;
};

// lidar features
static float cloudCurvature[400000];
static int cloudSortInd[400000];
static int cloudNeighborPicked[400000];
static int cloudLabel[400000];

static bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

static double para_q_[4] = {0, 0, 0, 1};
static double para_t_[3] = {0, 0, 0};
static Eigen::Map<Eigen::Quaterniond> q_last_curr_(para_q_);
static Eigen::Map<Eigen::Vector3d> t_last_curr_(para_t_);

class Registrator {
 public:
    Registrator();
    // ~Registrator();

    void inputFrame(PointCloud::Ptr cloud, Eigen::Matrix4d& extrinsic);

    Eigen::Matrix4d GetFinalTransformation();

private:
    void getFeatures(PointCloud::Ptr cloud, 
                     PointCloudFeatures& pcd_features);

    void removeClosedPointCloud(PointCloud::Ptr cloud_in,
                                PointCloud::Ptr cloud_out, 
                                float thres);
    
    void TransformToStart(PointXYZI const *const pi, 
                          PointXYZI *const po);
    
    void TransformToEnd(PointXYZI const *const pi, 
                        PointXYZI *const po);
    
 private:
    //  paramters
    const float minium_range_ = 0.1;
    // lidar parameters
    const float scan_period_ = 0.1;
    const float n_scans_ = 64;

    const float distance_sq_threshold_ = 25;
    const float nearby_scan_ = 2.5;
    const bool distortion_ = false;

    // skip first frame
    bool systemInited_;

    int corner_correspondence_;
    int plane_correspondence_;

    // prev frame
    PointCloudLidar::Ptr laserCloudCornerLast_;
    PointCloudLidar::Ptr laserCloudSurfLast_;
    PointCloudLidar::Ptr laserCloudFullRes_;

    pcl::KdTreeFLANN<PointXYZI>::Ptr kdtreeCornerLast_;
    pcl::KdTreeFLANN<PointXYZI>::Ptr kdtreeSurfLast_;
    
    // transformation
    // double para_q_[4];
    // double para_t_[3];
    Eigen::Quaterniond q_w_curr_; 
    Eigen::Vector3d t_w_curr_; 
    // Eigen::Map<Eigen::Quaterniond> q_last_curr_;
    // Eigen::Map<Eigen::Vector3d> t_last_curr_;

    Eigen::Matrix4d final_transformation_;
};

}  // namespace calibration
}  // namespace autoCalib

#endif  // LIDAR2CAR_REGISTRATION_REGISTRATION_HPP_