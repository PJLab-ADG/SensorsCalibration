#ifndef RPZ_CALIB_GROUND_EXTRACTOR_H_
#define RPZ_CALIB_GROUND_EXTRACTOR_H_

#include <iostream>
#include <vector>
#include <algorithm>
#include <omp.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pc_util.h"

struct PlaneParam {
    PlaneParam() {}
    PlaneParam(const Eigen::Vector3d& n, double i) : normal(n), intercept(i) {}
    Eigen::Vector3d normal;
    double intercept;
};

class GroundExtractor {
 public:
    GroundExtractor() = default;
    ~GroundExtractor() = default;

    /* @brief: ground extraction using lowest point representation algorithm,
     * aim at the master lidar*/
    bool LPRFitting(const PointCloudPtr in_cloud,
                    PointCloudPtr g_cloud,
                    PointCloudPtr ng_cloud,
                    PlaneParam *plane);

    /* @brief: ground extraction using random ransac algorithm,
     * aim at the slaver lidar*/
    bool RandomRansacFitting(const PointCloudPtr in_cloud,
                             PointCloudPtr g_cloud,
                             PointCloudPtr ng_cloud,
                             PlaneParam * plane);

 private:
    size_t RandIndex(size_t range);

    bool CalArea(const PointType& p1,
                 const PointType& p2,
                 const PointType& p3,
                 double* area);

    bool FittingPlane(PointCloudPtr in_cloud, PlaneParam *plane);
    bool FittingPlaneMesh(const PointCloudPtr in_cloud, PlaneParam *plane);
    bool RandomSearchPlane(const PointCloudPtr in_cloud, PlaneParam &best_plane, int &max_inlier_points,
                           double n1_scope, double n2_scope, double n3_scope, double i_scope, int iteration_times);

 private:
    const int lpr_max_iters_ = 100;
    const double lpr_fit_dist_thre_ = 0.05;
    const double lpr_least_gpoints_rate_ = 0.2;
    const double lpr_least_gpoints_interval_ = 0.2;

    const int rr_iter_times_ = 5;
    const int rr_max_rand_iters_ = 500;
    const double rr_gpoints_rate_ = 0.4;
    const double rr_min_area_thre_ = 0.25;
    const double rr_fit_dist_thre_ = 0.1;

};


#endif  // RPZ_CALIB_GROUND_EXTRACTOR_H_