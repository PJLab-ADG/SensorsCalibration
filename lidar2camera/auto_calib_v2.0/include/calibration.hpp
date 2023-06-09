/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#pragma once

#include "logging.hpp"
#include "utility.hpp"
#include "dataloader.hpp"

struct PointXYZINS
{
    PCL_ADD_POINT4D;
    // PCL_ADD_NORMAL4D;
    union{
        struct 
        {
            float intensity;
            float curvature;
            int segment; // store segmentation result
        };
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZINS,
    (float, intensity, intensity)
    (float, curvature, curvature)
    (int, segment, segment)
)

class Calibrator
{
public:
    Calibrator(
        const std::string mask_dir,
        const std::string lidar_file,
        const std::string calib_file,
        const std::string img_file,
        const std::string error_file);
    void Calibrate();
    void ProcessPointcloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_origin);
    bool CalScore(Eigen::Matrix4f T, float& score, bool is_coarse);
    void VisualProjection(Eigen::Matrix4f T, std::string img_file, std::string save_name);
    void VisualProjectionSegment(Eigen::Matrix4f T, std::string img_file, std::string save_name);
    void Segment_pc(const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_origin,
                    pcl::PointCloud<pcl::Normal>::Ptr normals,
                    std::vector<pcl::PointIndices>& seg_indices);
    void BruteForceSearch(int rpy_range, float rpy_resolution,int xyz_range, float xyz_resolution, bool is_coarse);
    void RandomSearch(int search_count, float xyz_range, float rpy_range, bool is_coarse);
    bool ProjectOnImage(const Eigen::Vector4f &vec, const Eigen::Matrix4f &T, int &x, int &y, int margin);
    void PrintCurrentError();
    Eigen::Matrix4f GetFinalTransformation();

private:
    Eigen::Matrix4f init_extrinsic_;
    Eigen::Matrix4f extrinsic_ = Eigen::Matrix4f::Identity();
    Eigen::MatrixXf intrinsic_;
    pcl::PointCloud<PointXYZINS>::Ptr pc_;
    cv::Mat masks_;
    std::string img_file_;
    std::vector<int> mask_point_num_, seg_point_num_;
    std::vector<double> dist_;
    float max_score_ = 0;
    int IMG_H, IMG_W, N_MASK, N_SEG;
    float POINT_PER_PIXEL = 0.07;
    float w_n = 1, w_i = 1, w_s = 1;
    float curvature_max_ = 0;
};