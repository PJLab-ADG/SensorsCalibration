#ifndef RPZ_CALIB_RPZ_CALIB_H_
#define RPZ_CALIB_RPZ_CALIB_H_

#include <iostream>
#include <string>
#include <vector>
#include <dirent.h>
#include <chrono>

#include "GroundExtractor.h"

class RPCalib{
    public:
        RPCalib(std::string output_dir);
        ~RPCalib();

        bool LoadData(std::string dataset_folder, const std::vector<Eigen::Matrix4d> &lidar_pose, int start_frame, int end_frame);
        bool Calibrate(Eigen::Matrix4d & extrinsic);
        bool GroundPlaneExtraction(const PointCloudPtr &in_cloud, PointCloudPtr g_cloud, PointCloudPtr ng_cloud, PlaneParam &plane);

    private:
        std::string lidar_path_;
        std::string output_dir_;
        std::vector<std::string> lidar_files_;
        std::unique_ptr<GroundExtractor> ground_extractor_;
        std::vector<double> lidar_pose_y_;
        int file_num_;

        // param
        const double filter_master_max_range_ = 100.0;
        const double master_normal_check_ = 0.8;
        const double filter_min_range_ = 0.5;
        const int down_sample_ = 5;
};

#endif  //  RPZ_CALIB_RPZ_CALIB_H_