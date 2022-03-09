#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include "logging.hpp"
#include "DataReader.hpp"
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/utility.h"

Estimator estimator;
std::mutex m_buf;

int main(int argc, char **argv)
{

    if(argc != 6)
    {
        printf("please intput: ./main yaml_ car_config_path raw_imu_file "
               "video_file video_timestamp_file output_path \n"
               "for example: ./bin/run_camera2imu "
               "~/data/2021_01_05_14_43_28/config/vehicle/CN-012/ "
               "~/data/2021_01_05_14_43_28/parsed/novatel_rawimu.csv "
               "~/data/2021_01_05_14_43_28/center_camera.avi "
               "~/data/2021_01_05_14_43_28/center_camera.txt ./outputs/\n");
        return 1;
    }

     // string config_file = argv[1];
    string car_config_path = argv[1];
    string raw_imu_file = argv[2];
    string video_file = argv[3];
    string video_timestamp_file = argv[4];
    string output_dir = argv[5];
    // printf("config_file: %s\n", argv[1]);
    printf("car_config_path: %s\n", argv[1]);
    
    // creat output dir
    if (output_dir.rfind('/') != output_dir.size() - 1) {
        output_dir = output_dir + "/";
    }
    if (opendir(output_dir.c_str()) == nullptr) {
        char command[1024];
        snprintf(command, sizeof(command), "mkdir -p %s", output_dir.c_str());
        if (system(command)) {
            printf("Create dir: %s\n", output_dir.c_str());
        }
    }

    readCarParameters(car_config_path);
    estimator.setParameter();

    std::vector<double> imu_t_vec;
    std::vector<Eigen::Vector3d> linearAcceleration_vec;
    std::vector<Eigen::Vector3d> angularVelocity_vec;
    std::vector<double> camera_t_vec;

    // load file
    readIMU(raw_imu_file, imu_t_vec, linearAcceleration_vec, angularVelocity_vec);
    readCamera(video_timestamp_file, camera_t_vec);

    size_t i = 0, j = 0;
    cv::VideoCapture cap(video_file);
    while(i < imu_t_vec.size() && j < camera_t_vec.size()) {
        if (ESTIMATE_EXTRINSIC == 0&& ESTIMATE_EXTRINSICT == 0) break;
        // cout<<estimator.frame_count<<"------"<<i<<endl;
        double imu_t = imu_t_vec[i]+estimator.td;
        double cam_t = camera_t_vec[j];
        if (cam_t > imu_t) {
            estimator.inputIMU(imu_t, linearAcceleration_vec[i], angularVelocity_vec[i]);
            ++i;
        }
        else {
            cv::Mat img;
            cap.read(img);
            cv::cvtColor(img, img, CV_BGR2GRAY);
            // estimator.inputImage(cam_t, img_vec[j]);
            estimator.inputImage(cam_t, img);
            ++j;
            if (j == camera_t_vec.size()) break;            
        }
    }

    std::cout << "\nFinish!\n";
    std::cout << "extrinsic camera to imu:\n" << estimator.ric[0] 
              << "\nt:\n" << estimator.tic[0](0) << "\t" 
              << estimator.tic[0](1) << "\t" << estimator.tic[0](2)
              << std::endl;
    
    Eigen::Matrix4d camera2imu_ex;
    camera2imu_ex = Eigen::Matrix4d::Identity();
    camera2imu_ex.block<3,3>(0, 0) = estimator.ric[0];
    camera2imu_ex.block<3,1>(0, 3) = estimator.tic[0];
    Eigen::Vector3d ypr_result = Utility::R2ypr(estimator.ric[0]);
    Eigen::Vector3d ypr_gt = Utility::R2ypr(estimator.estimated_gt.block<3,3>(0,0));
    Eigen::Vector3d t_result = estimator.tic[0];
    // Eigen::Vector3d t_gt = estimator.estimated_gt.block<3,1>(0,3);

    std::string output_file = output_dir + "camera2imu_result.txt";
    std::ofstream out_file(output_file);
    if (!out_file.is_open()) {
        LOGE("open output file %s failed.\n", output_file_);
        return 0;
    } 
    out_file << "camera to imu extrinsic:" << std::endl;
    out_file << camera2imu_ex << std::endl;
    out_file << "YPR:" << std::endl;
    out_file << ypr_result(0) << " " << ypr_result(1) << " "
             << ypr_result(2) << std::endl;
    out_file << "Position:" << std::endl;
    out_file << t_result(0) << " " << t_result(1) << " "
             << t_result(2) << std::endl;
    
    out_file << "\nground truth:" << std::endl;
    out_file << estimator.estimated_gt << std::endl;
    out_file << "YPR:" << std::endl;
    out_file << ypr_gt(0) << " " << ypr_gt(1) << " "
             << ypr_gt(2) << std::endl;
    // out_file << t_gt(0) << " " << t_gt(1) << " "
    //          << t_gt(2) << std::endl;

    out_file.close();
    std::cout << "Save Result to " << output_file << std::endl;
    cv::destroyAllWindows();
    exit(1);
    return 0;
}
