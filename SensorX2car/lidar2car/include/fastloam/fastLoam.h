#include <iostream>
#include <time.h>
#include <iterator>
#include <stdio.h>
#include <string>
#include <vector>
#include <dirent.h>
#include "Eigen/Core"
#include "Eigen/Dense"
#include <Eigen/Geometry>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "laserProcessingClass.h"
#include "odomEstimationClass.h"

bool RunFastLoam(const std::string dataset_folder, 
                 const std::string output_dir, 
                 std::vector<Eigen::Matrix4d> &lidar_pose, 
                 int start_frame, 
                 int end_frame);
