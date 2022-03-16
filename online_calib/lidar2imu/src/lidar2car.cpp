#include "lidar2car.hpp"

#include <stdio.h>
#include <dirent.h>
#include "utils/data_reader_util.hpp"

namespace autoCalib {
namespace calibration {

LidarToCar::LidarToCar() 
{
    calibrator_.reset(new LidarCarCalibrator);
}

void LidarToCar::calib(const std::string &lidar_dir,
                       const std::string &novatel_pose,
                       const std::string &config_dir,
                       const std::string &output_dir,
                       const LidarCalibParam &param)
{
    // get gnss2carcenter extrinsic
    Eigen::Matrix4d gnss2carcenter;
    std::string gnss2carcenter_path = config_dir + 
        "gnss/gnss-to-car_center-extrinsic.json";
    DataReaderUtil::getExtrinsic(gnss2carcenter_path, gnss2carcenter);

    // load lidar file
    std::vector<std::string> lidar_files;
    DIR *dir;
    struct dirent *ptr;
    int success_num = 0;
    if ((dir = opendir(lidar_dir.c_str())) == NULL) {
        LOGE("Open dir error !");
        exit(1);
    }
    while ((ptr = readdir(dir)) != NULL) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
            lidar_files.emplace_back(ptr->d_name);
        ptr++;
    }
    if (lidar_files.size() == 0) {
        LOGE("no file under parsed lidar dir.");
        exit(1);
    }
    closedir(dir);
    std::sort(lidar_files.begin(), lidar_files.end());

    // get start timestamp
    double start_timestamp;
    std::stringstream st(lidar_files[0]);
    std::string sts;
    std::getline(st, sts, '.'); 
    DataReaderUtil::timeString2timecount_2(sts, start_timestamp);

    // load novatel pose
    std::vector<double> timestamps;
    std::vector<Eigen::Matrix4d> car_poses;
    LoadCarPose(novatel_pose, gnss2carcenter, start_timestamp + param.start_sec,
                start_timestamp + param.start_sec + param.max_sec, timestamps, car_poses);
    calibrator_->setCarPose(timestamps, car_poses);

    // input lidar frames to calibrator
    for (size_t i = 0; i < lidar_files.size(); i+=param.frame_gap) {
        std::stringstream lts(lidar_files[i]);
        std::string timesting;
        std::getline(lts, timesting, '.');
        double l_time;
        DataReaderUtil::timeString2timecount_2(timesting, l_time);
        if (l_time - start_timestamp < param.start_sec) continue;
        if (l_time - start_timestamp - param.start_sec > param.max_sec) break;
        // load point cloud
        std::string pcd_path = lidar_dir + lidar_files[i];
        PointCloud::Ptr lidar_pcd(new PointCloud);
        LoadPointCloud(pcd_path, lidar_pcd);

        if(calibrator_->calib_t) calibrator_->inputLidarT(l_time, lidar_pcd);
        else calibrator_->inputLidar(l_time, lidar_pcd);
    }
    std::cout << "\nFinal lidar-to-car extrinsic:\n";
    Eigen::Matrix4d lidar2car;
    calibrator_->getTrandformation(lidar2car);
    std::cout << lidar2car << std::endl;
}


void LidarToCar::LoadCarPose(const std::string& novatel_pos_path,
                             const Eigen::Matrix4d& gnss2car,
                             const double& start_time,
                             const double& end_time,
                             std::vector<double>& timestamps,
                             std::vector<Eigen::Matrix4d>& car_poses) 
{
    std::ifstream infile(novatel_pos_path);
    std::string line;
    if (!infile) {
        LOGE("Open enu pose file failed.\n");
        exit(1);
    }
    Eigen::Matrix4d car2gnss = gnss2car.inverse();
    while (getline(infile, line)) {
        std::stringstream ss(line);
        std::vector<std::string> elements;
        std::string elem;
        while (getline(ss, elem, ' ')) {
            elements.emplace_back(elem);
        }
        if (elements.size() != 13) {
            LOGW("num of line elements error! skip this line.\n");
            continue;
        }
        double timestamp;
        DataReaderUtil::timeString2timecount_2(elements[0], timestamp);
        if (timestamp < start_time - 2) continue;
        if (timestamp > end_time + 2) break;
        Eigen::Matrix4d gnss_pos;
        gnss_pos << stod(elements[1]), stod(elements[2]), stod(elements[3]), stod(elements[4]),
                    stod(elements[5]), stod(elements[6]), stod(elements[7]), stod(elements[8]),
                    stod(elements[9]), stod(elements[10]), stod(elements[11]), stod(elements[12]),
                    0, 0, 0, 1; 
        timestamps.emplace_back(timestamp);
        car_poses.emplace_back(gnss2car * gnss_pos * car2gnss);
    }
}


void LidarToCar::LoadPointCloud(const std::string &point_cloud_path,
                                PointCloud::Ptr pcd) 
{
    if (pcl::io::loadPCDFile(point_cloud_path, *pcd) < 0) {
        LOGE("cannot open pcd file %s.", point_cloud_path.c_str());
        exit(1);
    }
}


} // namespace calibration
} // namespace autoCalib