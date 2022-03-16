#include "lidar_car_calibration.hpp"
#include "utils/transform_util.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>

namespace autoCalib {
namespace calibration {

LidarCarCalibrator::LidarCarCalibrator()
{
    calib_conf_ = 0;
    lidar2car_height_ = 0;
    frame_cnt_ = 0;

    handeye_calibrator_.reset(new HandEyeCalibration);
    registrator_.reset(new Registrator);
}

bool LidarCarCalibrator::inputLidar(const double& timestamp,
                                    PointCloud::Ptr cloud)
{
    Eigen::Matrix3d car_rot;
    if (!getCarRot(timestamp, car_rot)) return false;
    Eigen::Matrix3d car_rot_prev2now;

    Eigen::Matrix4d lidar_pose_rot;
    registrator_->inputFrame(cloud, lidar_pose_rot);
    if (frame_cnt_ != 0) {
        car_rot_prev2now =  prev_car_rot_.inverse() * car_rot;
        Eigen::Vector3d c_rpy = TransformUtil::GetRPY(car_rot_prev2now);
        Eigen::Vector3d l_rpy = TransformUtil::GetRPY(lidar_pose_rot.block<3, 3>(0, 0));
        LOGI("car rot is: roll = %.3f, pitch = %.3f, yaw = %.3f.",
            rad2deg(c_rpy(0)), rad2deg(c_rpy(1)), rad2deg(c_rpy(2)));
        LOGI("lidar rot is: roll = %.3f, pitch = %.3f, yaw = %.3f.",
            rad2deg(l_rpy(0)), rad2deg(l_rpy(1)), rad2deg(l_rpy(2)));

        // hand eye calibration frame
        Eigen::Matrix3d lidar2car_rot;
            // get car pos


        // skip static mode
        double move_angle = Eigen::Quaterniond(Eigen::Matrix3d::Identity()).
                            angularDistance(Eigen::Quaterniond(car_rot_prev2now));
        if (rad2deg(move_angle) < min_move_angle_) {
            std::cout << "move angle too small.\n";
        } else {
            if (handeye_calibrator_->calibrate(lidar_pose_rot.block<3, 3>(0, 0), 
                                            car_rot_prev2now, lidar2car_rot)) {
                Eigen::Vector3d rpy = TransformUtil::GetRPY(lidar2car_rot);
                handeye_results_.emplace_back(lidar2car_rot);
                LOGI("lidar2car extrinsic is: roll = %.2f, pitch = %.2f, yaw = %.2f.",
                    rad2deg(rpy(0)), rad2deg(rpy(1)), rad2deg(rpy(2)));
            }
            handeye_final_r_ = lidar2car_rot;
        }
    }

    maxRangeFilter(cloud, cloud);
    std::cout << "finish registration extraction!\n";
    // pcl::io::savePLYFile("cloud3.ply", *cloud);

    pcl::PointIndices::Ptr ground_idx(new pcl::PointIndices);
    pcl::ApproximateProgressiveMorphologicalFilter<PointXYZ> pmf_ground;
    pmf_ground.setInputCloud(cloud);
    pmf_ground.setMaxWindowSize(60);
    pmf_ground.setSlope(1.f);
    // pmf_ground.setInitialDistance(2.f);
    pmf_ground.setMaxDistance(0.5f);
    pmf_ground.extract(ground_idx->indices);

    pcl::ExtractIndices<PointXYZ> ground_extract;
    ground_extract.setInputCloud(cloud);
    ground_extract.setIndices(ground_idx);

    std::cout << "finish ground extraction!\n";
    PointCloud::Ptr ground_pcd(new PointCloud);
    ground_extract.filter(*ground_pcd);

    // pcl::io::savePLYFile("ground3.ply", *ground_pcd);
    // char stophaha[10];
    // std::cin >> stophaha;

    Eigen::Matrix3d ground_cov_mat;
    PointXYZ ground_mean;
    // pcl::computeCentroid(cloud, ground_idx, ground_mean);
    // pcl::computeCovarianceMatrix(cloud, ground_idx, ground_cov_mat);
    pcl::computeCentroid(*ground_pcd, ground_mean);
    pcl::computeCovarianceMatrix(*ground_pcd, ground_cov_mat);

    Eigen::EigenSolver<Eigen::Matrix3d> es(ground_cov_mat);
    Eigen::Matrix3d val = es.pseudoEigenvalueMatrix();
    Eigen::Matrix3d vec = es.pseudoEigenvectors();
    // find minum eigen value
    double val_min = val(0, 0);
    int cov_min_idx = 0;
    for (int mi = 1; mi < 3; mi ++) {
        if (val_min > val(mi, mi)) {
            cov_min_idx = mi;
            val_min = val(mi, mi);
        }
    }
    Eigen::Vector3d ground_normal = Eigen::Vector3d(vec(0, cov_min_idx),
                                    vec(1, cov_min_idx), vec(2, cov_min_idx));
    if (ground_normal(2) < 0) ground_normal = -ground_normal;
    Eigen::Vector3d mean_vec(ground_mean.x, ground_mean.y, ground_mean.z);
    double ground_intercept = -(ground_normal.transpose() * mean_vec)(0, 0);
    LOGI("ground fitting: norm = (%f, %f, %f), intercept = %f",
         ground_normal(0), ground_normal(1), ground_normal(2),
         ground_intercept);

    Eigen::Vector3d car_gplane(0, 0, 1);
    Eigen::Vector3d rot_axis = ground_normal.cross(car_gplane);
    rot_axis.normalize();
    double alpha = -std::acos(ground_normal.dot(car_gplane));
    Eigen::Matrix3d R_mp;
    R_mp = Eigen::AngleAxisd(alpha, rot_axis);
    Eigen::Vector3d t_mp(0, 0, -ground_intercept / ground_normal(2));
    Eigen::Matrix4d T_lidar2g = TransformUtil::GetMatrix(t_mp, R_mp).inverse();
    LOGI("lidar to ground extrinsic is: roll = %f, pitch = %f, yaw = %f, "
         "x = %f, y = %f, z = %f",
         rad2deg(TransformUtil::GetRoll(T_lidar2g)), 
         rad2deg(TransformUtil::GetPitch(T_lidar2g)),
         rad2deg(TransformUtil::GetYaw(T_lidar2g)),
         T_lidar2g(0, 3), T_lidar2g(1, 3), T_lidar2g(2, 3));

    // save ground plane calibration result(roll & pitch)
    ground_results_.emplace_back(T_lidar2g);
    Eigen::Quaterniond q_lidar2g(T_lidar2g.block<3, 3>(0, 0));
    // TODO: apply confidence based on whatever?
    // double confidence = 1;
    double confidence = ground_intercept * ground_intercept - 1.8;
    if (ground_normal(2) < lidar_ground_normal_check_) confidence = 0;
    if (ground_intercept < min_intercept_) confidence = 0;
    double q_scale = confidence / (calib_conf_ + confidence);
    lidar2car_height_ = q_scale * T_lidar2g(2, 3) + 
                        (1 - q_scale) * lidar2car_height_;
    if (calib_conf_ == 0) ground_rot_q_ = q_lidar2g;
    else ground_rot_q_ = ground_rot_q_.slerp(q_scale, q_lidar2g);
    calib_conf_ += confidence;

    // save to prev
    prev_car_rot_ = car_rot;

    frame_cnt_ ++;
    std::cout << "\n==>Frame " << frame_cnt_ << std::endl;
    return true;
}

bool LidarCarCalibrator::inputLidarT(const double& timestamp,
                                    PointCloud::Ptr cloud)
{
    Eigen::Matrix3d car_rot;
    Eigen::Vector3d car_pos;
    if (!getCarRot(timestamp, car_rot)||!getCarPosition(timestamp, car_pos)) return false;

    Eigen::Matrix3d car_rot_prev2now;
    Eigen::Vector3d car_pos_prev2now;

    Eigen::Matrix4d lidar_pose_rot;
    registrator_->inputFrame(cloud, lidar_pose_rot);
    if (frame_cnt_ != 0) {
        car_rot_prev2now =  prev_car_rot_.inverse() * car_rot;
        car_pos_prev2now =  car_pos - prev_car_pos_;
        Eigen::Vector3d c_rpy = TransformUtil::GetRPY(car_rot_prev2now);
        Eigen::Vector3d l_rpy = TransformUtil::GetRPY(lidar_pose_rot.block<3, 3>(0, 0));
        LOGI("car rot is: roll = %.3f, pitch = %.3f, yaw = %.3f.",
            rad2deg(c_rpy(0)), rad2deg(c_rpy(1)), rad2deg(c_rpy(2)));
        LOGI("lidar rot is: roll = %.3f, pitch = %.3f, yaw = %.3f.",
            rad2deg(l_rpy(0)), rad2deg(l_rpy(1)), rad2deg(l_rpy(2)));

        // hand eye calibration frame
        Eigen::Matrix3d lidar2car_rot;
        Eigen::Vector3d lidar2car_pos;
            // get car pos


        // skip static mode
        double move_angle = Eigen::Quaterniond(Eigen::Matrix3d::Identity()).
                            angularDistance(Eigen::Quaterniond(car_rot_prev2now));
        if (rad2deg(move_angle) < min_move_angle_) {
            std::cout << "move angle too small.\n";
        } else {
            if (handeye_calibrator_->calibrate(lidar_pose_rot.block<3, 3>(0, 0), 
                                            car_rot_prev2now, lidar2car_rot)) {
                if(handeye_calibrator_->calibrate(lidar_pose_rot.block<3, 3>(0, 0),
                                            lidar_pose_rot.block<3,1>(0, 3),
                                            car_rot_prev2now, car_pos_prev2now,
                                            lidar2car_rot,lidar2car_pos)){
                                                Eigen::Vector3d rpy = TransformUtil::GetRPY(lidar2car_rot);
                                                handeye_results_.emplace_back(lidar2car_rot);
                                                handeye_results_t_.emplace_back(lidar2car_pos);
                                                LOGI("lidar2car extrinsic is: roll = %.2f, pitch = %.2f, yaw = %.2f.",
                                                rad2deg(rpy(0)), rad2deg(rpy(1)), rad2deg(rpy(2)));
                                                LOGI("lidar2car extrinsic t is: x = %.2f , y = %.2f, z = %.2f. ",lidar2car_pos(0),lidar2car_pos(1),lidar2car_pos(2));
                                            }
            }
            handeye_final_r_ = lidar2car_rot;
        }
    }

    maxRangeFilter(cloud, cloud);
    std::cout << "finish registration extraction!\n";
    // pcl::io::savePLYFile("cloud3.ply", *cloud);

    pcl::PointIndices::Ptr ground_idx(new pcl::PointIndices);
    pcl::ApproximateProgressiveMorphologicalFilter<PointXYZ> pmf_ground;
    pmf_ground.setInputCloud(cloud);
    pmf_ground.setMaxWindowSize(60);
    pmf_ground.setSlope(1.f);
    // pmf_ground.setInitialDistance(2.f);
    pmf_ground.setMaxDistance(0.5f);
    pmf_ground.extract(ground_idx->indices);

    pcl::ExtractIndices<PointXYZ> ground_extract;
    ground_extract.setInputCloud(cloud);
    ground_extract.setIndices(ground_idx);

    std::cout << "finish ground extraction!\n";
    PointCloud::Ptr ground_pcd(new PointCloud);
    ground_extract.filter(*ground_pcd);

    // pcl::io::savePLYFile("ground3.ply", *ground_pcd);
    // char stophaha[10];
    // std::cin >> stophaha;

    Eigen::Matrix3d ground_cov_mat;
    PointXYZ ground_mean;
    // pcl::computeCentroid(cloud, ground_idx, ground_mean);
    // pcl::computeCovarianceMatrix(cloud, ground_idx, ground_cov_mat);
    pcl::computeCentroid(*ground_pcd, ground_mean);
    pcl::computeCovarianceMatrix(*ground_pcd, ground_cov_mat);

    Eigen::EigenSolver<Eigen::Matrix3d> es(ground_cov_mat);
    Eigen::Matrix3d val = es.pseudoEigenvalueMatrix();
    Eigen::Matrix3d vec = es.pseudoEigenvectors();
    // find minum eigen value
    double val_min = val(0, 0);
    int cov_min_idx = 0;
    for (int mi = 1; mi < 3; mi ++) {
        if (val_min > val(mi, mi)) {
            cov_min_idx = mi;
            val_min = val(mi, mi);
        }
    }
    Eigen::Vector3d ground_normal = Eigen::Vector3d(vec(0, cov_min_idx),
                                    vec(1, cov_min_idx), vec(2, cov_min_idx));
    if (ground_normal(2) < 0) ground_normal = -ground_normal;
    Eigen::Vector3d mean_vec(ground_mean.x, ground_mean.y, ground_mean.z);
    double ground_intercept = -(ground_normal.transpose() * mean_vec)(0, 0);
    LOGI("ground fitting: norm = (%f, %f, %f), intercept = %f",
         ground_normal(0), ground_normal(1), ground_normal(2),
         ground_intercept);

    Eigen::Vector3d car_gplane(0, 0, 1);
    Eigen::Vector3d rot_axis = ground_normal.cross(car_gplane);
    rot_axis.normalize();
    double alpha = -std::acos(ground_normal.dot(car_gplane));
    Eigen::Matrix3d R_mp;
    R_mp = Eigen::AngleAxisd(alpha, rot_axis);
    Eigen::Vector3d t_mp(0, 0, -ground_intercept / ground_normal(2));
    Eigen::Matrix4d T_lidar2g = TransformUtil::GetMatrix(t_mp, R_mp).inverse();
    LOGI("lidar to ground extrinsic is: roll = %f, pitch = %f, yaw = %f, "
         "x = %f, y = %f, z = %f",
         rad2deg(TransformUtil::GetRoll(T_lidar2g)), 
         rad2deg(TransformUtil::GetPitch(T_lidar2g)),
         rad2deg(TransformUtil::GetYaw(T_lidar2g)),
         T_lidar2g(0, 3), T_lidar2g(1, 3), T_lidar2g(2, 3));

    // save ground plane calibration result(roll & pitch)
    ground_results_.emplace_back(T_lidar2g);
    Eigen::Quaterniond q_lidar2g(T_lidar2g.block<3, 3>(0, 0));
    // TODO: apply confidence based on whatever?
    // double confidence = 1;
    double confidence = ground_intercept * ground_intercept - 1.8;
    if (ground_normal(2) < lidar_ground_normal_check_) confidence = 0;
    if (ground_intercept < min_intercept_) confidence = 0;
    double q_scale = confidence / (calib_conf_ + confidence);
    lidar2car_height_ = q_scale * T_lidar2g(2, 3) + 
                        (1 - q_scale) * lidar2car_height_;
    if (calib_conf_ == 0) ground_rot_q_ = q_lidar2g;
    else ground_rot_q_ = ground_rot_q_.slerp(q_scale, q_lidar2g);
    calib_conf_ += confidence;

    // save to prev
    prev_car_rot_ = car_rot;
    prev_car_pos_ = car_pos;
    frame_cnt_ ++;
    std::cout << "\n==>Frame " << frame_cnt_ << std::endl;
    return true;
}

void LidarCarCalibrator::getTrandformation(Eigen::Matrix4d& lidar2car)
{
    // combine ground plane and hand eye calibration result
    if (handeye_results_.size() < 1) 
        LOGW("Hand Eye calibration is not completed.");

    // Eigen::Matrix3d handeye_R = handeye_results_[handeye_results_.size() - 1];
    Eigen::Vector3d handeye_rpy = TransformUtil::GetRPY(handeye_final_r_);
    LOGI("handeye lidar2car: roll = %.2f, pitch = %.2f, yaw = %.2f.",
         rad2deg(handeye_rpy(0)), rad2deg(handeye_rpy(1)), rad2deg(handeye_rpy(2)));

    Eigen::Matrix3d ground_R = ground_rot_q_.toRotationMatrix();
    Eigen::Vector3d ground_rpy = TransformUtil::GetRPY(ground_R);
    LOGI("ground lidar2car: roll = %.2f, pitch = %.2f, yaw = %.2f.",
         rad2deg(ground_rpy(0)), rad2deg(ground_rpy(1)), rad2deg(ground_rpy(2)));

    if (rad2deg(fabs(handeye_rpy(1) - ground_rpy(1))) > pitch_diff_thres_) 
        LOGW("pitch diff larger than threshold.");
    lidar2car = Eigen::Matrix4d::Identity();
    lidar2car.block<3, 3>(0, 0) = TransformUtil::GetRotation(ground_rpy(0),
        (ground_rpy(1)+handeye_rpy(1))/2, handeye_rpy(2));
    lidar2car(2, 3) = lidar2car_height_;
}

bool LidarCarCalibrator::getCarRot(const double& timestamp,
                                    Eigen::Matrix3d& car_rot)
{
    for (;car_pos_idx_ < car_timestamp_.size(); car_pos_idx_++) {
        if (car_timestamp_[car_pos_idx_] > timestamp) break;
    }
    // if (car_timestamp_[car_pos_idx_] < timestamp) {
    if (car_pos_idx_ == car_timestamp_.size()) {
        LOGW("reach end of imu data.");
        return false;
    }
    int idx_min = std::max(car_pos_idx_-1, 0);
    // interpolate pos
    Eigen::Quaterniond q_min(car_poses_[idx_min].block<3,3>(0,0));
    Eigen::Quaterniond q_max(car_poses_[car_pos_idx_].block<3,3>(0,0));
    // attitude interpolation
    float cos_theta = q_min.dot(q_max);
    // to prevent go the long way
    if (cos_theta < 0.0) {
        LOGW("Quaternion Cos %.3f", cos_theta*180/M_PI);
        q_max.conjugate();
    }
    
    double scale = (timestamp - car_timestamp_[idx_min]) / 
                   (car_timestamp_[car_pos_idx_] - car_timestamp_[idx_min]);
    Eigen::Quaterniond q = q_min.slerp(scale, q_max);
    car_rot = q.toRotationMatrix();
    return true;
}
bool LidarCarCalibrator::getCarPosition(const double& timestamp,
                                         Eigen::Vector3d & car_pos)
{
    for (;car_pos_idx_1 < car_timestamp_.size(); car_pos_idx_1++) {
        if (car_timestamp_[car_pos_idx_1] > timestamp) break;
    }
    // if (car_timestamp_[car_pos_idx_1] < timestamp) {
    if (car_pos_idx_1 == car_timestamp_.size()) {
        LOGW("reach end of imu data.");
        return false;
    }
    int idx_min = std::max(car_pos_idx_1-1, 0);
    // interpolate pos
    Eigen::Vector3d t_min(car_poses_[idx_min].block<3,1>(0,3));
    Eigen::Vector3d t_max(car_poses_[car_pos_idx_1].block<3,1>(0,3));
    
    double scale = (timestamp - car_timestamp_[idx_min]) / 
                   (car_timestamp_[car_pos_idx_1] - car_timestamp_[idx_min]);
    Eigen::Vector3d t = scale*t_max +(1-scale)*t_min;
    car_pos = t;
    return true;    
}

void LidarCarCalibrator::maxRangeFilter(PointCloud::Ptr cloud_in, 
                                        PointCloud::Ptr cloud_out)
{
    if (cloud_in != cloud_out)
    {
        cloud_out->header = cloud_in->header;
        cloud_out->points.resize(cloud_in->points.size());
    }

    size_t j = 0;
    float thres_square = filter_max_range_ * filter_max_range_;
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        if (cloud_in->points[i].x * cloud_in->points[i].x + 
            cloud_in->points[i].y * cloud_in->points[i].y + 
            cloud_in->points[i].z * cloud_in->points[i].z > thres_square)
            continue;
        cloud_out->points[j] = cloud_in->points[i];
        j++;
    }
    if (j != cloud_in->points.size())
    {
        cloud_out->points.resize(j);
    }

    cloud_out->height = 1;
    cloud_out->width = static_cast<uint32_t>(j);
    cloud_out->is_dense = true;
}

}   // calibration 
}   // autoCalib