#include "handeye_calibration.hpp"
#include "common/logging.hpp"
#include "utils/transform_util.hpp"

namespace autoCalib {
namespace calibration {

HandEyeCalibration::HandEyeCalibration(){
    frame_count = 0;
    frame_countT = 0;
    cflag = true;
    tic = Eigen::Vector3d::Zero();
    Rlidar.push_back(Eigen::Matrix3d::Identity());
    Rcar.push_back(Eigen::Matrix3d::Identity());
    Rlidar_g.push_back(Eigen::Matrix3d::Identity());
    r_lidar2car = Eigen::Matrix3d::Identity();
}

bool HandEyeCalibration::calibrate(const Eigen::Matrix3d &lidar_rot, 
                                   const Eigen::Matrix3d &car_rot, 
                                   Eigen::Matrix3d &calib_r_result)
{
    frame_count++;
    Rlidar.push_back(lidar_rot);
    // imu or gps??
    Rcar.push_back(car_rot);
    // r_lidar2car only have 
    // Rlidar_g.push_back(r_lidar2car.inverse() * car_rot * r_lidar2car);
    Eigen::Matrix3d r_car2lidar = r_lidar2car.inverse();

    Eigen::MatrixXf A = Eigen::MatrixXf::Random(frame_count * 4, 4);
    A.setZero();
    int sum_ok = 0;
    for (int i = 1; i <= frame_count; i++)
    {
        Eigen::Quaterniond r1(Rlidar[i]);
        // Eigen::Quaterniond r2(Rlidar_g[i]);
        Eigen::Quaterniond r2(r_car2lidar * Rcar[i] * r_lidar2car);

        double angular_distance = 180 / M_PI * r1.angularDistance(r2);
        double huber = angular_distance > 3.0 ? 3.0 / angular_distance : 1.0;
        ++sum_ok;
        Eigen::Matrix4d L, R;

        double w = Eigen::Quaterniond(Rlidar[i]).w();
        Eigen::Vector3d q = Eigen::Quaterniond(Rlidar[i]).vec();
        L.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() + TransformUtil::skewSymmetric(q);
        L.block<3, 1>(0, 3) = q;
        L.block<1, 3>(3, 0) = -q.transpose();
        L(3, 3) = w;

        Eigen::Quaterniond R_ij(Rcar[i]);
        w = R_ij.w();
        q = R_ij.vec();
        R.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() - TransformUtil::skewSymmetric(q);
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

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<float, 4, 1> xf = svd.matrixV().col(3);
    Eigen::Matrix<double, 4, 1> x;
    x(0, 0) = xf(0, 0);
    x(1, 0) = xf(1, 0);
    x(2, 0) = xf(2, 0);
    x(3, 0) = xf(3, 0);
    Eigen::Quaterniond estimated_R(x);

    r_lidar2car = estimated_R.toRotationMatrix().inverse();
    Eigen::Vector3f ric_cov;
    ric_cov = svd.singularValues().tail<3>();
    
    Eigen::Vector3d rpy = TransformUtil::GetRPY(r_lidar2car);
    LOGI("lidar2car extrinsic is: roll = %.2f, pitch = %.2f, yaw = %.2f.",
         rad2deg(rpy(0)), rad2deg(rpy(1)), rad2deg(rpy(2)));
    rpy = TransformUtil::GetRPY(r_lidar2car.inverse());
    LOGI("lidar2car inverse is: roll = %.2f, pitch = %.2f, yaw = %.2f.",
         rad2deg(rpy(0)), rad2deg(rpy(1)), rad2deg(rpy(2)));
    if (frame_count >= window_size && ric_cov(1) > 0.25)
    {
        calib_r_result = r_lidar2car;
        return true;
    } else {
        calib_r_result = r_lidar2car;
        return false;
    }
}

bool HandEyeCalibration::calibrate(const Eigen::Matrix3d &lidar_rot, const Eigen::Vector3d &lidar_pos,
                                   const Eigen::Matrix3d &car_rot, const Eigen::Vector3d & car_pos,
                                   const Eigen::Matrix3d &calib_r_result, Eigen::Vector3d &calib_tic_result)
{
    frame_countT++;
    Rlidar.push_back(lidar_rot);
    // imu or gps??
    Rcar.push_back(car_rot);
    // r_lidar2car only have 
    // Rlidar_g.push_back(r_lidar2car.inverse() * car_rot * r_lidar2car);
    if(cflag) {
        Rlidar.clear(); Rcar.clear(); Plidar.clear(); Pcar.clear(); cflag = false;
        Eigen::Matrix3d ideal_ric;
        Rlidar.push_back(Eigen::Matrix3d::Identity());
        Rcar.push_back(Eigen::Matrix3d::Identity());
        Plidar.push_back(Eigen::Vector3d::Zero());Pcar.push_back(Eigen::Vector3d::Zero());
        }
    Plidar.push_back(lidar_pos);
    Pcar.push_back(car_pos);
    Rlidar.push_back(lidar_rot);
    Rcar.push_back(car_rot);
    // Rc_g.push_back(ric.inverse() * Utility::ypr2R(Rimu_ypr) * ric);

    Eigen::MatrixXd A = Eigen::MatrixXd::Random(frame_countT * 3, 3);
    Eigen::VectorXd B = Eigen::VectorXd::Random(frame_countT * 3, 1);

    std::cout << "frame " << frame_countT << std::endl;
    for (int i = 1; i <= frame_countT; i++)
    {
        Eigen::Matrix3d A1 = Rlidar[i] - Eigen::Matrix3d::Identity();  
        Eigen::Vector3d bstep = A1*tic;
        Eigen::Vector3d b1 = calib_r_result*Pcar[i] - Plidar[i];
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

    Eigen::Matrix<double, 3, 1> xf = A.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(B);
    // Matrix<double, 4, 1> x = svd.matrixV().col(3);

    // cout << svd.singularValues().transpose() << endl;
    // Vector3f tic_cov;
    // tic_cov = svd.singularValues().tail<3>();
    // std::cout << "cov: " << tic_cov(1) <<std::endl;
    if (frame_countT >= window_size*10&& (tic-xf).norm()<1e-2)
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




} // namespace calibration
} // namespace autoCalib