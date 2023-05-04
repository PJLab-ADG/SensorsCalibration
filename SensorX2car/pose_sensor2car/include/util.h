#ifndef UTIL_H_
#define UTIL_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <utility>
#include <fstream>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <datatable.h>
#include <bspline.h>
#include <bsplinebuilder.h>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }
inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

class Util {
 public:
    Util() = default;
    ~Util() = default;

    static Eigen::Matrix4d GetMatrix(
        double x, double y, double z, double roll, double pitch, double yaw) {
        Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
        Eigen::Vector3d translation = GetTranslation(x, y, z);
        Eigen::Matrix3d rotation = GetRotation(roll, pitch, yaw);
        ret.block<3, 1>(0, 3) = translation;
        ret.block<3, 3>(0, 0) = rotation;
        return ret;
    }
    static Eigen::Matrix4d GetMatrix(const Eigen::Vector3d& translation,
                                     const Eigen::Matrix3d& rotation) {
        Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
        ret.block<3, 1>(0, 3) = translation;
        ret.block<3, 3>(0, 0) = rotation;
        return ret;
    }
    static Eigen::Vector3d GetTranslation(const Eigen::Matrix4d& matrix) {
        return Eigen::Vector3d(matrix(0, 3), matrix(1, 3), matrix(2, 3));
    }
    static Eigen::Vector3d GetTranslation(double x, double y, double z) {
        return Eigen::Vector3d(x, y, z);
    }
    static Eigen::Matrix3d GetRotation(const Eigen::Matrix4d& matrix) {
        return matrix.block<3, 3>(0, 0);
    }
    static Eigen::Matrix3d GetRotation(double roll, double pitch, double yaw) {
        Eigen::Matrix3d rotation;
        Eigen::Matrix3d Rz;
        Rz << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
        Eigen::Matrix3d Ry;
        Ry << cos(pitch), 0., sin(pitch), 0., 1., 0., -sin(pitch), 0.,
            cos(pitch);

        Eigen::Matrix3d Rx;
        Rx << 1., 0., 0., 0., cos(roll), -sin(roll), 0., sin(roll), cos(roll);
        rotation = Rz * Ry * Rx;
        return rotation;
    }
    static double GetX(const Eigen::Matrix4d& matrix) { return matrix(0, 3); }
    static double GetY(const Eigen::Matrix4d& matrix) { return matrix(1, 3); }
    static double GetZ(const Eigen::Matrix4d& matrix) { return matrix(2, 3); }
    /* avoid using it: matrix.block<3, 3>(0, 0).eulerAngles(0, 1, 2)[0];*/
    static double GetRoll(const Eigen::Matrix4d& matrix) {
        Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
        
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);
        double y = atan2(n(1), n(0));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y),
                         -o(0) * sin(y) + o(1) * cos(y));
        return r;
    }
    static double GetPitch(const Eigen::Matrix4d& matrix) {
        Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
        Eigen::Vector3d n = R.col(0);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));

        return p;
    }
    static double GetYaw(const Eigen::Matrix4d& matrix) {
        Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
        Eigen::Vector3d n = R.col(0);
        double y = atan2(n(1), n(0));

        return y;
    }

    static void QV2Matrix(const Eigen::Quaterniond& quat,
                          const Eigen::Vector3d& vect,
                          Eigen::Matrix4d* mat) {
        *mat = Eigen::Matrix4d::Identity();
        mat->block<3, 3>(0, 0) = quat.matrix();
        mat->block<3, 1>(0, 3) = vect;
    }
    static void Matrix2QV(const Eigen::Matrix4d& mat,
                          Eigen::Quaterniond* quat,
                          Eigen::Vector3d* vect) {
        *quat = Eigen::Quaterniond(mat.block<3, 3>(0, 0));
        quat->normalize();
        *vect = mat.block<3, 1>(0, 3);
    }
    static void Matrix2Isometry(const Eigen::Matrix4d& mat,
                                Eigen::Isometry3d* iso) {
        iso->setIdentity();
        iso->linear() = mat.block<3, 3>(0, 0);
        iso->translation() = mat.block<3, 1>(0, 3);
    }
    static void Isometry2Matrix(const Eigen::Isometry3d& iso,
                                Eigen::Matrix4d* mat) {
        *mat = Eigen::Matrix4d::Identity();
        mat->block<3, 3>(0, 0) = iso.rotation();
        mat->block<3, 1>(0, 3) = iso.translation();
    }
    static void QV2Isometry(const Eigen::Quaterniond& quat,
                            const Eigen::Vector3d& vect,
                            Eigen::Isometry3d* iso) {
        iso->setIdentity();
        iso->linear() = quat.toRotationMatrix();
        iso->translation() = vect;
    }
    static void Isometry2QV(const Eigen::Isometry3d& iso,
                            Eigen::Quaterniond* quat,
                            Eigen::Vector3d* vect) {
        *quat = Eigen::Quaterniond(iso.rotation());
        *vect = iso.translation();
    }

    static bool LoadLidarPose(std::string pose_file, std::vector<Eigen::Matrix4d> &lidar_pose){
        std::ifstream infile(pose_file);
        std::string line;
        if (!infile) {
            std::cout << "Can't open file " << pose_file << std::endl;
            exit(1);
        }
        while(getline(infile,line)){
            std::stringstream ss(line);
            std::vector<std::string>  elements;
            std::string elem;
            while(getline(ss, elem,' ')){
                elements.emplace_back(elem);
            }
            if(elements.size()!=12){
                printf("num of line elements error! skip this line.\n");
                continue;
            }
            Eigen::Matrix4d pose;
            pose << stod(elements[0]), stod(elements[1]), stod(elements[2]),stod(elements[3]),
                    stod(elements[4]), stod(elements[5]), stod(elements[6]),stod(elements[7]),
                    stod(elements[8]), stod(elements[9]), stod(elements[10]),stod(elements[11]),
                    0,                 0,                 0,                 1;
            lidar_pose.push_back(pose);
        }
        return true;
    }

    static double WeightMean(const std::vector<double> a, std::vector<double> weight)
    {
        double sumweight = std::accumulate(std::begin(weight), std::end(weight), 0.0);
        for (unsigned int i = 0; i < weight.size();i++)
        {
            weight[i] /= sumweight;
        }
        double sum = 0;
        for (unsigned int i = 0; i < weight.size();i++)
        {
            sum += weight[i] * a[i];
        }
        return  sum;
    }

    static double Mean(const std::vector<double> a)
    {
        double sum = std::accumulate(std::begin(a), std::end(a), 0.0);
        return  sum / a.size();
    }

    static double Std(const std::vector<double> a)
    {
	    double mean = Mean(a);
        double accum = 0;
        std::for_each(std::begin(a), std::end(a), [&](const double d)
                      { accum += (d - mean) * (d - mean); });

        double std = sqrt(accum / (a.size() - 1));
        return std;
    }

    static bool DeleteOutliers(std::vector<double> &nums, std::vector<double> &new_nums, double s)
    {
        int n = nums.size();
        std::sort(nums.begin(), nums.end());
        double median = (n % 2 == 1) ? nums[n / 2] : (nums[n / 2] + nums[n / 2 - 1]) / 2;
        std::vector<double> deviations;
        for (int i = 0; i < n; i++)
        {
            deviations.push_back(fabs(nums[i] - median));
        }
        std::sort(deviations.begin(), deviations.end());
        double mad = (n % 2 == 1) ? deviations[n / 2] : (deviations[n / 2] + deviations[n / 2 - 1]) / 2;
        for (int i = 0; i < n; i++)
        {
            if (fabs(nums[i] - median) < s * mad)
                new_nums.push_back(nums[i]);
        }
        // std::cout << "Delete " << nums.size() - new_nums.size() << " outliers" << std::endl;
        return true;
    }
    
 private:
};

#endif  // UTIL_H_