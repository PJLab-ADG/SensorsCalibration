#ifndef UTIL_H_
#define UTIL_H_

#include <vector>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <utility>
#include <fstream>
#include <dirent.h>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <chrono>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

struct Var6
{
    float value[6] = {0};
};

class Util {
    public:
        Util() = default;
        ~Util() = default;


        static Eigen::Matrix4f GetDeltaT(const float var[6]) {
            auto deltaR = Eigen::Matrix3f(
                Eigen::AngleAxisf(DEG2RAD(var[2]), Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(DEG2RAD(var[1]), Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(DEG2RAD(var[0]), Eigen::Vector3f::UnitX()));
            Eigen::Matrix4f deltaT = Eigen::Matrix4f::Identity();
            deltaT.block<3, 3>(0, 0) = deltaR;
            deltaT(0, 3) = var[3];
            deltaT(1, 3) = var[4];
            deltaT(2, 3) = var[5];
            return deltaT;
        }

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
        static float GetX(const Eigen::Matrix4f& matrix) { return matrix(0, 3); }
        static float GetY(const Eigen::Matrix4f& matrix) { return matrix(1, 3); }
        static float GetZ(const Eigen::Matrix4f& matrix) { return matrix(2, 3); }
        /* avoid using it: matrix.block<3, 3>(0, 0).eulerAngles(0, 1, 2)[0];*/
        static float GetRoll(const Eigen::Matrix4f& matrix) {
            Eigen::Matrix3f R = matrix.block<3, 3>(0, 0);
            
            Eigen::Vector3f n = R.col(0);
            Eigen::Vector3f o = R.col(1);
            Eigen::Vector3f a = R.col(2);
            float y = atan2(n(1), n(0));
            float r = atan2(a(0) * sin(y) - a(1) * cos(y),
                            -o(0) * sin(y) + o(1) * cos(y));
            return r;
        }
        static float GetPitch(const Eigen::Matrix4f& matrix) {
            Eigen::Matrix3f R = matrix.block<3, 3>(0, 0);
            Eigen::Vector3f n = R.col(0);
            float y = atan2(n(1), n(0));
            float p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));

            return p;
        }
        static float GetYaw(const Eigen::Matrix4f& matrix) {
            Eigen::Matrix3f R = matrix.block<3, 3>(0, 0);
            Eigen::Vector3f n = R.col(0);
            float y = atan2(n(1), n(0));

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

        template <class T>
        static T WeightMean(const std::vector<T> a, std::vector<T> weight)
        {
            assert(a.size() == weight.size());
            T sumweight = std::accumulate(std::begin(weight), std::end(weight), 0.0);
            for (unsigned int i = 0; i < weight.size();i++)
            {
                weight[i] /= sumweight;
            }
            T sum = 0;
            for (unsigned int i = 0; i < weight.size();i++)
            {
                sum += weight[i] * a[i];
            }
            return sum;
        }
        
        template <class T>
        static T Mean(const std::vector<T> a)
        {
            T sum = std::accumulate(std::begin(a), std::end(a), 0.0);
            return  sum / a.size();
        }

        template <class T>
        static T Std(const std::vector<T> a)
        {
            T mean = Mean(a);
            float accum = 0;
            std::for_each(std::begin(a), std::end(a), [&](const T d)
                        { accum += (d - mean) * (d - mean); });

            T std = sqrt(accum / a.size());
            return std;
        }

        static void GenVars(int rpy_range, float rpy_resolution, int xyz_range, float xyz_resolution, std::vector<Var6> & out)
        {
            Var6 var;
            for (int i1 = -rpy_range; i1 <= rpy_range; i1 += 1)
            {
                for(int i2 = -rpy_range; i2 <= rpy_range; i2 += 1)
                {
                    for(int i3 = -rpy_range; i3 <= rpy_range; i3 += 1)
                    {
                        for(int i4 = -xyz_range; i4 <= xyz_range; i4 += 1)
                        {
                            for(int i5 = -xyz_range; i5 <= xyz_range; i5 += 1)
                            {
                                for(int i6 = -xyz_range; i6 <= xyz_range; i6 += 1)
                                {
                                    var.value[0] = i1 * rpy_resolution;
                                    var.value[1] = i2 * rpy_resolution;
                                    var.value[2] = i3 * rpy_resolution;
                                    var.value[3] = i4 * xyz_resolution;
                                    var.value[4] = i5 * xyz_resolution;
                                    var.value[5] = i6 * xyz_resolution;
                                    out.push_back(var);
                                }
                            }
                        }
                    }
                }
            }
        }

        static void UndistImg(cv::Mat & img, Eigen::Matrix3f intrinsic, std::vector<double> dist)
        {
            cv::Mat I = cv::Mat::eye(3, 3, CV_32FC1);
            cv::Mat mapX, mapY;
            cv::Mat img_undist = cv::Mat(img.size(), CV_32FC3);
            cv::Mat K;
            cv::eigen2cv(intrinsic, K);
            cv::initUndistortRectifyMap(K, dist, I, K, img.size(), CV_32FC1, mapX, mapY);
            cv::remap(img, img_undist, mapX, mapY, cv::INTER_LINEAR);
            img = img_undist;
        }

    private:
};

#endif  // UTIL_H_