#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>

struct NDTParameter {
  double ndt_maxIterations = 50;
  double ndt_transformation_epsilon = 0.05;
  double ndt_step_size = 0.1; // need to adjust
  double resolution = 10;     // nedd to adjust
};

struct PlaneParam {
  PlaneParam() {}
  PlaneParam(const Eigen::Vector3d &n, double i) : normal(n), intercept(i) {}
  Eigen::Vector3d normal;
  double intercept;
};

struct PointCloudBbox {
  int min_x = 0;
  int min_y = 0;
  int min_z = 0;

  int max_x = 0;
  int max_y = 0;
  int max_z = 0;
};

class Registrator {
public:
  Registrator();

  bool
  GroundPlaneExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr ng_cloud,
                        PlaneParam &plane);
  void
  PointCloudDownSampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud);
  size_t
  ComputeVoxelOccupancy(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
                        const float var[6], const PointCloudBbox &bbox);

  // registration method
  bool
  RegistrationByICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
                    Eigen::Matrix4f &transform);
  bool
  RegistrationByNDT(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
                    const NDTParameter ndt_parameter,
                    Eigen::Matrix4f &transform);
  bool
  RegistrationByGICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
                     Eigen::Matrix4f &transform);
  bool RegistrationByPointToPlane(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
      Eigen::Matrix4f &transform);
  bool RegistrationByGroundPlane(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
      Eigen::Matrix4f &transform);
  bool RegistrationByVoxelOccupancy(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
      Eigen::Matrix4f &transform);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr all_octree_;
};