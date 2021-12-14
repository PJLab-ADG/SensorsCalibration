#include "registration.hpp"

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "logging.hpp"
#include "transform_util.hpp"

Registrator::Registrator() {
  all_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  all_octree_.reset(
      new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.05));
  all_octree_->setInputCloud(all_cloud_);
};

bool Registrator::GroundPlaneExtraction(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ng_cloud, PlaneParam &plane) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.2);
  seg.setInputCloud(in_cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return (-1);
  }
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(inliers);
  extract.filter(*g_cloud);
  extract.setNegative(true);
  extract.filter(*ng_cloud);
  plane.normal(0) = coefficients->values[0];
  plane.normal(1) = coefficients->values[1];
  plane.normal(2) = coefficients->values[2];
  plane.intercept = coefficients->values[3];
  return true;
}

bool Registrator::RegistrationByICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
    Eigen::Matrix4f &transform) {
  int max_iter =
      50; // stop when it converges or reaches the maximum number of iterations
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(src_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
  icp.setMaxCorrespondenceDistance(0.05);
  icp.setMaximumIterations(50);
  icp.setTransformationEpsilon(0.01);
  icp.align(*src_cloud);
  bool is_suceed = icp.hasConverged();
  double match_score = icp.getFitnessScore();
  transform = icp.getFinalTransformation();
  return is_suceed;
}

bool Registrator::RegistrationByNDT(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
    const NDTParameter ndt_parameter, Eigen::Matrix4f &transform) {

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setInputSource(src_cloud);
  ndt.setInputTarget(target_cloud);
  ndt.setMaximumIterations(ndt_parameter.ndt_maxIterations);
  ndt.setTransformationEpsilon(ndt_parameter.ndt_transformation_epsilon);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize(ndt_parameter.ndt_step_size);
  ndt.setResolution(ndt_parameter.resolution);
  ndt.align(*src_cloud);
  bool is_suceed = ndt.hasConverged();
  double match_score = ndt.getFitnessScore();
  transform = ndt.getFinalTransformation();
  return is_suceed;
}

bool Registrator::RegistrationByGICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
    Eigen::Matrix4f &transform) {
  int max_iter = 50;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setInputSource(src_cloud);
  gicp.setInputTarget(target_cloud);
  gicp.setMaximumIterations(max_iter);
  gicp.align(*src_cloud);
  bool is_suceed = gicp.hasConverged();
  double match_score = gicp.getFitnessScore();
  transform = gicp.getFinalTransformation();
  return is_suceed;
}

bool Registrator::RegistrationByPointToPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
    Eigen::Matrix4f &transform) {

  int max_iter = 50;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setRadiusSearch(0.5);
  pcl::PointCloud<pcl::Normal>::Ptr tgt_norm(new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(target_cloud);
  ne.compute(*tgt_norm);
  pcl::PointCloud<pcl::PointNormal>::Ptr tgt_concat(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*target_cloud, *tgt_norm, *tgt_concat);
  pcl::PointCloud<pcl::Normal>::Ptr src_norm(new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(src_cloud);
  pcl::PointCloud<pcl::PointNormal>::Ptr src_concat(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*src_cloud, *src_norm, *src_concat);
  pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
  using PointToPlane =
      pcl::registration::TransformationEstimationPointToPlaneLLS<
          pcl::PointNormal, pcl::PointNormal>;
  boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
  icp.setTransformationEstimation(point_to_plane);
  icp.setInputSource(src_concat);
  icp.setInputTarget(tgt_concat);
  icp.setMaximumIterations(max_iter);
  icp.align(*src_concat);
  bool is_suceed = icp.hasConverged();
  double match_score = icp.getFitnessScore();
  transform = icp.getFinalTransformation();
  return is_suceed;
}

bool Registrator::RegistrationByGroundPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
    Eigen::Matrix4f &transform) {
  PlaneParam source_plane;
  PlaneParam target_plane;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_g_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_ng_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_g_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_ng_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  GroundPlaneExtraction(src_cloud, source_g_cloud, source_ng_cloud,
                        source_plane);
  GroundPlaneExtraction(target_cloud, target_g_cloud, target_ng_cloud,
                        target_plane);
  Eigen::Vector3d rot_axis2 = source_plane.normal.cross(target_plane.normal);
  rot_axis2.normalize();
  double alpha2 = std::acos(source_plane.normal.dot(target_plane.normal));
  Eigen::Matrix3d R_ms;
  R_ms = Eigen::AngleAxisd(alpha2, rot_axis2);
  Eigen::Vector3d slave_intcpt_local(
      0, 0, -source_plane.intercept / source_plane.normal(2));
  Eigen::Vector3d slave_intcpt_master = R_ms * slave_intcpt_local;
  Eigen::Vector3d t_mp(0, 0, -target_plane.intercept / target_plane.normal(2));
  Eigen::Vector3d t_ms(0, 0, t_mp(2) - slave_intcpt_master(2));
  Eigen::Matrix4d T_ms = TransformUtil::GetMatrix(t_ms, R_ms);
  transform = T_ms.cast<float>();
  pcl::transformPointCloud(*src_cloud, *src_cloud, transform);
  return true;
}
bool Registrator::RegistrationByVoxelOccupancy(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
    Eigen::Matrix4f &transform) {
  PlaneParam source_plane;
  PlaneParam target_plane;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_g_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_ng_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_g_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_ng_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  GroundPlaneExtraction(src_cloud, source_g_cloud, source_ng_cloud,
                        source_plane);
  GroundPlaneExtraction(target_cloud, target_g_cloud, target_ng_cloud,
                        target_plane);
  PointCloudBbox bbox;
  for (const auto &src_pt : source_ng_cloud->points) {
    if (src_pt.x < bbox.min_x)
      bbox.min_x = src_pt.x;
    if (src_pt.y < bbox.min_y)
      bbox.min_y = src_pt.y;
    if (src_pt.z < bbox.min_z)
      bbox.min_z = src_pt.z;

    if (src_pt.x > bbox.max_x)
      bbox.max_x = src_pt.x;
    if (src_pt.y > bbox.max_y)
      bbox.max_y = src_pt.y;
    if (src_pt.z > bbox.max_z)
      bbox.max_z = src_pt.z;
  }
  float var[6] = {0}, bestVal[6] = {0};
  std::string varName[6] = {"roll", "pitch", "yaw", "tx", "ty", "tz"};
  size_t minPointCnt =
      ComputeVoxelOccupancy(source_ng_cloud, target_ng_cloud, var, bbox);
  // just adjust x and y, If you need to adjust other items, you can modify the
  // code
  for (int delta_x = -10; delta_x < 10; delta_x++) {
    var[3] = delta_x * 0.1;
    for (int delta_y = -10; delta_y < 10; delta_y++) {
      var[4] = delta_y * 0.1;
      size_t cnt =
          ComputeVoxelOccupancy(source_ng_cloud, target_ng_cloud, var, bbox);
      if (cnt < minPointCnt * (1 - 1e-4)) {
        minPointCnt = cnt;
        bestVal[3] = var[3];
        bestVal[4] = var[4];
      } else {
        // break;
      }
    }
  }
  transform = TransformUtil::GetDeltaT(bestVal).cast<float>();
  pcl::transformPointCloud(*src_cloud, *src_cloud, transform);
  return true;
}

void Registrator::PointCloudDownSampling(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud) {
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(in_cloud);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(*out_cloud);
}

size_t Registrator::ComputeVoxelOccupancy(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud, const float var[6],
    const PointCloudBbox &bbox) {

  Eigen::Matrix4d T = TransformUtil::GetDeltaT(var);
  for (const auto &src_pt : target_cloud->points) {
    if (src_pt.x < bbox.min_x || src_pt.y < bbox.min_y || src_pt.z < bbox.min_z)
      continue;
    if (src_pt.x > bbox.max_x || src_pt.y > bbox.max_y || src_pt.z > bbox.max_z)
      continue;
    pcl::PointXYZ dst_pt;
    dst_pt.x = src_pt.x;
    dst_pt.y = src_pt.y;
    dst_pt.z = src_pt.z;
    if (!all_octree_->isVoxelOccupiedAtPoint(dst_pt)) {
      all_octree_->addPointToCloud(dst_pt, all_cloud_);
    }
  }
  for (const auto &src_pt : src_cloud->points) {
    Eigen::Vector3d p(src_pt.x, src_pt.y, src_pt.z);
    Eigen::Vector3d p_res;
    p_res = T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);
    pcl::PointXYZ dst_pt;
    dst_pt.x = p_res(0);
    dst_pt.y = p_res(1);
    dst_pt.z = p_res(2);

    if (!all_octree_->isVoxelOccupiedAtPoint(dst_pt)) {
      all_octree_->addPointToCloud(dst_pt, all_cloud_);
    }
  }
  size_t pcdCnt = all_cloud_->size();
  all_cloud_->clear();
  all_octree_->deleteTree();
  return pcdCnt;
}