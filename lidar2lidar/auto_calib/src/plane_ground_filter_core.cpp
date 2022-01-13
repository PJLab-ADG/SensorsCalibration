#include "plane_ground_filter_core.h"
#include "logging.hpp"

PlaneGroundFilter::PlaneGroundFilter() {
  sensor_height_ = 1.7;
  min_distance_ = 2;
  max_distance_ = 75;
  clip_height_ = 4;
  sensor_model_ = 64;
  num_iter_ = 3;
  num_lpr_ = 20;
  th_seeds_ = 1.2;
  th_dist_ = 0.3;
  th_dist_d_ = 0.2;
  //
  g_seeds_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
  g_ground_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
  g_not_ground_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
  g_all_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void PlaneGroundFilter::estimate_plane() {
  Eigen::Matrix3f cov;
  Eigen::Vector4f pc_mean;
  pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
  // Singular Value Decomposition: SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(
      cov, Eigen::DecompositionOptions::ComputeFullU);
  // use the least singular vector as normal
  normal_ = (svd.matrixU().col(2));
  // mean ground seeds value
  Eigen::Vector3f seeds_mean = pc_mean.head<3>();

  // according to normal.T*[x,y,z] = -d
  d_ = -(normal_.transpose() * seeds_mean)(0, 0);
  // set distance threhold to `th_dist - d`
  th_dist_d_ = th_dist_ - d_;
}

void PlaneGroundFilter::extract_initial_seeds(
    const pcl::PointCloud<pcl::PointXYZI> &p_sorted) {
  // LPR is the mean of low point representative
  double sum = 0;
  int cnt = 0;
  // Calculate the mean height value.
  for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
    sum += p_sorted.points[i].z;
    cnt++;
  }
  double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0
  g_seeds_pc->clear();
  // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
  for (int i = 0; i < p_sorted.points.size(); i++) {
    if (p_sorted.points[i].z < lpr_height + th_seeds_) {
      g_seeds_pc->points.push_back(p_sorted.points[i]);
    }
  }
  // return seeds points
}

void PlaneGroundFilter::clip_above(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr out) {
  pcl::ExtractIndices<pcl::PointXYZI> cliper;

  cliper.setInputCloud(in);
  pcl::PointIndices indices;
#pragma omp for
  for (size_t i = 0; i < in->points.size(); i++) {
    if (in->points[i].z > clip_height_) {
      indices.indices.push_back(i);
    }
  }
  cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
  cliper.setNegative(true); // ture to remove the indices
  cliper.filter(*out);
}

void PlaneGroundFilter::remove_close_far_pt(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr out) {
  pcl::ExtractIndices<pcl::PointXYZI> cliper;

  cliper.setInputCloud(in);
  pcl::PointIndices indices;
#pragma omp for
  for (size_t i = 0; i < in->points.size(); i++) {
    double distance = sqrt(in->points[i].x * in->points[i].x +
                           in->points[i].y * in->points[i].y);

    if ((distance < min_distance_) || (distance > max_distance_)) {
      indices.indices.push_back(i);
    }
  }
  cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
  cliper.setNegative(true); // ture to remove the indices
  cliper.filter(*out);
}

void PlaneGroundFilter::post_process(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr out) {
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cliped_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  clip_above(in, cliped_pc_ptr);
  const pcl::PointCloud<pcl::PointXYZI>::Ptr remove_close(
      new pcl::PointCloud<pcl::PointXYZI>);
  remove_close_far_pt(cliped_pc_ptr, out);
}
bool point_cmp(pcl::PointXYZI a, pcl::PointXYZI b) { return a.z < b.z; }
void PlaneGroundFilter::point_cb(
    const pcl::PointCloud<pcl::PointXYZI> &in_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud, PlaneParam &plane) {
  g_all_pc = in_cloud.makeShared();
  pcl::PointCloud<pcl::PointXYZI> laserCloudIn = in_cloud;
  pcl::PointCloud<pcl::PointXYZI> laserCloudIn_org = in_cloud;
  // 2.Sort on Z-axis value.
  sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_cmp);
  // 3.Error point removal
  // As there are some error mirror reflection under the ground,
  // here regardless point under 2* sensor_height
  // Sort point according to height, here uses z-axis in default
  pcl::PointCloud<pcl::PointXYZI>::iterator it = laserCloudIn.points.begin();
  for (int i = 0; i < laserCloudIn.points.size(); i++) {
    if (laserCloudIn.points[i].z < -1.5 * sensor_height_) {
      it++;
    } else {
      break;
    }
  }
  laserCloudIn.points.erase(laserCloudIn.points.begin(), it);
  // 4. Extract init ground seeds.
  extract_initial_seeds(laserCloudIn);
  g_ground_pc = g_seeds_pc;
  // 5. Ground plane fitter mainloop
  for (int i = 0; i < num_iter_; i++) {
    estimate_plane();
    g_ground_pc->clear();
    g_not_ground_pc->clear();

    // pointcloud to matrix
    Eigen::MatrixXf points(laserCloudIn_org.points.size(), 3);
    int j = 0;
    for (auto p : laserCloudIn_org.points) {
      points.row(j++) << p.x, p.y, p.z;
    }
    // ground plane model
    Eigen::VectorXf result = points * normal_;
    // threshold filter
    for (int r = 0; r < result.rows(); r++) {
      if (result[r] < th_dist_d_) {
        // g_all_pc->points[r].label = 1u; // means ground
        g_ground_pc->points.push_back(laserCloudIn_org[r]);
      } else {
        // g_all_pc->points[r].label = 0u; // means not ground and non
        // clusterred
        g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr final_no_ground(
      new pcl::PointCloud<pcl::PointXYZI>);
  post_process(g_not_ground_pc, final_no_ground);
  // return params
  *g_cloud = *g_ground_pc;
  *ng_cloud = *final_no_ground;
  plane.normal(0) = normal_(0);
  plane.normal(1) = normal_(1);
  plane.normal(2) = normal_(2);
  plane.intercept = d_;
  //   pcl::io::savePCDFileBinary("ground.pcd", *g_ground_pc);
  //   pcl::io::savePCDFileBinary("no_ground.pcd", *final_no_ground);
}

bool PlaneGroundFilter::GroundPlaneExtraction(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud, PlaneParam &plane) {
  point_cb(*in_cloud, g_cloud, ng_cloud, plane);
  return true;
}