/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#pragma once

#include <pcl/common/transforms.h>
#include <unordered_map>
// #include <opencv/cv.h>
#include "optimize.hpp"
#include "utils/myso3.hpp"
#include <fstream>
#include <mutex>
#include <thread>

#define MIN_PS 7

typedef std::vector<Eigen::Vector3d> PL_VEC;
// typedef pcl::PointXYZINormal pcl::PointXYZINormal;
const double one_three = (1.0 / 3.0);
// double feat_eigen_limit[2] = {3 * 3, 2 * 2};
// double feat_eigen_limit[2] = {5 * 5, 3 * 3};
double feat_eigen_limit[2] = {4 * 4, 3 * 3};
double opt_feat_eigen_limit[2] = {4 * 4, 3 * 3};

// Key of hash table
class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// Hash value
namespace std {
template <> struct hash<VOXEL_LOC> {
  size_t operator()(const VOXEL_LOC &s) const {
    using std::size_t;
    using std::hash;
    return ((hash<int64_t>()(s.x) ^ (hash<int64_t>()(s.y) << 1)) >> 1) ^
           (hash<int64_t>()(s.z) << 1);
  }
};
}

struct M_POINT {
  float xyz[3];
  int count = 0;
};

// get feature point if a single pcd
// Similar with PCL voxelgrid filter
void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZINormal> &pl_feat,
                         double voxel_size) {
  if (voxel_size < 0.01) {
    return;
  }

  std::unordered_map<VOXEL_LOC, M_POINT> feat_map;
  uint plsize = pl_feat.size();

  for (uint i = 0; i < plsize; i++) {
    pcl::PointXYZINormal &p_c = pl_feat[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      iter->second.xyz[0] += p_c.x;
      iter->second.xyz[1] += p_c.y;
      iter->second.xyz[2] += p_c.z;
      iter->second.count++;
    } else {
      M_POINT anp;
      anp.xyz[0] = p_c.x;
      anp.xyz[1] = p_c.y;
      anp.xyz[2] = p_c.z;
      anp.count = 1;
      feat_map[position] = anp;
    }
  }

  plsize = feat_map.size();
  pl_feat.clear();
  pl_feat.resize(plsize);

  uint i = 0;
  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
    pl_feat[i].x = iter->second.xyz[0] / iter->second.count;
    pl_feat[i].y = iter->second.xyz[1] / iter->second.count;
    pl_feat[i].z = iter->second.xyz[2] / iter->second.count;
    i++;
  }
}

// P_fix in the paper
// Summation of P_fix
class SIG_VEC_CLASS {
public:
  Eigen::Matrix3d sigma_vTv;
  Eigen::Vector3d sigma_vi;
  int sigma_size;

  SIG_VEC_CLASS() {
    sigma_vTv.setZero();
    sigma_vi.setZero();
    sigma_size = 0;
  }

  void tozero() {
    sigma_vTv.setZero();
    sigma_vi.setZero();
    sigma_size = 0;
  }
};

class OCTO_TREE {
public:
  static int voxel_windowsize;
  static std::vector<Eigen::Matrix4d> imu_transmat;
  std::vector<PL_VEC *> plvec_orig;
  std::vector<PL_VEC *> plvec_tran;

  int octo_state; // 0 is end of tree, 1 is not
  PL_VEC sig_vec_points;
  SIG_VEC_CLASS sig_vec;
  int ftype;
  int points_size, sw_points_size;
  double feat_eigen_ratio, feat_eigen_ratio_test;
  pcl::PointXYZINormal ap_centor_direct;
  pcl::PointXYZINormal ap_centor_direct_zero;
  pcl::PointXYZINormal ap_centor_direct_orig;
  std::vector<pcl::PointXYZINormal> ap_centor_direct_origs;
  double voxel_center[3]; // x, y, z
  double quater_length;
  OCTO_TREE *leaves[8];
  bool is2opt;
  int capacity;
  pcl::PointCloud<pcl::PointXYZINormal> root_centors;

  OCTO_TREE(int ft, int capa) : ftype(ft), capacity(capa) {
    octo_state = 0;
    for (int i = 0; i < 8; i++) {
      leaves[i] = nullptr;
    }

    for (int i = 0; i < capacity; i++) {
      plvec_orig.push_back(new PL_VEC());
      plvec_tran.push_back(new PL_VEC());
      // ap_centor_direct_origs.resize(capacity);
    }
    is2opt = true;
  }

  int getpointsize() {
    int num = 0;
    for (int i = 0; i < OCTO_TREE::voxel_windowsize; i++) {
      num += plvec_tran[i]->size();
    }
    return num;
  }
  void savePointNormal() {
    int asize;
    for (int i = 0; i < OCTO_TREE::voxel_windowsize; i++) {
      asize = plvec_tran[i]->size();
      Eigen::Matrix3d covMat(Eigen::Matrix3d::Zero());
      Eigen::Vector3d center(0, 0, 0);
      for (uint j = 0; j < asize; j++) {
        covMat += (*plvec_orig[i])[j] * (*plvec_orig[i])[j].transpose();
        center += (*plvec_orig[i])[j];
      }
      covMat += sig_vec.sigma_vTv;
      center += sig_vec.sigma_vi;
      center /= static_cast<double>(asize);
      covMat =
          covMat / static_cast<double>(asize) - center * center.transpose();

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
      Eigen::Vector3d direct_vec = saes.eigenvectors().col(2 * ftype);

      ap_centor_direct_origs[i].x = center.x();
      ap_centor_direct_origs[i].y = center.y();
      ap_centor_direct_origs[i].z = center.z();
      ap_centor_direct_origs[i].normal_x = direct_vec.x();
      ap_centor_direct_origs[i].normal_y = direct_vec.y();
      ap_centor_direct_origs[i].normal_z = direct_vec.z();
    }
  }
  // Used by "recut"
  void calc_eigen() {
    Eigen::Matrix3d covMat(Eigen::Matrix3d::Zero());
    Eigen::Matrix3d covMat_zero(Eigen::Matrix3d::Zero());
    Eigen::Matrix3d covMat_orig(Eigen::Matrix3d::Zero());
    Eigen::Vector3d center(0, 0, 0);
    Eigen::Vector3d center_zero(0, 0, 0);
    Eigen::Vector3d center_orig(0, 0, 0);

    uint asize;
    for (int i = 0; i < OCTO_TREE::voxel_windowsize; i++) {
      if (i == 0) {
        asize = plvec_tran[i]->size();
        for (uint j = 0; j < asize; j++) {
          covMat_zero += (*plvec_tran[i])[j] * (*plvec_tran[i])[j].transpose();
          center_zero += (*plvec_tran[i])[j];
          covMat_orig += (*plvec_orig[i])[j] * (*plvec_orig[i])[j].transpose();
          center_orig += (*plvec_orig[i])[j];
        }
      }
      asize = plvec_tran[i]->size();
      for (uint j = 0; j < asize; j++) {
        covMat += (*plvec_tran[i])[j] * (*plvec_tran[i])[j].transpose();
        center += (*plvec_tran[i])[j];
      }
    }

    covMat += sig_vec.sigma_vTv;
    center += sig_vec.sigma_vi;
    center /= points_size;
    covMat_zero += sig_vec.sigma_vTv;
    center_zero += sig_vec.sigma_vi;
    center_zero /= double(plvec_orig[0]->size());
    covMat_orig += sig_vec.sigma_vTv;
    center_orig += sig_vec.sigma_vi;
    center_orig /= double(plvec_orig[0]->size());

    covMat = covMat / points_size - center * center.transpose();
    covMat_zero = covMat_zero / double(plvec_tran[0]->size()) -
                  center_zero * center_zero.transpose();
    covMat_orig = covMat_orig / double(plvec_tran[0]->size()) -
                  center_orig * center_orig.transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
    feat_eigen_ratio = saes.eigenvalues()[2] / saes.eigenvalues()[ftype];
    Eigen::Vector3d direct_vec = saes.eigenvectors().col(2 * ftype);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes_zero(covMat_zero);
    Eigen::Vector3d direct_vec_zero = saes_zero.eigenvectors().col(2 * ftype);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes_orig(covMat_orig);
    Eigen::Vector3d direct_vec_orig = saes_orig.eigenvectors().col(2 * ftype);

    ap_centor_direct.x = center.x();
    ap_centor_direct.y = center.y();
    ap_centor_direct.z = center.z();
    ap_centor_direct.normal_x = direct_vec.x();
    ap_centor_direct.normal_y = direct_vec.y();
    ap_centor_direct.normal_z = direct_vec.z();

    ap_centor_direct_zero.x = center_zero.x();
    ap_centor_direct_zero.y = center_zero.y();
    ap_centor_direct_zero.z = center_zero.z();
    ap_centor_direct_zero.normal_x = direct_vec_zero.x();
    ap_centor_direct_zero.normal_y = direct_vec_zero.y();
    ap_centor_direct_zero.normal_z = direct_vec_zero.z();

    ap_centor_direct_orig.x = center_orig.x();
    ap_centor_direct_orig.y = center_orig.y();
    ap_centor_direct_orig.z = center_orig.z();
    ap_centor_direct_orig.normal_x = direct_vec_orig.x();
    ap_centor_direct_orig.normal_y = direct_vec_orig.y();
    ap_centor_direct_orig.normal_z = direct_vec_orig.z();
    // this->savePointNormal();
  }

  // Cut root voxel into small pieces
  // frame_head: Position of newest scan in sliding window
  void recut(int layer, uint frame_head,
             pcl::PointCloud<pcl::PointXYZINormal> &pl_feat_map) {
    if (octo_state == 0) {
      points_size = 0;
      for (int i = 0; i < OCTO_TREE::voxel_windowsize; i++) {
        points_size += plvec_tran[i]->size();
      }

      points_size += sig_vec.sigma_size;
      if (points_size < MIN_PS) {
        feat_eigen_ratio = -1;
        return;
      }

      calc_eigen(); // calculate eigenvalue ratio

      // if (isnan(feat_eigen_ratio)) {
      //     feat_eigen_ratio = -1;
      //     return;
      // }

      if (feat_eigen_ratio >= feat_eigen_limit[ftype]) {
        pl_feat_map.push_back(ap_centor_direct);
        return;
      }

      // if(layer == 3)
      if (layer == 5) {
        return;
      }

      octo_state = 1;
      // All points in slidingwindow should be put into subvoxel
      frame_head = 0;
    }

    int leafnum;
    uint a_size;

    for (int i = frame_head; i < OCTO_TREE::voxel_windowsize; i++) {
      a_size = plvec_tran[i]->size();
      for (uint j = 0; j < a_size; j++) {
        int xyz[3] = {0, 0, 0};
        for (uint k = 0; k < 3; k++) {
          if ((*plvec_tran[i])[j][k] > voxel_center[k]) {
            xyz[k] = 1;
          }
        }
        leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves[leafnum] == nullptr) {
          leaves[leafnum] = new OCTO_TREE(ftype, capacity);
          leaves[leafnum]->voxel_center[0] =
              voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
          leaves[leafnum]->voxel_center[1] =
              voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
          leaves[leafnum]->voxel_center[2] =
              voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
          leaves[leafnum]->quater_length = quater_length / 2;
        }
        leaves[leafnum]->plvec_orig[i]->push_back((*plvec_orig[i])[j]);
        leaves[leafnum]->plvec_tran[i]->push_back((*plvec_tran[i])[j]);
      }
    }

    if (layer != 0) {
      for (int i = frame_head; i < OCTO_TREE::voxel_windowsize; i++) {
        if (plvec_tran[i]->size() != 0) {
          std::vector<Eigen::Vector3d>().swap(*plvec_orig[i]);
          std::vector<Eigen::Vector3d>().swap(*plvec_tran[i]);
        }
      }
    }

    layer++;
    for (uint i = 0; i < 8; i++) {
      if (leaves[i] != nullptr) {
        leaves[i]->recut(layer, frame_head, pl_feat_map);
      }
    }
  }
};

int OCTO_TREE::voxel_windowsize = 0;
std::vector<Eigen::Matrix4d> OCTO_TREE::imu_transmat =
    std::vector<Eigen::Matrix4d>(0);

void clear_tree(OCTO_TREE *root) {
  if (root != nullptr) {
    for (int i = 0; i < 8; i++) {
      clear_tree(root->leaves[i]);
    }
    // for (int i = 0; i < root->plvec_orig.size(); i++) {
    //     // PL_VEC().swap(*(root->plvec_orig[i]));
    //     // PL_VEC().swap(*(root->plvec_tran[i]));
    //     delete(root->plvec_orig[i]);
    //     delete(root->plvec_tran[i]);
    // }
    delete (root);
    for (int i = 0; i < 8; i++) {
      if (root->leaves[i] != NULL)
        root->leaves[i] = NULL;
    }
    root = NULL;
  }
}

// Put feature points into root voxel
// feat_map: The hash table which manages voxel map
// pl_feat: Current feature pointcloud
// R_p, t_p: Current pose
// feattype: 0 is surf, 1 is corn
// fnum: The position in sliding window
// capacity: The capacity of sliding window, a little bigger than windowsize
void cut_voxel(std::unordered_map<VOXEL_LOC, OCTO_TREE *> &feat_map,
               pcl::PointCloud<pcl::PointXYZI>::Ptr pl_feat, Eigen::Matrix4d T,
               int feattype, int fnum, int capacity) {
  double voxel_size[2] = {1, 1}; // {surf, corn}
  uint plsize = pl_feat->size();
  for (uint i = 0; i < plsize; i++) {
    // Transform point to world coordinate
    pcl::PointXYZI &p_c = pl_feat->points[i];
    Eigen::Vector3d pvec_orig(p_c.x, p_c.y, p_c.z);
    Eigen::Vector3d pvec_tran =
        T.block<3, 3>(0, 0) * pvec_orig + T.block<3, 1>(0, 3);

    // Eigen::Vector3d pvec_tran = R_p * pvec_orig + t_p;

    // Determine the key of hash table
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = pvec_tran[j] / voxel_size[feattype];
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);

    // Find corresponding voxel
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      iter->second->plvec_orig[fnum]->push_back(pvec_orig);
      iter->second->plvec_tran[fnum]->push_back(pvec_tran);
      iter->second->is2opt = true;
    } else // If not finding, build a new voxel
    {
      OCTO_TREE *ot = new OCTO_TREE(feattype, capacity);
      ot->plvec_orig[fnum]->push_back(pvec_orig);
      ot->plvec_tran[fnum]->push_back(pvec_tran);

      // Voxel center coordinate
      ot->voxel_center[0] = (0.5 + position.x) * voxel_size[feattype];
      ot->voxel_center[1] = (0.5 + position.y) * voxel_size[feattype];
      ot->voxel_center[2] = (0.5 + position.z) * voxel_size[feattype];
      ot->quater_length = voxel_size[feattype] / 4.0; // A quater of side length
      feat_map[position] = ot;
    }
  }
}

void getVoxelMap(OCTO_TREE *root,
                 pcl::PointCloud<pcl::PointXYZRGB> &display_pcd,
                 int &voxel_num) {
  if (root == nullptr)
    return;
  if (root->octo_state == 0) {
    if (root->plvec_tran.size() != 0) {
      if (root->quater_length > 0.5)
        std::cout << root->quater_length << "  ";
      voxel_num++;
      int r = (rand() % 255 + 10) % 255;
      int g = (rand() % 255 + 10) % 255;
      int b = (rand() % 255 + 10) % 255;
      for (int j = 0; j < OCTO_TREE::voxel_windowsize; j++) {
        // std::cout << current_root->plvec_tran[j]->size() << "  ";
        for (int pt = 0; pt < root->plvec_tran[j]->size(); pt++) {
          pcl::PointXYZRGB pcd_pt;
          pcd_pt.x = (*root->plvec_tran[j])[pt](0);
          pcd_pt.y = (*root->plvec_tran[j])[pt](1);
          pcd_pt.z = (*root->plvec_tran[j])[pt](2);
          pcd_pt.r = r;
          pcd_pt.b = b;
          pcd_pt.g = g;
          display_pcd.push_back(pcd_pt);
        }
      }
    }
    return;
  } else {
    for (int i = 0; i < 8; i++)
      getVoxelMap(root->leaves[i], display_pcd, voxel_num);
  }
}

void displayVoxelMap(std::unordered_map<VOXEL_LOC, OCTO_TREE *> map) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_pcd(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  srand((unsigned)time(NULL));
  int voxel_num = 0;
  for (auto iter = map.begin(); iter != map.end(); ++iter) {
    getVoxelMap(iter->second, *display_pcd, voxel_num);
  }
  std::cout << "voxel num: " << voxel_num << std::endl;
  std::cout << "display pcd " << display_pcd->points.size() << std::endl;
  pcl::PCDWriter writer;
  writer.write("voxel_map.pcd", *display_pcd);
}
void getVoxelResidual1(OCTO_TREE *root, ceres::Problem &problem,
                       double *deltaRPY, double *deltaT) {
  if (root == nullptr)
    return;
  if (root->octo_state == 0) {
    if ((*root->plvec_orig[0]).size() != 0) {
      Eigen::Vector3d point_average;
      Eigen::Vector3d normal;
      point_average(0) = root->ap_centor_direct_zero.x;
      point_average(1) = root->ap_centor_direct_zero.y;
      point_average(2) = root->ap_centor_direct_zero.z;
      normal(0) = root->ap_centor_direct_zero.normal_x;
      normal(1) = root->ap_centor_direct_zero.normal_y;
      normal(2) = root->ap_centor_direct_zero.normal_z;
      for (int j = 0; j < OCTO_TREE::voxel_windowsize; j++) {
        Eigen::Matrix4d imu_tran = OCTO_TREE::imu_transmat[j];
        for (int pt = 0; pt < root->plvec_orig[j]->size(); pt++) {
          Eigen::Vector3d current_laser_point = (*root->plvec_orig[j])[pt];
          ceres::CostFunction *cost_function = BalmVoxelEnergy1::Create(
              current_laser_point, point_average, normal, imu_tran);
          problem.AddResidualBlock(cost_function, nullptr, deltaRPY, deltaT);
        }
      }
    }
    return;
  } else {
    for (int i = 0; i < 8; i++)
      getVoxelResidual1(root->leaves[i], problem, deltaRPY, deltaT);
  }
}

void getVoxelResidual2(OCTO_TREE *root, ceres::Problem &problem,
                       double *deltaRPY, double *deltaT) {
  if (root == nullptr)
    return;
  if (root->octo_state == 0) {
    if ((*root->plvec_orig[0]).size() != 0) {
      Eigen::Matrix4d imu_tran_orig = OCTO_TREE::imu_transmat[0];
      Eigen::Vector3d point_average;
      Eigen::Vector3d normal;
      point_average(0) = root->ap_centor_direct_orig.x;
      point_average(1) = root->ap_centor_direct_orig.y;
      point_average(2) = root->ap_centor_direct_orig.z;
      normal(0) = root->ap_centor_direct_orig.normal_x;
      normal(1) = root->ap_centor_direct_orig.normal_y;
      normal(2) = root->ap_centor_direct_orig.normal_z;
      for (int j = 0; j < OCTO_TREE::voxel_windowsize; j++) {
        Eigen::Matrix4d imu_tran = OCTO_TREE::imu_transmat[j];
        for (int pt = 0; pt < root->plvec_orig[j]->size(); pt++) {
          Eigen::Vector3d current_laser_point = (*root->plvec_orig[j])[pt];
          ceres::CostFunction *cost_function =
              BalmVoxelEnergy2::Create(current_laser_point, point_average,
                                       normal, imu_tran, imu_tran_orig);
          problem.AddResidualBlock(cost_function, nullptr, deltaRPY, deltaT);
        }
      }
    }
    return;
  } else {
    for (int i = 0; i < 8; i++)
      getVoxelResidual2(root->leaves[i], problem, deltaRPY, deltaT);
  }
}

void getVoxelResidual2_NOT(OCTO_TREE *root, ceres::Problem &problem,
                           double *deltaRPY) {
  if (root == nullptr)
    return;
  if (root->octo_state == 0) {
    if ((*root->plvec_orig[0]).size() != 0) {
      Eigen::Matrix4d imu_tran_orig = OCTO_TREE::imu_transmat[0];
      Eigen::Vector3d point_average;
      Eigen::Vector3d normal;
      point_average(0) = root->ap_centor_direct_orig.x;
      point_average(1) = root->ap_centor_direct_orig.y;
      point_average(2) = root->ap_centor_direct_orig.z;
      normal(0) = root->ap_centor_direct_orig.normal_x;
      normal(1) = root->ap_centor_direct_orig.normal_y;
      normal(2) = root->ap_centor_direct_orig.normal_z;
      for (int j = 0; j < OCTO_TREE::voxel_windowsize; j++) {
        Eigen::Matrix4d imu_tran = OCTO_TREE::imu_transmat[j];
        for (int pt = 0; pt < root->plvec_orig[j]->size(); pt++) {
          Eigen::Vector3d current_laser_point = (*root->plvec_orig[j])[pt];
          ceres::CostFunction *cost_function =
              BalmVoxelEnergy2_NOT::Create(current_laser_point, point_average,
                                           normal, imu_tran, imu_tran_orig);
          problem.AddResidualBlock(cost_function, nullptr, deltaRPY);
        }
      }
    }
    return;
  } else {
    for (int i = 0; i < 8; i++)
      getVoxelResidual2_NOT(root->leaves[i], problem, deltaRPY);
  }
}

void getVoxelResidual3(OCTO_TREE *root, ceres::Problem &problem,
                       double *deltaRPY, double *deltaT) {
  if (root == nullptr)
    return;
  if (root->octo_state == 0) {
    if (root->plvec_tran.size() != 0) {
      int point_size = root->getpointsize();
      std::vector<Eigen::Matrix4d> imu_trans = OCTO_TREE::imu_transmat;
      std::vector<pcl::PointXYZINormal> ap_centor_origns;
      std::vector<std::vector<Eigen::Vector3d> *> current_laser_points;
      current_laser_points = root->plvec_orig;
      ap_centor_origns = root->ap_centor_direct_origs;
      for (int j = 0; j < OCTO_TREE::voxel_windowsize; j++) {
        Eigen::Matrix4d cur_imu_tran = OCTO_TREE::imu_transmat[j];
        for (int pt = 0; pt < root->plvec_orig[j]->size(); pt++) {
          Eigen::Vector3d cur_laser_point = (*root->plvec_orig[j])[pt];
          ceres::CostFunction *cost_function = BalmVoxelEnergy3::Create(
              cur_laser_point, cur_imu_tran, current_laser_points,
              ap_centor_origns, imu_trans);
          problem.AddResidualBlock(cost_function, nullptr, deltaRPY, deltaT);
        }
      }
    }
    return;
  } else {
    for (int i = 0; i < 8; i++)
      getVoxelResidual3(root->leaves[i], problem, deltaRPY, deltaT);
  }
}

void getVoxelResidualTrans(OCTO_TREE *root, ceres::Problem &problem,
                           const Eigen::Matrix4d deltaTrans, double *deltaT) {
  if (root == nullptr)
    return;
  if (root->octo_state == 0) {
    if ((*root->plvec_orig[0]).size() != 0) {
      Eigen::Matrix4d imu_tran_orig = OCTO_TREE::imu_transmat[0];
      Eigen::Vector3d point_average;
      Eigen::Vector3d normal;
      point_average(0) = root->ap_centor_direct_orig.x;
      point_average(1) = root->ap_centor_direct_orig.y;
      point_average(2) = root->ap_centor_direct_orig.z;
      normal(0) = root->ap_centor_direct_orig.normal_x;
      normal(1) = root->ap_centor_direct_orig.normal_y;
      normal(2) = root->ap_centor_direct_orig.normal_z;
      for (int j = 0; j < OCTO_TREE::voxel_windowsize; j++) {
        Eigen::Matrix4d imu_tran = OCTO_TREE::imu_transmat[j];
        for (int pt = 0; pt < root->plvec_orig[j]->size(); pt++) {
          Eigen::Vector3d current_laser_point = (*root->plvec_orig[j])[pt];
          ceres::CostFunction *cost_function = BalmVoxelTransEnergy::Create(
              current_laser_point, point_average, normal, imu_tran,
              imu_tran_orig, deltaTrans);
          problem.AddResidualBlock(cost_function, nullptr, deltaT);
        }
      }
    }
    return;
  } else {
    for (int i = 0; i < 8; i++)
      getVoxelResidualTrans(root->leaves[i], problem, deltaTrans, deltaT);
  }
}
// method = 1, 2, 3
void optimizeDeltaTrans(std::unordered_map<VOXEL_LOC, OCTO_TREE *> surf_map,
                        std::unordered_map<VOXEL_LOC, OCTO_TREE *> corn_map,
                        const int &method, double *deltaRPY, double *deltaT) {
  ceres::Problem problem;

  double deltaQ[4];
  ceres::AngleAxisToQuaternion(deltaRPY, deltaQ);
  // ceres::EulerAnglesToQuaternion(deltaRPY, deltaQ);
  std::cout << "deltaQ: " << deltaQ[0] << " " << deltaQ[1] << " " << deltaQ[2]
            << " " << deltaQ[3] << std::endl;

  if (method == 1) {
    for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
      getVoxelResidual1(iter->second, problem, deltaQ, deltaT);
    }
    for (auto iter = corn_map.begin(); iter != corn_map.end(); ++iter) {
      getVoxelResidual1(iter->second, problem, deltaQ, deltaT);
    }
  }
  if (method == 2) {
    for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
      getVoxelResidual2(iter->second, problem, deltaQ, deltaT);
    }
    for (auto iter = corn_map.begin(); iter != corn_map.end(); ++iter) {
      getVoxelResidual2(iter->second, problem, deltaQ, deltaT);
    }
  }
  if (method == 3) {
    for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
      getVoxelResidual3(iter->second, problem, deltaQ, deltaT);
    }
    for (auto iter = corn_map.begin(); iter != corn_map.end(); ++iter) {
      getVoxelResidual3(iter->second, problem, deltaQ, deltaT);
    }
  }
  if (method == 4) {
    for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
      getVoxelResidual2_NOT(iter->second, problem, deltaQ);
    }
    for (auto iter = corn_map.begin(); iter != corn_map.end(); ++iter) {
      getVoxelResidual2_NOT(iter->second, problem, deltaQ);
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;
  ceres::QuaternionToAngleAxis(deltaQ, deltaRPY);
}