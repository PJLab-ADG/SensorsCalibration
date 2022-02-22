/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include "calibration.hpp"

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

const double eps = 1.0e-6;


Calibrator::Calibrator() { registrator_.reset(new ICPRegistrator); }

void Calibrator::LoadCalibrationData(
    const std::map<int32_t, pcl::PointCloud<pcl::PointXYZI>> lidar_points,
    const std::map<int32_t, InitialExtrinsic> extrinsics) {
  pcs_ = lidar_points;
  for (auto src : extrinsics) {
    int32_t device_id = src.first;
    InitialExtrinsic extrinsic = src.second;

    Eigen::Matrix3d rot = TransformUtil::GetRotation(extrinsic.euler_angles[0],
                                                     extrinsic.euler_angles[1],
                                                     extrinsic.euler_angles[2]);
    Eigen::Matrix4d init_ext =
        TransformUtil::GetMatrix(extrinsic.t_matrix, rot);
    init_extrinsics_.insert(std::make_pair(device_id, init_ext));
  }
}

void Calibrator::Calibrate() {
  float degree_2_radian = 0.017453293;
  LOGI("calibrate");
  int32_t master_id = 0;
  //调试代码：保存中间结果-------------------------------------------------------------------
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  auto master_iter = pcs_.find(0);
  pcl::PointCloud<pcl::PointXYZI> master_pc = master_iter->second;
  for (auto src : master_pc.points) {
    int32_t master_id = 0;
    pcl::PointXYZRGB point;
    point.x = src.x;
    point.y = src.y;
    point.z = src.z;
    point.r = color_map[master_id % 7][0];
    point.g = color_map[master_id % 7][1];
    point.b = color_map[master_id % 7][2];
    all_cloud->push_back(point);
  }

  for (auto iter = pcs_.begin(); iter != pcs_.end(); iter++) {
    int32_t slave_id = iter->first;
    if (slave_id == master_id)
      continue;

    pcl::PointCloud<pcl::PointXYZI> slave_pc = iter->second;
    pcl::PointCloud<pcl::PointXYZI> trans_cloud;
    Eigen::Matrix4d init_ext = init_extrinsics_[slave_id];
    pcl::transformPointCloud(slave_pc, trans_cloud, init_ext);
    for (auto src : trans_cloud.points) {
      pcl::PointXYZRGB point;
      point.x = src.x;
      point.y = src.y;
      point.z = src.z;
      point.r = color_map[slave_id % 7][0];
      point.g = color_map[slave_id % 7][1];
      point.b = color_map[slave_id % 7][2];
      all_cloud->push_back(point);
    }
  
  }
  all_cloud->height = 1;
  all_cloud->width = all_cloud->points.size();
  std::string path = "bad_case/results/";
  // std::string path = "./";
  path +=  "start_post";
  path +=  ".pcd";
  pcl::io::savePCDFileBinary(path, *all_cloud);


  //-----------------------------------------------------------------------------------------
  // auto master_iter = pcs_.find(master_id);
  // pcl::PointCloud<pcl::PointXYZI> master_pc = master_iter->second;
  pcl::PointCloud<pcl::PointXYZI>::Ptr master_pc_ptr = master_pc.makeShared();
  PlaneParam master_gplane;
  pcl::PointCloud<pcl::PointXYZI>::Ptr master_gcloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr master_ngcloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  bool ret = GroundPlaneExtraction(master_pc_ptr, master_gcloud,
  master_ngcloud, master_gplane);
  // pcl::io::savePCDFileBinary("master_gcloud.pcd", *master_gcloud);
  // pcl::io::savePCDFileBinary("master_ngcloud.pcd", *master_ngcloud);
  // std::cout<<master_gplane.normal<<std::endl;
  // std::cout<<master_gplane.intercept<<std::endl;
  // pcl::io::savePCDFileBinary("ran_master_ground.pcd", *master_gcloud);
  // pcl::io::savePCDFileBinary("ran_master_no_ground.pcd", *master_ngcloud);
  // PlaneGroundFilter plane_ground_filter;
  // bool ret = plane_ground_filter.GroundPlaneExtraction(
  //     master_pc_ptr, master_gcloud, master_ngcloud, master_gplane);
  // std::cout<<master_gplane.normal<<std::endl;
  // std::cout<<master_gplane.intercept<<std::endl;
  // return;

  if (!ret) {
    LOGE("master lidar ground fitting failed.\n");
    return;
  }
  registrator_->SetTargetCloud(master_gcloud, master_ngcloud, master_pc_ptr);
  Eigen::Vector3d t_mp(0, 0,
                       -master_gplane.intercept / master_gplane.normal(2));

  for (auto iter = pcs_.begin(); iter != pcs_.end(); iter++) {
    int32_t slave_id = iter->first;
    LOGI("slave %d begin", slave_id);
    if (slave_id == master_id)
      continue;
    LOGI("start calibrating slave lidar, id: %d\n", slave_id);
    pcl::PointCloud<pcl::PointXYZI> slave_pc = iter->second;
    if (init_extrinsics_.find(slave_id) == init_extrinsics_.end()) {
      LOGE("cannot find the init extrinsic, slave id: %d\n", slave_id);
      return;
    }
    Eigen::Matrix4d init_ext = init_extrinsics_[slave_id];
    LOGI("init extrinsic T_ms is: roll = %f, pitch = %f, yaw =  %f, x = %f, "
         "y = %f, z = %f\n",
         TransformUtil::GetRoll(init_ext) / degree_2_radian,
         TransformUtil::GetPitch(init_ext) / degree_2_radian,
         TransformUtil::GetYaw(init_ext) / degree_2_radian, init_ext(0, 3),
         init_ext(1, 3), init_ext(2, 3));
    pcl::PointCloud<pcl::PointXYZI>::Ptr slave_pc_ptr = slave_pc.makeShared();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_Condition(new pcl::PointCloud<pcl::PointXYZI>);   
    // pcl::PointCloud<pcl::PointXYZI>::Ptr noCar = slave_pc.makeShared();  
    PlaneParam slave_gplane;
    pcl::PointCloud<pcl::PointXYZI>::Ptr slave_gcloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr slave_ngcloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    
    // pcl::io::savePCDFileBinary("before_filterd.pcd", *slave_pc_ptr);
    // earse the points close to LiDAR
    if(slave_id)
    {

      pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZI>());
      range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
        pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, -1)));  //GT表示大于等于
      range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
        pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, 1)));  //LT表示小于等于
      range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
        pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, -1.0)));  //GT表示大于等于
      range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
        pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, 1)));  //LT表示小于等于
      range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
        pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::GT, -1)));  //GT表示大于等于
      range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
        pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::LT, 1)));  //LT表示小于等于


      pcl::ConditionalRemoval<pcl::PointXYZI> condition;
      condition.setCondition(range_condition);
      condition.setInputCloud(slave_pc_ptr);                   //输入点云
      condition.setKeepOrganized(false);
      condition.filter(*cloud_after_Condition);



      pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
      pcl::PointXYZI searchPoint;
      int K = 1;
      std::vector<int> pointIdxNKNSearch(K);      //存储查询点近邻索引
      std::vector<float> pointNKNSquaredDistance(K); //存储近邻点对应距离平方
      std::vector<pcl::PointXYZI> DeleteData;
      int num = 0;
      for (auto iter = cloud_after_Condition->begin(); iter != cloud_after_Condition->end(); iter++)
      {
          searchPoint.x = iter->x;
          searchPoint.y = iter->y;
          searchPoint.z = iter->z;
          kdtree.setInputCloud(slave_pc_ptr);    //在cloudB中找到对应点后，在cloudB中直接删除该点
          num = kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
          if (num > 0)
          {
              if (sqrt(pointNKNSquaredDistance[0])<eps)
              {
                  auto iterB = slave_pc_ptr->begin() + pointIdxNKNSearch[0];
                  slave_pc_ptr->erase(iterB);
                  DeleteData.push_back(searchPoint);
                  if (slave_pc_ptr->size()==0)
                  {
                      break;
                  }
                  searchPoint.x = 0;
                  searchPoint.y = 0;
                  searchPoint.z = 0;
                  num = 0;
                  pointIdxNKNSearch.clear();
                  pointNKNSquaredDistance.clear();
              }
          }
          
      }


      

    }


    ret = GroundPlaneExtraction(slave_pc_ptr, slave_gcloud, slave_ngcloud,
                                slave_gplane);
    std::cout<<slave_id<<std::endl;

    if (!ret) {
      LOGE("slave %d lidar ground fitting failed.\n", slave_id);
      continue;
    }
    registrator_->SetSourceCloud(slave_gcloud, slave_ngcloud, slave_pc_ptr);

    //ground normal direction
    //1、suppose a point in the ground plane
    // Eigen::Vector3f ground_point(1,1,(slave_gplane.normal(0)+slave_gplane.normal(1)+slave_gplane.intercept)/(-slave_gplane.normal(2)));
    Eigen::Vector3f ground_point(0,0,(slave_gplane.intercept)/(-slave_gplane.normal(2)));
    // Eigen::Vector3f samplePoint;
    Eigen::Vector3f point2plane_vector;
    int Ontheground=0;
    int Undertheground=0;
    for (auto iter = slave_ngcloud->begin(); iter < slave_ngcloud->end()-100; iter+=100)
    {
      Eigen::Vector3f samplePoint(iter->x,iter->y,iter->z);
      point2plane_vector = samplePoint - ground_point;
      if((point2plane_vector(0)*slave_gplane.normal(0)+
          point2plane_vector(1)*slave_gplane.normal(1)+
          point2plane_vector(2)*slave_gplane.normal(2)) >= 0){
            Ontheground ++;
          }
      else{
            Undertheground++;
      }
        
    }    
    std::cout<<"总的点数为"<<slave_ngcloud->size()<<std::endl;
    std::cout<<"面上的点数为："<<Ontheground<<std::endl;
    std::cout<<"面下的点数为："<<Undertheground<<std::endl;
    // if (Ontheground<Undertheground)
    // {
    //   slave_gplane.normal = -slave_gplane.normal;
    //   slave_gplane.intercept = -slave_gplane.intercept;
    // }

    // ground plane align
    Eigen::Vector3d rot_axis2 = slave_gplane.normal.cross(master_gplane.normal);
    rot_axis2.normalize();
    double alpha2 = std::acos(slave_gplane.normal.dot(master_gplane.normal));
    Eigen::Matrix3d R_ms;
    R_ms = Eigen::AngleAxisd(alpha2, rot_axis2);
    Eigen::Vector3d slave_intcpt_local(
        0, 0, -slave_gplane.intercept / slave_gplane.normal(2));
    Eigen::Vector3d slave_intcpt_master = R_ms * slave_intcpt_local;
    Eigen::Vector3d t_ms(0, 0, t_mp(2) - slave_intcpt_master(2));
    Eigen::Matrix4d T_ms = TransformUtil::GetMatrix(t_ms, R_ms);
    double z_error = std::fabs(t_ms(2) - init_ext(2, 3));
    std::cout<< "z_error " << z_error <<std::endl;
    // if (z_error > 2) 
    if (Ontheground<Undertheground)
    {
      std::cout<<"方向反转"<<std::endl;
      // maybe the direction is diffetent
      slave_gplane.normal = -slave_gplane.normal;
      slave_gplane.intercept = -slave_gplane.intercept;
      rot_axis2 = slave_gplane.normal.cross(master_gplane.normal);
      rot_axis2.normalize();
      alpha2 = std::acos(slave_gplane.normal.dot(master_gplane.normal));
      R_ms = Eigen::AngleAxisd(alpha2, rot_axis2);
      slave_intcpt_local = Eigen::Vector3d(
          0, 0, -slave_gplane.intercept / slave_gplane.normal(2));
      slave_intcpt_master = R_ms * slave_intcpt_local;
      t_ms = Eigen::Vector3d(0, 0, t_mp(2) - slave_intcpt_master(2));
      T_ms = TransformUtil::GetMatrix(t_ms, R_ms);
      z_error = std::fabs(t_ms(2) - init_ext(2, 3));
      std::cout<< "z_error " << z_error <<std::endl;
      // if (z_error > 2) {
      //   LOGE(
      //       "slave %d ground fitting failed, error: z-diff error is too big.\n",
      //       slave_id);
      //   continue;
      // }
    }

    double roll = TransformUtil::GetRoll(T_ms);
    double pitch = TransformUtil::GetPitch(T_ms);
    double z = TransformUtil::GetZ(T_ms);
    double init_x = TransformUtil::GetX(init_ext);
    double init_y = TransformUtil::GetY(init_ext);
    double init_yaw = TransformUtil::GetYaw(init_ext);
    Eigen::Matrix4d init_guess =
        TransformUtil::GetMatrix(init_x, init_y, z, roll, pitch, init_yaw);
    LOGI("ground plane param align, roll = %f, pitch = %f, z = %f\n", roll,
         pitch, z);


    // registration
    double refined_yaw = 0;
    registrator_->RegistrationByICP(init_guess, &refined_yaw);
    double init_roll = TransformUtil::GetRoll(init_guess);
    double init_pitch = TransformUtil::GetPitch(init_guess);
    Eigen::Matrix4d yaw_opt_resust = TransformUtil::GetMatrix(
        TransformUtil::GetTranslation(init_guess),
        TransformUtil::GetRotation(init_roll, init_pitch, refined_yaw));
    Eigen::Matrix4d final_opt_result;
    registrator_->RegistrationByICP2(yaw_opt_resust, final_opt_result);
    Eigen::Matrix4d temp = final_opt_result;
    registrator_->RegistrationByVoxelOccupancy(temp, final_opt_result);

    refined_extrinsics_.insert(std::make_pair(slave_id, final_opt_result));
  }
}

bool Calibrator::GroundPlaneExtraction(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud, PlaneParam &plane) {

  //filter the input point cloud 
  pcl::VoxelGrid<pcl::PointXYZI> sor;  //创建滤波对象
  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZI>);
  // sor.setInputCloud(in_cloud);            //设置需要过滤的点云给滤波对象
  // sor.setLeafSize(0.1f, 0.1f, 0.1f);  //设置滤波时创建的体素体积为1cm的立方体
  // sor.filter(*cloud_filtered);           //执行滤波处理，存储输出

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.2);
  seg.setInputCloud(in_cloud);  //cloud_filtered
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return false;
  }
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(in_cloud);  //cloud_filtered
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

std::map<int32_t, Eigen::Matrix4d> Calibrator::GetFinalTransformation() {
  return refined_extrinsics_;
}