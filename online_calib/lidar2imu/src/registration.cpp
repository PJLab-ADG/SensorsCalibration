#include "registration.hpp"
#include "lidar_factors.hpp"
#include "utils/transform_util.hpp"

// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace autoCalib {
namespace calibration {

Registrator::Registrator()
{
    systemInited_ = false;

    laserCloudCornerLast_.reset(new PointCloudLidar);
    laserCloudSurfLast_.reset(new PointCloudLidar);
    laserCloudFullRes_.reset(new PointCloudLidar);
    kdtreeCornerLast_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    kdtreeSurfLast_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);

    // para_q_[0] = 0;
    // para_q_[1] = 0;
    // para_q_[2] = 0;
    // para_q_[3] = 1;
    // para_t_[0] = 0;
    // para_t_[1] = 0;
    // para_t_[2] = 0;
    // para_t_ = {0, 0, 0};
    q_w_curr_ = Eigen::Quaterniond(1, 0, 0, 0);
    t_w_curr_ = Eigen::Vector3d(0, 0, 0);

    // q_last_curr_ = Eigen::Map<Eigen::Quaterniond>(para_q_);
    // t_last_curr_ = Eigen::Map<Eigen::Vector3d>(para_t_);

    corner_correspondence_ = 0;
    plane_correspondence_ = 0;
}

void Registrator::removeClosedPointCloud(PointCloud::Ptr cloud_in,
                                         PointCloud::Ptr cloud_out, 
                                         float thres)
{
    if (cloud_in != cloud_out)
    {
        cloud_out->header = cloud_in->header;
        cloud_out->points.resize(cloud_in->points.size());
    }

    size_t j = 0;
    float thres_square = thres * thres;
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        if (cloud_in->points[i].x * cloud_in->points[i].x + 
            cloud_in->points[i].y * cloud_in->points[i].y + 
            cloud_in->points[i].z * cloud_in->points[i].z < thres_square)
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


void Registrator::getFeatures(PointCloud::Ptr cloud,
                              PointCloudFeatures& pcd_features)
{
    std::vector<int> scanStartInd(n_scans_, 0);
    std::vector<int> scanEndInd(n_scans_, 0);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    removeClosedPointCloud(cloud, cloud, minium_range_);


    int cloudSize = cloud->points.size();
    float startOri = -atan2(cloud->points[0].y, cloud->points[0].x);
    float endOri = -atan2(cloud->points[cloudSize - 1].y,
                          cloud->points[cloudSize - 1].x) +
                   2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = cloudSize;
    PointXYZI point;
    std::vector<PointCloudLidar> laserCloudScans(n_scans_);
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (n_scans_ == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (n_scans_ - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (n_scans_ == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (n_scans_ - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (n_scans_ == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = n_scans_ / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            LOGE("wrong lidar scan number.");
            exit(1);
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        { 
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + scan_period_ * relTime;
        laserCloudScans[scanID].push_back(point); 
    }
    
    cloudSize = count;
    printf("points size %d \n", cloudSize);

    PointCloudLidar::Ptr laserCloud(new PointCloudLidar);
    for (int i = 0; i < n_scans_; i++)
    { 
        scanStartInd[i] = laserCloud->size() + 5;
        // laserCloud->points.insert(laserCloud->points.end(), 
        //                           laserCloudScans[i].points.begin(),
        //                           laserCloudScans[i].points.end())
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->points.size() - 6;
    }


    for (int i = 5; i < cloudSize - 5; i++)
    { 
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }

    TicToc t_pts;

    PointCloudLidar cornerPointsSharp;
    PointCloudLidar cornerPointsLessSharp;
    PointCloudLidar surfPointsFlat;
    PointCloudLidar surfPointsLessFlat;

    float t_q_sort = 0;
    for (int i = 0; i < n_scans_; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        PointCloudLidar::Ptr surfPointsLessFlatScan(new PointCloudLidar);
        for (int j = 0; j < 6; j++)
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k]; 

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {                        
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)
                    {                        
                        cloudLabel[ind] = 1; 
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1; 

                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {

                    cloudLabel[ind] = -1; 
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    { 
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        PointCloudLidar surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointXYZI> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    printf("sort q time %f \n", t_q_sort);
    printf("seperate points time %f \n", t_pts.toc());

    *(pcd_features.corner_sharp_pts) = cornerPointsSharp;
    *(pcd_features.corner_less_sharp_pts) = cornerPointsLessSharp;
    *(pcd_features.surf_flat_pts) = surfPointsFlat;
    *(pcd_features.surf_less_flat_pts) = surfPointsLessFlat;
}

// undistort lidar point
void Registrator::TransformToStart(PointXYZI const *const pi, 
                                   PointXYZI *const po)
{
    //interpolation ratio
    double s;
    if (distortion_)
        s = (pi->intensity - int(pi->intensity)) / scan_period_;
    else
        s = 1.0;
    //s = 1;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr_);
    Eigen::Vector3d t_point_last = s * t_last_curr_;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}


// transform all lidar points to the start of the next frame

void Registrator::TransformToEnd(PointXYZI const *const pi, 
                                 PointXYZI *const po)
{
    // undistort point first
    PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr_.inverse() * (un_point - t_last_curr_);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}


void Registrator::inputFrame(PointCloud::Ptr cloud,
                            //  const PointCloudFeatures& input_features,
                             Eigen::Matrix4d& extrinsic)
{
    // get lidar point cloud features
    PointCloudFeatures input_features;
    getFeatures(cloud, input_features);

    TicToc t_whole;
    if (!systemInited_)
    {
        systemInited_ = true;
        std::cout << "Initialization finished \n";
    }
    else
    {
        int cornerPointsSharpNum = input_features.corner_sharp_pts->points.size();
        int surfPointsFlatNum = input_features.surf_flat_pts->points.size();

        TicToc t_opt;
        for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
        {
            corner_correspondence_ = 0;
            plane_correspondence_ = 0;

            //ceres::LossFunction *loss_function = NULL;
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::LocalParameterization *q_parameterization =
                new ceres::EigenQuaternionParameterization();
            ceres::Problem::Options problem_options;

            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(para_q_, 4, q_parameterization);
            problem.AddParameterBlock(para_t_, 3);

            pcl::PointXYZI pointSel;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            TicToc t_data;
            // find correspondence for corner features
            for (int i = 0; i < cornerPointsSharpNum; ++i)
            {
                TransformToStart(&(input_features.corner_sharp_pts->points[i]), &pointSel);
                kdtreeCornerLast_->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                int closestPointInd = -1, minPointInd2 = -1;
                if (pointSearchSqDis[0] < distance_sq_threshold_)
                {
                    closestPointInd = pointSearchInd[0];
                    int closestPointScanID = int(laserCloudCornerLast_->points[closestPointInd].intensity);

                    double minPointSqDis2 = distance_sq_threshold_;
                    // search in the direction of increasing scan line
                    for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast_->points.size(); ++j)
                    {
                        // if in the same scan line, continue
                        if (int(laserCloudCornerLast_->points[j].intensity) <= closestPointScanID)
                            continue;

                        // if not in nearby scans, end the loop
                        if (int(laserCloudCornerLast_->points[j].intensity) > (closestPointScanID + nearby_scan_))
                            break;

                        double pointSqDis = (laserCloudCornerLast_->points[j].x - pointSel.x) *
                                                (laserCloudCornerLast_->points[j].x - pointSel.x) +
                                            (laserCloudCornerLast_->points[j].y - pointSel.y) *
                                                (laserCloudCornerLast_->points[j].y - pointSel.y) +
                                            (laserCloudCornerLast_->points[j].z - pointSel.z) *
                                                (laserCloudCornerLast_->points[j].z - pointSel.z);

                        if (pointSqDis < minPointSqDis2)
                        {
                            // find nearer point
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                    }

                    // search in the direction of decreasing scan line
                    for (int j = closestPointInd - 1; j >= 0; --j)
                    {
                        // if in the same scan line, continue
                        if (int(laserCloudCornerLast_->points[j].intensity) >= closestPointScanID)
                            continue;

                        // if not in nearby scans, end the loop
                        if (int(laserCloudCornerLast_->points[j].intensity) < (closestPointScanID - nearby_scan_))
                            break;

                        double pointSqDis = (laserCloudCornerLast_->points[j].x - pointSel.x) *
                                                (laserCloudCornerLast_->points[j].x - pointSel.x) +
                                            (laserCloudCornerLast_->points[j].y - pointSel.y) *
                                                (laserCloudCornerLast_->points[j].y - pointSel.y) +
                                            (laserCloudCornerLast_->points[j].z - pointSel.z) *
                                                (laserCloudCornerLast_->points[j].z - pointSel.z);

                        if (pointSqDis < minPointSqDis2)
                        {
                            // find nearer point
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                    }
                }
                if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
                {
                    Eigen::Vector3d curr_point(input_features.corner_sharp_pts->points[i].x,
                                                input_features.corner_sharp_pts->points[i].y,
                                                input_features.corner_sharp_pts->points[i].z);
                    Eigen::Vector3d last_point_a(laserCloudCornerLast_->points[closestPointInd].x,
                                                    laserCloudCornerLast_->points[closestPointInd].y,
                                                    laserCloudCornerLast_->points[closestPointInd].z);
                    Eigen::Vector3d last_point_b(laserCloudCornerLast_->points[minPointInd2].x,
                                                    laserCloudCornerLast_->points[minPointInd2].y,
                                                    laserCloudCornerLast_->points[minPointInd2].z);

                    double s;
                    if (distortion_)
                        s = (input_features.corner_sharp_pts->points[i].intensity - 
                            int(input_features.corner_sharp_pts->points[i].intensity)) 
                            / scan_period_;
                    else
                        s = 1.0;
                    ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                    problem.AddResidualBlock(cost_function, loss_function, para_q_, para_t_);
                    corner_correspondence_++;
                }
            }

            // find correspondence for plane features
            for (int i = 0; i < surfPointsFlatNum; ++i)
            {
                TransformToStart(&(input_features.surf_flat_pts->points[i]), &pointSel);
                kdtreeSurfLast_->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                if (pointSearchSqDis[0] < distance_sq_threshold_)
                {
                    closestPointInd = pointSearchInd[0];

                    // get closest point's scan ID
                    int closestPointScanID = int(laserCloudSurfLast_->points[closestPointInd].intensity);
                    double minPointSqDis2 = distance_sq_threshold_, minPointSqDis3 = distance_sq_threshold_;

                    // search in the direction of increasing scan line
                    for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast_->points.size(); ++j)
                    {
                        // if not in nearby scans, end the loop
                        if (int(laserCloudSurfLast_->points[j].intensity) > (closestPointScanID + nearby_scan_))
                            break;

                        double pointSqDis = (laserCloudSurfLast_->points[j].x - pointSel.x) *
                                                (laserCloudSurfLast_->points[j].x - pointSel.x) +
                                            (laserCloudSurfLast_->points[j].y - pointSel.y) *
                                                (laserCloudSurfLast_->points[j].y - pointSel.y) +
                                            (laserCloudSurfLast_->points[j].z - pointSel.z) *
                                                (laserCloudSurfLast_->points[j].z - pointSel.z);

                        // if in the same or lower scan line
                        if (int(laserCloudSurfLast_->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                        {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                        // if in the higher scan line
                        else if (int(laserCloudSurfLast_->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                        {
                            minPointSqDis3 = pointSqDis;
                            minPointInd3 = j;
                        }
                    }

                    // search in the direction of decreasing scan line
                    for (int j = closestPointInd - 1; j >= 0; --j)
                    {
                        // if not in nearby scans, end the loop
                        if (int(laserCloudSurfLast_->points[j].intensity) < (closestPointScanID - nearby_scan_))
                            break;

                        double pointSqDis = (laserCloudSurfLast_->points[j].x - pointSel.x) *
                                                (laserCloudSurfLast_->points[j].x - pointSel.x) +
                                            (laserCloudSurfLast_->points[j].y - pointSel.y) *
                                                (laserCloudSurfLast_->points[j].y - pointSel.y) +
                                            (laserCloudSurfLast_->points[j].z - pointSel.z) *
                                                (laserCloudSurfLast_->points[j].z - pointSel.z);

                        // if in the same or higher scan line
                        if (int(laserCloudSurfLast_->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                        {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                        else if (int(laserCloudSurfLast_->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                        {
                            // find nearer point
                            minPointSqDis3 = pointSqDis;
                            minPointInd3 = j;
                        }
                    }

                    if (minPointInd2 >= 0 && minPointInd3 >= 0)
                    {

                        Eigen::Vector3d curr_point(input_features.surf_flat_pts->points[i].x,
                                                    input_features.surf_flat_pts->points[i].y,
                                                    input_features.surf_flat_pts->points[i].z);
                        Eigen::Vector3d last_point_a(laserCloudSurfLast_->points[closestPointInd].x,
                                                        laserCloudSurfLast_->points[closestPointInd].y,
                                                        laserCloudSurfLast_->points[closestPointInd].z);
                        Eigen::Vector3d last_point_b(laserCloudSurfLast_->points[minPointInd2].x,
                                                        laserCloudSurfLast_->points[minPointInd2].y,
                                                        laserCloudSurfLast_->points[minPointInd2].z);
                        Eigen::Vector3d last_point_c(laserCloudSurfLast_->points[minPointInd3].x,
                                                        laserCloudSurfLast_->points[minPointInd3].y,
                                                        laserCloudSurfLast_->points[minPointInd3].z);

                        double s;
                        if (distortion_)
                            s = (input_features.surf_flat_pts->points[i].intensity - 
                                int(input_features.surf_flat_pts->points[i].intensity)) / 
                                scan_period_;
                        else
                            s = 1.0;
                        ceres::CostFunction *cost_function = LidarPlaneFactor::Create(
                            curr_point, last_point_a, last_point_b, last_point_c, s);
                        problem.AddResidualBlock(cost_function, loss_function, para_q_, para_t_);
                        plane_correspondence_++;
                    }
                }
            }

            //printf("coner_correspondance %d, plane_correspondence_ %d \n", corner_correspondence_, plane_correspondence_);
            printf("data association time %f ms \n", t_data.toc());

            if ((corner_correspondence_ + plane_correspondence_) < 10)
            {
                printf("less correspondence! *************************************************\n");
            }

            TicToc t_solver;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            printf("solver time %f ms \n", t_solver.toc());
        }
        printf("optimization twice time %f \n", t_opt.toc());

        t_w_curr_ = t_w_curr_ + q_w_curr_ * t_last_curr_;
        q_w_curr_ = q_w_curr_ * q_last_curr_;
    }
// transform corner features and plane features to the scan end point
    if (0)
    {
        int cornerPointsLessSharpNum = input_features.corner_less_sharp_pts->points.size();
        for (int i = 0; i < cornerPointsLessSharpNum; i++)
        {
            TransformToEnd(&input_features.corner_less_sharp_pts->points[i], 
                           &input_features.corner_less_sharp_pts->points[i]);
        }

        int surfPointsLessFlatNum = input_features.surf_less_flat_pts->points.size();
        for (int i = 0; i < surfPointsLessFlatNum; i++)
        {
            TransformToEnd(&input_features.surf_less_flat_pts->points[i], 
                           &input_features.surf_less_flat_pts->points[i]);
        }

        // int laserCloudFullResNum = laserCloudFullRes->points.size();
        // for (int i = 0; i < laserCloudFullResNum; i++)
        // {
        //     TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
        // }
    }

    PointCloudLidar::Ptr laserCloudTemp = input_features.corner_less_sharp_pts;
    input_features.corner_less_sharp_pts = laserCloudCornerLast_;
    laserCloudCornerLast_ = laserCloudTemp;

    laserCloudTemp = input_features.surf_less_flat_pts;
    input_features.surf_less_flat_pts = laserCloudSurfLast_;
    laserCloudSurfLast_ = laserCloudTemp;

    // laserCloudCornerLastNum = laserCloudCornerLast_->points.size();
    // laserCloudSurfLastNum = laserCloudSurfLast_->points.size();

    kdtreeCornerLast_->setInputCloud(laserCloudCornerLast_);
    kdtreeSurfLast_->setInputCloud(laserCloudSurfLast_);

    extrinsic = TransformUtil::GetMatrix(t_last_curr_, q_last_curr_.toRotationMatrix());


    printf("whole laserOdometry time %f ms \n \n", t_whole.toc());
    if(t_whole.toc() > 100)
        LOGW("odometry process over 100ms");

    // frameCount++;
}

} // calibration
} // autoCalib