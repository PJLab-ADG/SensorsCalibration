#include "GroundExtractor.h"


static omp_lock_t rr_lock;

bool GroundExtractor::LPRFitting(const PointCloudPtr in_cloud,
                                 PointCloudPtr g_cloud,
                                 PointCloudPtr ng_cloud,
                                 PlaneParam * plane) {
    // find the lpr plane
    PointCloudPtr sort_cloud(new PointCloud());
    PCUtil::MaxRangeFilter(in_cloud, 50.0, sort_cloud);
    std::sort(
        sort_cloud->points.begin(), sort_cloud->points.end(),
        [&](const PointType& p1, const PointType& p2) { return p1.z < p2.z; });
    for (size_t i = 0; i < sort_cloud->points.size(); i++) {
        size_t next_idx =
            static_cast<size_t>(i + 0.001 * sort_cloud->points.size());
        if (next_idx >= sort_cloud->points.size()) {
            std::cout << "LFR fitting failed, not found ground plane." << std::endl;
            return false;
        }
        // remove the noise points, which not belong to grounds
        if ((sort_cloud->points[next_idx].z - sort_cloud->points[i].z) < 0.2) {
            sort_cloud->points.erase(sort_cloud->points.begin(),
                                     sort_cloud->points.begin() + i);
            break;
        }
    }

    // extract init ground seeds
    double lpr_avg_height = 0;
    size_t lpr_num = static_cast<size_t>(lpr_least_gpoints_rate_ *
                                         sort_cloud->points.size());
    for (size_t i = 0; i < lpr_num; i++) {
        lpr_avg_height += sort_cloud->points[i].z;
    }
    lpr_avg_height /= lpr_num;

    for (size_t i = 0; i < sort_cloud->points.size(); i++) {
        if (sort_cloud->points[i].z <=
            (lpr_avg_height + lpr_least_gpoints_interval_)) {
            g_cloud->points.emplace_back(sort_cloud->points[i]);
        } else {
            break;
        }
    }

    // ransac fitting iteratively
    int iter_cnt = 0;
    PlaneParam last_pp(Eigen::Vector3d::Zero(), 0);
    for (; iter_cnt < lpr_max_iters_; iter_cnt++) {
        // fitting plane
        FittingPlaneMesh(g_cloud, plane);

        g_cloud->points.clear();
        ng_cloud->points.clear();
        // split ground and non-ground points
        for (size_t j = 0; j < in_cloud->points.size(); j++) {
            PointType point = in_cloud->points[j];
            double point_to_plane_dis = std::fabs(
                plane->normal(0) * point.x + plane->normal(1) * point.y +
                plane->normal(2) * point.z + plane->intercept);
            if (point_to_plane_dis <= lpr_fit_dist_thre_) {
                g_cloud->points.emplace_back(point);
            } else {
                ng_cloud->points.emplace_back(point);
            }
        }

        // convergence check
        Eigen::Vector3d dlt_norm = plane->normal - last_pp.normal;
        double dlt_intcpt = plane->intercept - last_pp.intercept;
        if (dlt_norm.norm() < 0.001 && dlt_intcpt < 0.01 &&
            iter_cnt > lpr_max_iters_ / 2)
            break;

        last_pp = *plane;
    }

    // normal check, must point to up
    int neg = plane->normal(2) < 0 ? -1 : 1;
    plane->normal(0) *= neg;
    plane->normal(1) *= neg;
    plane->normal(2) *= neg;
    plane->intercept *= neg;
    return true;
}

bool GroundExtractor::RandomRansacFitting(const PointCloudPtr in_cloud,
                                          PointCloudPtr g_cloud,
                                          PointCloudPtr ng_cloud,
                                          PlaneParam* plane) {
    size_t range = in_cloud->points.size();

    // random ransac fitting iteratively
    PlaneParam best_plane;
    int max_inlier_points = 0;
    int gpoints_thre = static_cast<int>(rr_gpoints_rate_ * range);

    for (int i = 0; i < rr_iter_times_; i++)
    {
        for (int j = 0; j < rr_max_rand_iters_; j++) {
            PointType rand_p1, rand_p2, rand_p3;
            rand_p1 = in_cloud->points[RandIndex(range)];
            rand_p2 = in_cloud->points[RandIndex(range)];
            rand_p3 = in_cloud->points[RandIndex(range)];

            // area check
            double area;
            if (!CalArea(rand_p1, rand_p2, rand_p3, &area) || area < rr_min_area_thre_) {
                j--;
                continue;
            }

            PointCloudPtr sample_cloud(new PointCloud());
            sample_cloud->points.emplace_back(rand_p1);
            sample_cloud->points.emplace_back(rand_p2);
            sample_cloud->points.emplace_back(rand_p3);

            // fitting plane using three random points
            PlaneParam pp;
            FittingPlane(sample_cloud, &pp);

            // (TODO:zm) check the points below ground for verification
            int inlier_points = 0;
            for (size_t j = 0; j < range; j++) {
                PointType point = in_cloud->points[j];
                double point_to_plane_dis =
                    std::fabs(pp.normal(0) * point.x + pp.normal(1) * point.y +
                              pp.normal(2) * point.z + pp.intercept);
                if (point_to_plane_dis <= rr_fit_dist_thre_) {
                    inlier_points++;
                }
            }
            omp_set_lock(&rr_lock);
            if (inlier_points > max_inlier_points) {
                best_plane = pp;
                max_inlier_points = inlier_points;
            }
            omp_unset_lock(&rr_lock);
        }

        // random search to refine parameter
        RandomSearchPlane(in_cloud, best_plane, max_inlier_points, 0.01, 0.01, 0.01, 0.1, 300);
        RandomSearchPlane(in_cloud, best_plane, max_inlier_points, 0.002, 0.002, 0.002, 0.02, 100);

        if (max_inlier_points > gpoints_thre)
            break;
    }

     // refine plane param
    for (size_t j = 0; j < in_cloud->points.size(); j++) {
        PointType point = in_cloud->points[j];
        double point_to_plane_dis = std::fabs(
            best_plane.normal(0) * point.x + best_plane.normal(1) * point.y +
            best_plane.normal(2) * point.z + best_plane.intercept);
        if (point_to_plane_dis <= rr_fit_dist_thre_) {
            g_cloud->points.emplace_back(point);
        }
    }
    FittingPlaneMesh(g_cloud, &best_plane);

    // re-get ground and non-ground points
    g_cloud->points.clear();
    ng_cloud->points.clear();
    for (size_t j = 0; j < in_cloud->points.size(); j++) {
        PointType point = in_cloud->points[j];
        double point_to_plane_dis = std::fabs(
            best_plane.normal(0) * point.x + best_plane.normal(1) * point.y +
            best_plane.normal(2) * point.z + best_plane.intercept);
        if (point_to_plane_dis <= rr_fit_dist_thre_) {
            g_cloud->points.emplace_back(point);
        } else {
            ng_cloud->points.emplace_back(point);
        }
    }

    // normal check, must point to up
    int neg = best_plane.normal(2) < 0 ? -1 : 1;
    plane->normal(0) = neg * best_plane.normal(0);
    plane->normal(1) = neg * best_plane.normal(1);
    plane->normal(2) = neg * best_plane.normal(2);
    plane->intercept = neg * best_plane.intercept;
    return true;
}

bool GroundExtractor::RandomSearchPlane(const PointCloudPtr in_cloud, 
                                        PlaneParam &best_plane,
                                        int & max_inlier_points,
                                        double n1_scope, double n2_scope, 
                                        double n3_scope, double i_scope,
                                        int iteration_times)
{
    std::default_random_engine generator((clock() - time(0)) /
                                       (double)CLOCKS_PER_SEC);
    std::uniform_real_distribution<double> distribution_n1(-n1_scope, n1_scope);
    std::uniform_real_distribution<double> distribution_n2(-n2_scope, n2_scope);
    std::uniform_real_distribution<double> distribution_n3(-n3_scope, n3_scope);
    std::uniform_real_distribution<double> distribution_i(-i_scope, i_scope);

    PlaneParam tmp_plane, final_plane;
    final_plane = best_plane;
    for (int i = 0; i < iteration_times; i++)
    {
        tmp_plane.normal(0) = best_plane.normal(0) + distribution_n1(generator);
        tmp_plane.normal(1) = best_plane.normal(1) + distribution_n2(generator);
        tmp_plane.normal(2) = best_plane.normal(2) + distribution_n3(generator);
        tmp_plane.normal /= tmp_plane.normal.norm();
        tmp_plane.intercept = best_plane.intercept + distribution_i(generator);
        int inlier_points = 0;
        for (size_t j = 0; j < in_cloud->points.size(); j++) {
            PointType point = in_cloud->points[j];
            double point_to_plane_dis =
                std::fabs(tmp_plane.normal(0) * point.x + tmp_plane.normal(1) * point.y +
                            tmp_plane.normal(2) * point.z + tmp_plane.intercept);
            if (point_to_plane_dis <= rr_fit_dist_thre_) {
                inlier_points++;
            }
        }
        if (inlier_points > max_inlier_points)
        {
            final_plane = tmp_plane;
            max_inlier_points = inlier_points;
        }
        
    }
    best_plane = final_plane;
    return true;
}

size_t GroundExtractor::RandIndex(size_t range) {
    static std::default_random_engine e;
    static std::uniform_int_distribution<size_t> u(0, range);
    return u(e);
}

bool GroundExtractor::CalArea(const PointType& p1,
                              const PointType& p2,
                              const PointType& p3,
                              double* area) {
    double side[3];  // the sides of triangle
    side[0] = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) +
                        std::pow(p1.z - p2.z, 2));
    side[1] = std::sqrt(std::pow(p1.x - p3.x, 2) + std::pow(p1.y - p3.y, 2) +
                        std::pow(p1.z - p3.z, 2));
    side[2] = std::sqrt(std::pow(p3.x - p2.x, 2) + std::pow(p3.y - p2.y, 2) +
                        std::pow(p3.z - p2.z, 2));

    // not triangle
    if (side[0] + side[1] <= side[2] || side[0] + side[2] <= side[1] ||
        side[1] + side[2] <= side[0])
        return false;

    // Heron's formula, calculate the area
    double semi_perimeter = (side[0] + side[1] + side[2]) / 2;
    *area = std::sqrt(semi_perimeter * (semi_perimeter - side[0]) *
                      (semi_perimeter - side[1]) * (semi_perimeter - side[2]));

    return true;
}

bool GroundExtractor::FittingPlane(PointCloudPtr in_cloud,
                                   PlaneParam* plane) {
    Eigen::Vector3d point(in_cloud->points[0].x, in_cloud->points[0].y,
                          in_cloud->points[0].z);
    Eigen::Vector3d vec1(point(0) - in_cloud->points[1].x,
                         point(1) - in_cloud->points[1].y,
                         point(2) - in_cloud->points[1].z);
    Eigen::Vector3d vec2(point(0) - in_cloud->points[2].x,
                         point(1) - in_cloud->points[2].y,
                         point(2) - in_cloud->points[2].z);
    Eigen::Vector3d norm = vec1.cross(vec2);
    plane->normal = norm / norm.norm();
    // normal.T*[x,y,z] = -d
    plane->intercept = -(plane->normal.transpose() * point)(0, 0);
    return true;
}

bool GroundExtractor::FittingPlaneMesh(const PointCloudPtr in_cloud,
                                       PlaneParam *plane) {

    // calculate the mean and cov
    std::vector<Eigen::Vector3d> points;
    Eigen::Vector3d mean(0.0, 0.0, 0.0);
    for (size_t j = 0; j < in_cloud->points.size(); j++) {
        PointType point = in_cloud->points[j];
        Eigen::Vector3d temp(point.x, point.y, point.z);
        mean += temp;
        points.emplace_back(temp);
    }
    mean = mean / in_cloud->points.size();
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (size_t j = 0; j < points.size(); j++) {
        Eigen::Vector3d temp = points[j] - mean;
        cov = cov + temp * temp.transpose();
    }
    // svd
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        cov, Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    plane->normal = (svd.matrixU().col(2));
    // normal.T*[x,y,z] = -d
    plane->intercept = -(plane->normal.transpose() * mean)(0, 0);

    return true;
}