#ifndef RPZ_CALIB_PC_UTIL_H_
#define RPZ_CALIB_PC_UTIL_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <utility>

#include "util.h"

class PCUtil {
 public:
    PCUtil() = default;
    ~PCUtil() = default;

    static bool MinRangeFilter(const PointCloudPtr in_cloud, double range,
                               PointCloudPtr out_cloud) {
        if (out_cloud == nullptr) return false;
        out_cloud->points.clear();
        for (size_t i = 0; i < in_cloud->points.size(); i++) {
            PointType p = in_cloud->points[i];
            double dist = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (dist > range) out_cloud->points.emplace_back(p);
        }
        return true;
    }

    static bool MaxRangeFilter(const PointCloudPtr in_cloud, double range,
                               PointCloudPtr out_cloud) {
        out_cloud->points.clear();
        for (size_t i = 0; i < in_cloud->points.size(); i++) {
            PointType p = in_cloud->points[i];
            double dist = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (dist < range) out_cloud->points.emplace_back(p);
        }
        return true;
    }

 private:
};

#endif  // RPZ_CALIB_PC_UTIL_H_