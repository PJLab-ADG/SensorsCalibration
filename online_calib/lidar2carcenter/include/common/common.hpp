/*
 * Copyright (C) 2020-2020 by SenseTime Group Limited. All rights reserved.
 * Liang Yu<liangyu@sensetime.com>
 */
#ifndef LIDAR2CAR_INCLUDE_COMMON_HPP_
#define LIDAR2CAR_INCLUDE_COMMON_HPP_

#include <stdint.h>
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <chrono>  // NOLINT
#include <pcl/io/pcd_io.h>


namespace autoCalib {
namespace calibration {

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudLidar;

template <size_t N>
using UnitCol = std::array<float, N>;

using TMatrix = UnitCol<3>;
using RMatrix = std::array<UnitCol<3>, 3>;

struct EulerAngles {
    float roll;
    float pitch;
    float yaw;
};

/** A structure to extrinsic transform matrix */
struct Extrinsic {
    RMatrix rotation;     //!< Rotation matrix 3x3
    TMatrix translation;  //!< Translation matrix 3x3
    int32_t device_id;  //!< device id. negative id are invalid. master id is 0
    Extrinsic() : device_id(-1) {}
};

/** A structure to extrinsic transform matrices */
struct ExtrinsicGroup {
    std::vector<Extrinsic> extrinsics;  // extrinsic transform matrices' vector
};

class TicToc {
 public:
    TicToc() { tic(); }

    void tic() { start = std::chrono::system_clock::now(); }

    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000; /* ms */
    }

 private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

}  // namespace calibration
}  // namespace autoCalib

#endif  // LIDAR2CAR_INCLUDE_COMMON_HPP_
