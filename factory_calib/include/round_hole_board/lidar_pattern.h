#define PCL_NO_PRECOMPILE
#include <eigen3/Eigen/Dense>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidarcalib {

struct Point3D {
  Point3D(float a, float b, float c) : x(a), y(b), z(c) {}
  float x, y, z;
};

class LidarDetector {
private:
  double passthrough_radius_left_min_ = 5;
  double passthrough_radius_left_max_ = 6;
  double passthrough_radius_right_min_ = 4;
  double passthrough_radius_right_max_ = 5;
  double angle_threshold_ = 0.55;

  double plane_threshold_ = 0.05;
  double gradient_threshold_ = 2;
  double plane_distance_inliers_ = 0.1;
  Eigen::Vector3f axis_ = {0, 1, 0};
  int rings_count_ = 64;

private:
  std::vector<Point3D> lidar_points_;

public:
  LidarDetector();
  ~LidarDetector();

  bool LidarCircleCenterDetection(std::string pcds_dir, bool first, bool front);
  std::vector<Point3D> GetLidarDetectPoints();
};

} // lidarcalib
