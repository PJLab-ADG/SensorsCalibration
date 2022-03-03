#include <eigen3/Eigen/Dense>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidarcalib {

class LidarDetector {

public:
  LidarDetector(){};
  ~LidarDetector(){};

  void LidarDetection(std::string pcds_dir);
};

} // lidarcalib
