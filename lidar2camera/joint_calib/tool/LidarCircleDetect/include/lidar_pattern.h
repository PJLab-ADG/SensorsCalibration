#include <eigen3/Eigen/Dense>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LidarDetector {

public:
  LidarDetector(){};
  ~LidarDetector(){};

  void STLidarDetection(std::string pcds_dir);
};
