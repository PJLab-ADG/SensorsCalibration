#define PCL_NO_PRECOMPILE
#include "lidar_pattern.h"
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cout << "Usage: ./run_lidardetection <pcd_dir>"
                 "\nexample:\n\t"
                 "./bin/run_lidardetection data/ "
              << std::endl;
    return 0;
  }
  bool front = true;

  LidarDetector lidar_detector;
  lidar_detector.STLidarDetection(argv[1]);

  return 0;
}
