/*
  velo2cam_utils: Helper functions
*/

#ifndef velo2cam_utils_H
#define velo2cam_utils_H
#define PCL_NO_PRECOMPILE

#define DEBUG 0

#include "math.h"
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#define TARGET_NUM_CIRCLES 4
#define TARGET_RADIUS 0.12
#define GEOMETRY_TOLERANCE 0.06

using namespace std;

namespace Velodyne {
struct Point {
  PCL_ADD_POINT4D;    // quad-word XYZ
  float intensity;    ///< laser intensity reading
  std::uint16_t ring; ///< laser ring number
  float range;
  float theta;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

void addRange(pcl::PointCloud<Velodyne::Point> &pc, bool first) {
  for (pcl::PointCloud<Point>::iterator pt = pc.points.begin();
       pt < pc.points.end(); pt++) {
    if (pt->x != pt->x)
      continue;
    // pt->intensity = sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);
    // pt->range = sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);
    pt->intensity = sqrt(pt->x * pt->x);
    // if(first)
    //   pt->intensity = sqrt(pt->x * pt->x);
    // else
    //   pt->intensity = sqrt(pt->y * pt->y);
    // pt->range = pt->x;
  }
}

void addTheta(pcl::PointCloud<Velodyne::Point> &pc) {
  for (pcl::PointCloud<Point>::iterator pt = pc.points.begin();
       pt < pc.points.end(); pt++) {
    if (pt->x != pt->x)
      continue;
    float theta = atan(float(pt->y) / pt->x);
    if (pt->x >= 0)
      theta = 3.1415926 + theta;
    else if (pt->y > 0)
      theta = 2 * 3.1415926 + theta;
    pt->theta = theta;
  }
}

vector<vector<float>>
generateLidarImage(pcl::PointCloud<Velodyne::Point> &pc, int rings_count,
                   vector<vector<vector<int>>> &indexMap) {
  vector<vector<float>> output(rings_count, vector<float>(1800, 0.0));
  int max_ring = 0, max_theta = 0;
  int index = 0;
  for (pcl::PointCloud<Point>::iterator pt = pc.points.begin();
       pt < pc.points.end(); pt++) {
    output[pt->ring][int(pt->theta / (0.2 * 3.1415926 / 180))] = pt->range;
    indexMap[pt->ring][int(pt->theta / (0.2 * 3.1415926 / 180))].emplace_back(
        index);
    index++;
  }
  return output;
}

void roifilter(pcl::PointCloud<Velodyne::Point> &pc, bool front) {
  pcl::PointCloud<Velodyne::Point>::Ptr rt_pc(
      new pcl::PointCloud<Velodyne::Point>);
  for (pcl::PointCloud<Point>::iterator pt = pc.points.begin();
       pt < pc.points.end(); pt++) {
    // pt->range = sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);
    if (pt->x != pt->x)
      continue;
    if (front) {
      if (pt->x < 0)
        continue;
    } else {
      if (pt->x > 0)
        continue;
    }
    float theta = atan(float(pt->y) / pt->x);
    // GQ
    // if(theta<-0.3 || theta>0.3) continue;

    // BDJ fov120
    if (theta < -0.8 || theta > 0.8)
      continue;

    // if(theta>-0.3 || theta<-1.2) continue;
    rt_pc->points.push_back(*pt);
  }
  pc = *rt_pc;
}

bool GreaterSort(Point *a, Point *b) { return (a->y > b->y); }

vector<vector<Point *>> getRings(pcl::PointCloud<Velodyne::Point> &pc,
                                 int rings_count) {
  vector<vector<Point *>> rings(rings_count);

  for (pcl::PointCloud<Point>::iterator pt = pc.points.begin();
       pt < pc.points.end(); pt++) {
    rings[pt->ring].push_back(&(*pt));
  }
  // sort by y
  for (vector<vector<Velodyne::Point *>>::iterator ring = rings.begin();
       ring < rings.end(); ++ring) {

    std::sort(ring->begin(), ring->end(), GreaterSort);
  }
  return rings;
}

// all intensities to range min-max
void normalizeIntensity(pcl::PointCloud<Point> &pc, float minv, float maxv) {
  float min_found = INFINITY;
  float max_found = -INFINITY;

  for (pcl::PointCloud<Point>::iterator pt = pc.points.begin();
       pt < pc.points.end(); pt++) {
    max_found = max(max_found, pt->intensity);
    min_found = min(min_found, pt->intensity);
  }

  for (pcl::PointCloud<Point>::iterator pt = pc.points.begin();
       pt < pc.points.end(); pt++) {
    pt->intensity =
        (pt->intensity - min_found) / (max_found - min_found) * (maxv - minv) +
        minv;
  }
}
} // namespace Velodyne

POINT_CLOUD_REGISTER_POINT_STRUCT(Velodyne::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity,
                                      intensity)(std::uint16_t, ring, ring));

void sortPatternCenters(pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                        vector<pcl::PointXYZ> &v) {
  // 0 -- 1
  // |    |
  // 3 -- 2

  if (v.empty()) {
    v.resize(4);
  }

  // Transform points to polar coordinates
  pcl::PointCloud<pcl::PointXYZ>::Ptr spherical_centers(
      new pcl::PointCloud<pcl::PointXYZ>());
  int top_pt = 0;
  int index = 0; // Auxiliar index to be used inside loop
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = pc->points.begin();
       pt < pc->points.end(); pt++, index++) {
    pcl::PointXYZ spherical_center;
    spherical_center.x = atan2(pt->y, pt->x); // Horizontal
    spherical_center.y =
        atan2(sqrt(pt->x * pt->x + pt->y * pt->y), pt->z); // Vertical
    spherical_center.z =
        sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z); // Range
    spherical_centers->push_back(spherical_center);

    if (spherical_center.y < spherical_centers->points[top_pt].y) {
      top_pt = index;
    }
  }

  // Compute distances from top-most center to rest of points
  vector<double> distances;
  for (int i = 0; i < 4; i++) {
    pcl::PointXYZ pt = pc->points[i];
    pcl::PointXYZ upper_pt = pc->points[top_pt];
    distances.push_back(sqrt(pow(pt.x - upper_pt.x, 2) +
                             pow(pt.y - upper_pt.y, 2) +
                             pow(pt.z - upper_pt.z, 2)));
  }

  // Get indices of closest and furthest points
  int min_dist = (top_pt + 1) % 4, max_dist = top_pt;
  for (int i = 0; i < 4; i++) {
    if (i == top_pt)
      continue;
    if (distances[i] > distances[max_dist]) {
      max_dist = i;
    }
    if (distances[i] < distances[min_dist]) {
      min_dist = i;
    }
  }

  // Second highest point shoud be the one whose distance is the median value
  int top_pt2 = 6 - (top_pt + max_dist + min_dist); // 0 + 1 + 2 + 3 = 6

  // Order upper row centers
  int lefttop_pt = top_pt;
  int righttop_pt = top_pt2;

  if (spherical_centers->points[top_pt].x <
      spherical_centers->points[top_pt2].x) {
    int aux = lefttop_pt;
    lefttop_pt = righttop_pt;
    righttop_pt = aux;
  }

  // Swap indices if target is located in the pi,-pi discontinuity
  double angle_diff = spherical_centers->points[lefttop_pt].x -
                      spherical_centers->points[righttop_pt].x;
  if (angle_diff > M_PI - spherical_centers->points[lefttop_pt].x) {
    int aux = lefttop_pt;
    lefttop_pt = righttop_pt;
    righttop_pt = aux;
  }

  // Define bottom row centers using lefttop == top_pt as hypothesis
  int leftbottom_pt = min_dist;
  int rightbottom_pt = max_dist;

  // If lefttop != top_pt, swap indices
  if (righttop_pt == top_pt) {
    leftbottom_pt = max_dist;
    rightbottom_pt = min_dist;
  }

  // Fill vector with sorted centers
  v[0] = pc->points[lefttop_pt];     // lt
  v[1] = pc->points[righttop_pt];    // rt
  v[2] = pc->points[rightbottom_pt]; // rb
  v[3] = pc->points[leftbottom_pt];  // lb
}

void colourCenters(const std::vector<pcl::PointXYZ> pc,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr coloured) {
  float intensity = 0;
  for (int i = 0; i < 4; i++) {
    pcl::PointXYZI cc;
    cc.x = pc[i].x;
    cc.y = pc[i].y;
    cc.z = pc[i].z;

    cc.intensity = intensity;
    coloured->push_back(cc);
    intensity += 0.3;
  }
}

void getCenterClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud,
                       double cluster_tolerance = 0.10,
                       int min_cluster_size = 15, int max_cluster_size = 200,
                       bool verbosity = true) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster;
  euclidean_cluster.setClusterTolerance(cluster_tolerance);
  euclidean_cluster.setMinClusterSize(min_cluster_size);
  euclidean_cluster.setMaxClusterSize(max_cluster_size);
  euclidean_cluster.setSearchMethod(tree);
  euclidean_cluster.setInputCloud(cloud_in);
  euclidean_cluster.extract(cluster_indices);

  if (DEBUG && verbosity)
    cout << cluster_indices.size() << " clusters found from "
         << cloud_in->points.size() << " points in cloud" << endl;

  for (std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin();
       it < cluster_indices.end(); it++) {
    float accx = 0., accy = 0., accz = 0.;
    for (vector<int>::iterator it2 = it->indices.begin();
         it2 < it->indices.end(); it2++) {
      accx += cloud_in->at(*it2).x;
      accy += cloud_in->at(*it2).y;
      accz += cloud_in->at(*it2).z;
    }
    // Compute and add center to clouds
    pcl::PointXYZ center;
    center.x = accx / it->indices.size();
    center.y = accy / it->indices.size();
    center.z = accz / it->indices.size();
    centers_cloud->push_back(center);
  }
}

Eigen::Affine3f getRotationMatrix(Eigen::Vector3f source,
                                  Eigen::Vector3f target) {
  Eigen::Vector3f rotation_vector = target.cross(source);
  rotation_vector.normalize();
  double theta = acos(source[2] / sqrt(pow(source[0], 2) + pow(source[1], 2) +
                                       pow(source[2], 2)));

  if (DEBUG)
    cout << "Rot. vector: " << rotation_vector << " / Angle: " << theta << endl;

  Eigen::Matrix3f rotation =
      Eigen::AngleAxis<float>(theta, rotation_vector) * Eigen::Scaling(1.0f);
  Eigen::Affine3f rot(rotation);
  return rot;
}

class Square {
private:
  pcl::PointXYZ _center;
  std::vector<pcl::PointXYZ> _candidates;
  float _target_width, _target_height, _target_diagonal;

public:
  Square(std::vector<pcl::PointXYZ> candidates, float width, float height) {
    _candidates = candidates;
    _target_width = width;
    _target_height = height;
    _target_diagonal = sqrt(pow(width, 2) + pow(height, 2));

    // Compute candidates centroid
    for (int i = 0; i < candidates.size(); ++i) {
      _center.x += candidates[i].x;
      _center.y += candidates[i].y;
      _center.z += candidates[i].z;
    }

    _center.x /= candidates.size();
    _center.y /= candidates.size();
    _center.z /= candidates.size();
  }

  float distance(pcl::PointXYZ pt1, pcl::PointXYZ pt2) {
    return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) +
                pow(pt1.z - pt2.z, 2));
  }

  float perimeter() { // TODO: It is assumed that _candidates are ordered, it
                      // shouldn't
    float perimeter = 0;
    for (int i = 0; i < 4; ++i) {
      perimeter += distance(_candidates[i], _candidates[(i + 1) % 4]);
    }
    return perimeter;
  }

  pcl::PointXYZ at(int i) {
    assert(0 <= i && i < 4);
    return _candidates[i];
  }

  bool is_valid() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidates_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    // Check if candidates are at 5% of target's diagonal/2 to their centroid
    for (int i = 0; i < _candidates.size(); ++i) {
      candidates_cloud->push_back(_candidates[i]);
      float d = distance(_center, _candidates[i]);
      if (fabs(d - _target_diagonal / 2.) / (_target_diagonal / 2.) >
          GEOMETRY_TOLERANCE) {
        return false;
      }
    }
    // Check perimeter?
    std::vector<pcl::PointXYZ> sorted_centers;
    sortPatternCenters(candidates_cloud, sorted_centers);
    float perimeter = 0;
    for (int i = 0; i < sorted_centers.size(); ++i) {
      float current_distance = distance(
          sorted_centers[i], sorted_centers[(i + 1) % sorted_centers.size()]);
      if (i % 2) {
        if (fabs(current_distance - _target_height) / _target_height >
            GEOMETRY_TOLERANCE) {
          return false;
        }
      } else {
        if (fabs(current_distance - _target_width) / _target_width >
            GEOMETRY_TOLERANCE) {
          return false;
        }
      }
      perimeter += current_distance;
    }
    float ideal_perimeter = (2 * _target_width + 2 * _target_height);
    if (fabs((perimeter - ideal_perimeter) / ideal_perimeter >
             GEOMETRY_TOLERANCE)) {
      return false;
    }

    // Check width + height?
    return true;
  }
};

void comb(int N, int K, std::vector<std::vector<int>> &groups) {
  int upper_factorial = 1;
  int lower_factorial = 1;

  for (int i = 0; i < K; i++) {
    upper_factorial *= (N - i);
    lower_factorial *= (K - i);
  }
  int n_permutations = upper_factorial / lower_factorial;

  if (DEBUG)
    cout << N << " centers found. Iterating over " << n_permutations
         << " possible sets of candidates" << endl;

  std::string bitmask(K, 1); // K leading 1's
  bitmask.resize(N, 0);      // N-K trailing 0's

  // print integers and permute bitmask
  do {
    std::vector<int> group;
    for (int i = 0; i < N; ++i) // [0..N-1] integers
    {
      if (bitmask[i]) {
        group.push_back(i);
      }
    }
    groups.push_back(group);
  } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

  assert(groups.size() == n_permutations);
}

const std::string currentDateTime() {
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

  return buf;
}

#endif
