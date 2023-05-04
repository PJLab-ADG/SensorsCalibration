#define __USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Dense>
#include "../matplotlibcpp.h"
namespace plt = matplotlibcpp;

void waves(const unsigned n) {
  Eigen::MatrixXd X(n, n);
  for (unsigned i = 0; i < n; ++i) {
    for (unsigned j = 0; j < n; ++j) {
      X(i, j) = sin(3.0 * M_PI * i / n) * cos(20.0 * M_PI * j / n);
    }
  }
  plt::figure();
  plt::imshow(X, {{"cmap", "Spectral"}});
  plt::colorbar();
  plt::show();
}

int main() {
  waves(200);
  return 0;
}
