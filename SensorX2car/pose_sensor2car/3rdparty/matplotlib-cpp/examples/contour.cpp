#include <Eigen/Dense>
#include "../matplotlibcpp.h"
namespace plt = matplotlibcpp;

template <typename Vector>
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> meshgrid(const Vector& x, const Vector& y) {
  const unsigned m = x.size(), n = y.size();
  // todo fix type
  Eigen::MatrixXd X(m, n), Y(m, n);
  for (unsigned i = 0; i < m; ++i) {
    for (unsigned j = 0; j < n; ++ j) {
      X(i, j) = *(x.data() + i);
      Y(i, j) = *(y.data() + j);
    }
  }
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> res = std::make_pair(X, Y);
  return res;
}

void plot_contour() {
  const unsigned n = 100;
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(n, 0, 1),
                  y = Eigen::VectorXd::LinSpaced(n, 0, 1);
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> XY = meshgrid(x, y);
  Eigen::MatrixXd Z = XY.first + 2 * XY.second;

  plt::contour(XY.first, XY.second, Z);
  plt::show();

}

int main() {
  plot_contour();
  return 0;
}
