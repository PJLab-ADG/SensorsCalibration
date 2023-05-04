#include <cmath>
#include "../matplotlibcpp.h"
namespace plt = matplotlibcpp;

#ifdef WITH_EIGEN
#include <Eigen/Dense>
#endif

template <typename Vector>
void get_data(Vector& x, Vector& y, Vector& err) {
  // get number of data points
  const unsigned n = x.size();
  assert(y.size() == n && err.size() == n);

  // compute data
  for (unsigned i = 0; i < n; ++i) {
    *(x.data() + i) = 1.0 * i / n;
    *(y.data() + i) = sin(2.0 * M_PI * i / n);
    *(err.data() + i) = exp(-0.1 * i);
  }
}

template <typename Vector>
void plot(unsigned n) {
  // get the data
  Vector x(n), y(n), err(n);
  get_data(x, y, err);

  // plot errorbar plot
  plt::errorbar(x, y, err);
}

int main() {
  // create figure
  plt::figure();

  // plot with std::vector
  plot<std::vector<double>>(10);

  // plot with Eigen::VectorXd, if specified
  #ifdef WITH_EIGEN
  plot<Eigen::VectorXd>(13);
  #endif

  // show plot
  plt::show();

  return 0;
}
