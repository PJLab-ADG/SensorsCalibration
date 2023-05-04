#define _USE_MATH_DEFINES
#include "../matplotlibcpp.h"
#include <cmath>
#include <vector>

namespace plt = matplotlibcpp;

int main() {
  // Prepare data for parametric plot.
  int n = 5000; // number of data points
  std::vector<double> x(n), y(n), z(n);
  for (int i = 0; i < n; ++i) {
    double t = 2 * M_PI * i / n;
    x.at(i) = 16 * sin(t) * sin(t) * sin(t);
    y.at(i) = 13 * cos(t) - 5 * cos(2 * t) - 2 * cos(3 * t) - cos(4 * t);
    z.at(i) = 12.5 + abs(sin(x.at(i)));
  }

  // plot() takes an arbitrary number of (x,y,format)-triples.
  // x must be iterable (that is, anything providing begin(x) and end(x)),
  // y must either be callable (providing operator() const) or iterable.
  plt::plot(x, z, "k-", x, y, "r-");

  // show plots
  plt::show();
}
