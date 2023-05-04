#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include "../matplotlibcpp.h"
namespace plt = matplotlibcpp;

void plot() {
  const unsigned n = 100;
  std::vector<double> x(n), y(n);

  #include <iostream>
  for (unsigned i = 0; i < n; ++i) {
    x[i] = sin(2 * M_PI * i / n);
    y[i] = cos(2 * M_PI * i / n);
  }

  plt::scatter(x, y, {{"color", "red"}, {"label", "a circle!"}});
  plt::legend();
  plt::show();
}

int main() {
  plot();
  return 0;
}
