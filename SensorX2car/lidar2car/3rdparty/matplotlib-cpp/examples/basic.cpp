#define _USE_MATH_DEFINES // for sin/log
#include "../matplotlibcpp.h"
#include <cmath>
#include <iostream>

namespace plt = matplotlibcpp;

int main() {
  int n = 5000; // 5000 data points
  std::vector<double> x(n), y(n), z(n), w(n, 2);
  for (int i = 0; i < n; ++i) {
    x.at(i) = i * i;
    y.at(i) = sin(2 * M_PI * i / 360.0);
    z.at(i) = log(i);
  }

  plt::figure(); // declare a new figure (optional if only one is used)

  plt::plot(x, y); // automatic coloring: tab:blue
  plt::show(false);
  plt::plot(x, w, "r--");                 // red dashed line
  plt::plot(x, z, {{"label", "log(x)"}}); // legend label "log(x)"

  plt::xlim(0, 1000 * 1000);    // x-axis interval: [0, 1e6]
  plt::title("Standard usage"); // set a title
  plt::legend();                // enable the legend

  plt::savefig("standard.pdf"); // save the figure
  plt::show();
}
