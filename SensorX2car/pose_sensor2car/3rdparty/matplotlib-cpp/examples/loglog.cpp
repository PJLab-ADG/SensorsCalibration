#define _USE_MATH_DEFINES
#include "../matplotlibcpp.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace plt = matplotlibcpp;

int main() {
  // Prepare data.
  int n = 5000;
  Eigen::VectorXd x(n), y(n), z(n), w = Eigen::VectorXd::Ones(n);
  for (int i = 0; i < n; ++i) {
    double value = (1.0 + i) / n;
    x(i) = value;
    y(i) = value * value;
    z(i) = value * value * value;
  }

  // Plot line from given x and y data. Color is selected automatically.
  plt::loglog(x, y);

  // Plot a red dashed line from given x and y data.
  plt::loglog(x, w, "r--");

  // Plot a line whose name will show up as "log(x)" in the legend.
  plt::loglog(x, z, "g:", {{"label", "$x^3$"}});

  // Add graph title
  plt::title("Sample figure");

  // Put the legend in the center of the bottom right quadrant.
  // First argument: loc, second: bbox_to_anchor
  plt::legend("center", {0.5, 0, 0.5, 0.5});

  // show figure
  plt::show();
}
