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
  plt::semilogy(x, y);

  // Plot a red dashed line from given x and y data.
  plt::semilogy(x, w, {{"c", "r"}, {"ls", "--"}});

  // Plot a line whose name will show up as "log(x)" in the legend.
  plt::semilogy(x, z, "g:", {{"label", "$x^3$"}});

  // Add graph title
  plt::title("Sample figure");

  // Enable legend.
  plt::legend();

  // show figure
  plt::show();
}
