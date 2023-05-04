#define _USE_MATH_DEFINES
#include "../matplotlibcpp.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace plt = matplotlibcpp;

int main() {
  // Prepare data.
  int n = 5000;
  Eigen::VectorXd x(n), y(n), z(n), w = 2 * Eigen::VectorXd::Ones(n);
  for (int i = 0; i < n; ++i) {
    x(i) = i * i;
    y(i) = sin(2 * M_PI * i / 360.0);
    z(i) = log(i);
  }

  // Set the size of output image = 1200x780 pixels
  plt::figure_size(1200, 780);

  // Plot line from given x and y data. Color is selected automatically.
  plt::plot(x, y);

  // Plot a red dashed line from given x and y data.
  plt::plot(x, w, "r--");

  // Plot a line whose name will show up as "log(x)" in the legend.
  plt::plot(x, z, {{"label", "log(x)"}});

  // Set x-axis to interval [0,1000000]
  plt::xlim(0, 1000 * 1000);

  // Add graph title
  plt::title("Sample figure");

  // Enable legend.
  plt::legend();

  // save figure
  const char *filename = "./eigen_basic.png";
  std::cout << "Saving result to " << filename << std::endl;

  plt::savefig(filename);
}
