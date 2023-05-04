#include "../matplotlibcpp.h"
#include <Eigen/Dense>
#include <iostream>

namespace plt = matplotlibcpp;

int main() {

  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(200, 0, 6);
  Eigen::VectorXd y, z;

  // y = exp(sin(x)), z = exp(cos(z))
  y = x.array().sin().exp().matrix();
  z = x.array().cos().exp().matrix();

  plt::figure();

  plt::loglog(x, y, "tab:red");
  plt::loglog(x, z, "tab:blue", {{"linestyle", "--"}});

  plt::xlabel("Time in lecture");
  plt::ylabel("Student confusion");

  plt::grid();
  plt::savefig("eigen.pdf");
  // plt::show(); // show the figure instead of saving it
}
