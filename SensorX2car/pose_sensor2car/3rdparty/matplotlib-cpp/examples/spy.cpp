#include <vector>
#include <Eigen/Dense>
#include "../matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main() {

  const unsigned n = 100;
  Eigen::MatrixXd A(n / 2, n);
  std::vector<std::vector<double>> B;

  for (unsigned i = 0; i < n / 2; ++i) {
    A(i, i) = 1;
    std::vector<double> row(n);
    row[i] = 1;

    if (i < n / 2) {
      A(i, i + n / 2) = 1;
      row[i + n / 2] = 1;
    }
    B.push_back(row);
  }

  for (unsigned i = 0; i < n / 2; ++i) {
    for (unsigned j = 0; j < n; ++j) {
      if (A(i, j) != B[i][j]) {
        std::cout << i << "," << j << " differ!\n";
      }
    }
  }

  plt::figure();
  plt::title("Eigen");
  plt::spy(A);

  plt::figure();
  plt::title("vector");
  plt::spy(B);
  plt::show();
  return 0;
}
