#include <cmath>
#include "../matplotlibcpp.h"
namespace plt = matplotlibcpp;

#ifdef WITH_EIGEN
  #include <Eigen/Dense>
#endif

typedef std::vector<std::vector<double>> nested_vectors;

template <typename Matrix>
void get_data(Matrix& x, Matrix& y, Matrix& z) {
  // check the sizes
  const unsigned ncols = x.cols(),
                 nrows = x.rows();
  assert(ncols == y.cols() && ncols == z.cols());
  assert(nrows == y.rows() && nrows == z.rows());

  // write data
  const double a = -5,
               b = 5;
  const double col_incr = (b - a) / ncols,
               row_incr = (b - a) / nrows;
  for (unsigned i = 0; i < nrows; ++i) {
    for (unsigned j = 0; j < ncols; ++j) {
      x(i, j) = i;
      y(i, j) = j;
      z(i, j) = ::std::sin(::std::hypot(a + i * col_incr, a + j * row_incr));
    }
  }
}

template <>
void get_data(nested_vectors& x, nested_vectors& y, nested_vectors& z) {
    for (double i = -5; i <= 5;  i += 0.25) {
        std::vector<double> x_row, y_row, z_row;
        for (double j = -5; j <= 5; j += 0.25) {
            x_row.push_back(i);
            y_row.push_back(j);
            z_row.push_back(::std::sin(::std::hypot(i, j)));
        }
        x.push_back(x_row);
        y.push_back(y_row);
        z.push_back(z_row);
    }
}


int main() {
    std::vector<std::vector<double>> x, y, z;
    get_data(x, y, z);
    plt::plot_surface(x, y, z);
    plt::show();

    #ifdef WITH_EIGEN
      const unsigned n = 100;  // resolution of hypot function
      Eigen::MatrixXd X(n,n), Y(n,n), Z(n,n);
      get_data(X, Y, Z);
      plt::plot_surface(X, Y,   Z);
      plt::show();
    #endif
}
