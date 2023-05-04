#include <vector>
#include "../matplotlibcpp.h"
namespace plt = matplotlibcpp;

void basic() {
  plt::figure();
  plt::plot({1, 2, 3}, {{"label", "a line"}});
  plt::plot({1, 3, 5}, {{"label", "also a line"}});
  plt::legend();
  plt::show();
}

void loc() {
  plt::figure();
  plt::plot({1, 2, 3}, {{"label", "a line"}});
  plt::plot({1, 3, 5}, {{"label", "also a line"}});
  plt::legend("lower left");
  plt::show();
}

void bbox() {
  plt::figure();
  plt::plot({1, 2, 3}, {{"label", "a line"}});
  plt::plot({1, 3, 5}, {{"label", "also a line"}});
  plt::legend(std::vector<double>{0.5, 0.7});
  plt::show();
}

// currently not working: only strings can be provided in the keyword arguments
// void keywords() {
//  plt::figure();
//  plt::plot({1, 2, 3}, {{"label", "a line"}});
//  plt::plot({1, 3, 5}, {{"label", "also a line"}});
//  plt::legend("best", {{"borderpad", "0.2"}});
//  plt::show();
// }

int main() {
  basic();
  loc();
  bbox();
  // keywords();
  return 0;
}
