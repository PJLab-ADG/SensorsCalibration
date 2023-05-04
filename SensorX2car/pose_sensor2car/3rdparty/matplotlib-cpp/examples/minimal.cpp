#include "../matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

int main() {
  // plot(y) - the x-coordinates are implicitly set to [0,1,...,n)
  // note, that plot({..}, {..}) is not supported due to the ambiguous cast
  // of {..} to either std::string or std::vector
  plt::plot({1, 3, 2, 4});
  plt::savefig("minimal.pdf");
}
