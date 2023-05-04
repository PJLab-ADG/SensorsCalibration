#include "../matplotlibcpp.h"
namespace plt = matplotlibcpp;

void horizontal() {
    plt::axhline(0.2);
    plt::axhline(0.1, {{"color", "red"}});
}

void vertical() {
    plt::axvline(0.2);
    plt::axvline(0.1, {{"color", "red"}});
}

int main() {
  plt::figure();
  horizontal();
  vertical();
  plt::show();

  return 0;
}
