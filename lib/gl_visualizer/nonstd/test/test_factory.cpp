#include <iostream>
#include <tuple>

#include "nonstd/factory.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

static const string er = "\ttest_factory error: ";

int main() {
  cerr << "\ttesting factory..." << flush;

  // Create factories for integer and tuple
  factory<int> fac_i;
  factory<tuple<int, char, double>> fac_t;

  // Generate factory products
  auto x = fac_i(1);
  auto t = fac_t(1, 'a', .1);

  string when = "when testing the output product, ";

  // Check that the products are correct
  assert_msg(*x == 1, er + when + "expected int factory to produce 1, but got "
      + to_string(*x) + "!");
  assert_msg(*t == tuple<int, char, double>(1, 'a', .1), er + when +
      "expected tuple factory to produce {1, 'a', .1}, but got {" +
      to_string(get<0>(*t)) + ", " + to_string(get<1>(*t)) + ", " +
      to_string(get<2>(*t)) + "!");

  cerr << "passed" << endl;
}
