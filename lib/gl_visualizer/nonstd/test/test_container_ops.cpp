#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "nonstd/container_ops.h"
#include "nonstd/numerics.h"
#include "nonstd/runtime.h"
#include "nonstd/vector.h"

using namespace std;
using namespace nonstd;

static const string er = "\n\t\terror: ";
typedef vector_type<double, 3> vec;


void test_magnitudes()
{
  static const string when = "when testing magnitudes, ";
  vector<double> x{1, 1, 1};
  vec y{1, 1, 1};

  auto mag = &magnitude<double, decltype(x)>;
  auto mag_sqr = &magnitude_sqr<double, decltype(x)>;

  assert_msg(mag(x) == y.mag(), er + when + "expected same magnitude from "
      "container and vector_type versions, but got " + to_string(mag(x)) +
      " != " + to_string(y.mag()));
  assert_msg(mag_sqr(x) == y.mag_sqr(), er + when + "expected same "
      "magnitude squared from container and vector_type versions, but got " +
      to_string(mag_sqr(x)) + " != " + to_string(y.mag_sqr()));
}


void test_unit()
{
  static const string when = "when testing unit function, ";
  vector<double> x{2, 1, -5};
  vec y{2, 1, -5};

  vector<double> xu = unit(x);
  vec yu = y.hat();

  for(size_t i = 0; i < 3; ++i)
    assert_msg(xu[i] == yu[i], er + when + "expected same result from container "
        "and vector_type versions, but got " + print_container(xu) + " != " +
        to_string(yu));
}


void test_dot()
{
  static const string when = "when testing dot function, ";
  vector<double> x{2, 1, -5};
  vector<double> y{1, 3, 1};

  vec u{2, 1, -5};
  vec v{1, 3, 1};

  assert_msg(dot<double>(x, y) == u * v, er + when + "expected same result from "
      "container and vector_type versions, but got " +
      to_string(dot<double>(x, y)) + " != " + to_string(u * v) + ".");
}


int main()
{
  cerr << "\ttesting container_ops..." << flush;

  test_magnitudes();
  test_unit();
  test_dot();

  cerr << "passed" << endl;
}
