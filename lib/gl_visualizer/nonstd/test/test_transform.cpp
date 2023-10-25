#include <algorithm>
#include <iostream>
#include <string>

#include "nonstd/transform.h"
#include "nonstd/numerics.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

static const string er = "\ttest_transform error: ";


void test_identity() {
  const string when = "when testing identity transform, ";

  transform_type<float> trans;
  vector_type<float, 3> v{2, 3, 0};
  auto vt = trans * v;

  assert_msg(vt == v, er + when + "expected no change to vector " +
      to_string(v) + ", " + " but got " + to_string(vt));
}


void test_generators() {
  const string when = "when testing generators, ";
  const float pi = 3.14159;

  const auto x90 = transform_type<float>::rotation_about_x(pi / 2);
  const auto y90 = transform_type<float>::rotation_about_y(pi / 2);
  const auto z90 = transform_type<float>::rotation_about_z(pi / 2);

  using vec = vector_type<float, 3>;
  const vec vo{1, 1, 1}, x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};
  vec v, vt;

  (v = vo).rotate(x, pi / 2);
  vt = x90 * vo;
  assert_msg(v == vt, er + when + "expected equivalent results from vector and "
      "transform methods, but got:\n\t" + to_string(v) + "\nand\n\t" +
      to_string(vt));

  (v = vo).rotate(y, pi / 2);
  vt = y90 * vo;
  assert_msg(v == vt, er + when + "expected equivalent results from vector and "
      "transform methods, but got:\n\t" + to_string(v) + "\nand\n\t" +
      to_string(vt));

  (v = vo).rotate(z, pi / 2);
  vt = z90 * vo;
  assert_msg(v == vt, er + when + "expected equivalent results from vector and "
      "transform methods, but got:\n\t" + to_string(v) + "\nand\n\t" +
      to_string(vt));
}


int main() {
  cerr << "\ttesting transform..." << flush;

  test_identity();
  test_generators();

  cerr << "passed" << endl;
}
