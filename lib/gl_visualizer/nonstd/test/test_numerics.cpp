#include <iostream>
#include <vector>

#include "nonstd/numerics.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

static const string er = "\ttest_numerics error: ";


void test_sign() {
  string when = "when testing sign, ";

  // Test integers
  assert_msg(sign(-1) == -1, er + when + "expected sign of -1 = -1, but got " +
      to_string(sign(-1)));
  assert_msg(sign(1) == 1, er + when + "expected sign of 1 = 1, but got " +
      to_string(sign(1)));
  assert_msg(sign(0) == 0, er + when + "expected sign of 0 = 0, but got " +
      to_string(sign(0)));

  // Test doubles
  assert_msg(sign(-.00001) == -1, er + when + "expected sign of -.00001 = -1, "
      "but got " + to_string(sign(-.00001)));
  assert_msg(sign(.00001) == 1, er + when + "expected sign of .00001 = 1, "
      "but got " + to_string(sign(.00001)));
  assert_msg(sign(0.) == 0, er + when + "expected sign of 0. = 0, "
      "but got " + to_string(sign(0.)));
}


void test_approx() {
  string when = "when testing approx, ";

  // Test integer
  assert_msg(approx(1, 0, 1), er + when + "expected 1 ~= 0 with "
      "tolerance = 1, but got false!");
  assert_msg(!approx(2, 0, 1), er + when + "expected 2 !~= 0 with "
      "tolerance = 1, but got true!");

  // Test double
  assert_msg(approx(.0001, 0., .0001), er + when + "expected .0001 ~= 0 with "
      "tolerance = .0001, but got false!");
  assert_msg(!approx(.00011, 0., .0001), er + when + "expected .00011 !~= 0 with "
      "tolerance = .0001, but got true!");
}


void test_in_bounds() {
  string when = "when testing in_bounds, ";

  // Test integer
  assert_msg(in_bounds(3, 1, 5), er + when + "expected 3 to be in the range "
      "[1,5], but got false!");
  assert_msg(!in_bounds(0, 1, 5), er + when + "expected 0 to not be in the range "
      "[1,5], but got true!");

  // Test double
  assert_msg(in_bounds(.1, 0., 1.), er + when + "expected .1 to be in the range "
      "[0,1], but got false!");
  assert_msg(!in_bounds(1.00001, 0., 1.), er + when + "expected 1.00001 to not "
      "be in the range [0,1], but got true!");
}


void test_approx_in_bounds() {
  string when = "when testing approx_in_bounds, ";

  // Test integer
  assert_msg(approx_in_bounds(0, 1, 5, 1), er + when + "expected 0 to be "
      "approximately in the range [1,5] with tol = 1, but got false!");
  assert_msg(!approx_in_bounds(-1, 1, 5, 1), er + when + "expected -1 to not be "
      "approximately in the range [1,5] with tol = 1, but got true!");

  // Test double
  assert_msg(approx_in_bounds(1.1, 0., 1., .1), er + when + "expected 1.1 to be "
      "approximately in the range [0,1] with tol = .1, but got false!");
  assert_msg(!approx_in_bounds(1.1001, 0., 1., .1), er + when + "expected 1.101 "
      "to not be approximately in the range [0,1] with tol = .1, but got true!");
}


void test_rescale() {
  string when = "when testing rescale, ";

  // Test integer
  assert_msg(approx(rescale(2, 1, 5), -.5), er + when + "expected rescaling of 2 "
      "from range [1,5] to [-1,1] ~= -.5, but got " +
      to_string(rescale(2, 1, 5)) + "!");
  assert_msg(approx(rescale(-2, -11, 5), .125), er + when + "expected rescaling "
      "of -2 from range [-11,5] to [-1,1] ~= -.125, but got " +
      to_string(rescale(-2, -11, 5)) + "!");

  // Test double
}


void test_factorial() {
  string when = "when testing factorial, ";

  assert_msg(factorial(0) == 1, er + when + "expected 0! = 1, but got " +
      to_string(factorial(0)) + "!");
  assert_msg(factorial(1) == 1, er + when + "expected 1! = 1, but got " +
      to_string(factorial(1)) + "!");
  assert_msg(factorial(12) == 479001600, er + when + "expected 12! = 479001600, "
      "but got " + to_string(factorial(0)) + "!");
}


void test_log_base() {
  string when = "when testing log_base, ";

  assert_msg(log_base(100, 10) == 2, er + when + "expected log_10(100) = 2, but "
      "got " + to_string(log_base(100, 10)) + "!");
  assert_msg(log_base(64, 4) == 3, er + when + "expected log_4(64) = 3, but "
      "got " + to_string(log_base(64, 4)) + "!");
}


int main() {
  cerr << "\ttesting numerics..." << flush;

  test_sign();
  test_approx();
  test_in_bounds();
  test_approx_in_bounds();
  test_rescale();
  test_factorial();
  test_log_base();

  cerr << "passed" << endl;
}
