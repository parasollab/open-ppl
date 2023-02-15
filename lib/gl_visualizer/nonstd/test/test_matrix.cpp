#include <algorithm>
#include <iostream>
#include <string>

#include "nonstd/matrix.h"
#include "nonstd/vector.h"
#include "nonstd/numerics.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

static const string er = "\ttest_matrix error: ";


template <typename T, size_t N>
bool
validate_identity(const matrix_type<T, N, N> _m) {
  // Check size.
  if(_m.rows() != N or _m.cols() != N or _m.size() != N * N)
    return false;

  // Check diagonal.
  for(size_t i = 0; i < N; ++i)
    if(_m(i, i) != 1)
      return false;

  // Check of-diagonals.
  for(size_t i = 0; i < N; ++i) {
    for(size_t j = 0; j < N; ++j) {
      if(i == j)
        continue;
      else if(_m(i, j) != 0)
        return false;
    }
  }

  return true;
}


// Test instantiation and element access.
void test_instantiation() {
  const string when = "when testing instantiation, ";

  // Default constructor test.
  matrix_type<float, 3, 2> m0;
  assert_msg(m0.rows() == 3 and m0.cols() == 2 and m0.size() == 6 and
      all_of(m0.begin(), m0.end(), [](float& _f) {return _f == 0;}),
      er + when + "default constructor failed!");

  // 2d array construction test.
  float a1[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  matrix_type<float, 3, 3> m1(a1);
  assert_msg(validate_identity(m1), er + when + "2d array constructor failed!");

  // 1d array construction test.
  float a2[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  matrix_type<float, 3, 3> m2(a2);
  assert_msg(validate_identity(m2), er + when + "1d array constructor failed!"
      + "\n\t\texpected identity, but got:\n" + to_string(m2));

  // Copy constructor test.
  matrix_type<float, 3, 3> m3(m2);
  assert_msg(validate_identity(m3), er + when + "copy constructor failed!");
}


void test_arithmetic() {
}


int main() {
  cerr << "\ttesting matrix..." << flush;

  test_instantiation();
  test_arithmetic();

  cerr << "passed" << endl;
}
