#include <iostream>

#include "nonstd/condition_counter.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

static const string er = "\ttest_condition_counter error: ";

void test_consecutive() {
  string when = "when testing consecutive counter, expected to "
      "return true after 100 consecutive true evaluations, but ";

  // Make consecutive counter for 100 counts
  condition_counter c(100);

  // Evaluate true 99 times and be sure it says false
  for(size_t i = 1; i <= 99; ++i)
    assert_msg(!c(true), er + when + "got true after " + to_string(i) +
        " consecutive true evaluations!");

  // Evaluate false once and be sure it says false
  assert_msg(!c(false), er + when + "got true after 99 true evaluations and " +
      "1 false evaluation!");

  // Evaluate true 99 times again to be sure the false eval reset things
  for(size_t i = 1; i <= 99; ++i)
    assert_msg(!c(true), er + when + "got true after " + to_string(i) +
        " consecutive true evaluations!");

  // Make sure the 100th true eval returns true
  assert_msg(c(true), er + when + "didn't get true after 100 true evaluations!");
}


void test_non_consecutive() {
  string when = "when testing non-consecutive counter, expected to "
      "return true after 100 true evaluations, but ";

  // Make a non-consecutive counter for 100 counts
  condition_counter c(100, false);

  // Evaluate true 99 times and be sure it says false
  for(size_t i = 1; i <= 99; ++i)
    assert_msg(!c(true), er + when + "got true after " + to_string(i) +
        " true evaluations!");

  // Evaluate false 99 times and be sure it says false
  for(size_t i = 1; i <= 99; ++i)
    assert_msg(!c(false), er + when + "got true after 99 true evaluations and " +
        to_string(i) + " false evaluations!");

  // Make sure the 100th (non-consecutive) true eval returns true
  assert_msg(c(true), er + when + "didn't get true after 100 true evaluations!");
}


int main() {
  cerr << "\ttesting condition_counter..." << flush;

  test_consecutive();
  test_non_consecutive();

  cerr << "passed" << endl;
}
