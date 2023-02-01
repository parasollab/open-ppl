#include <iostream>
#include <string>

#include <unistd.h>

#include "nonstd/timer.h"
#include "nonstd/numerics.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

static const string er = "\ttest_timing error: ";


// Use a timer to measure a usleep duration; test that the measured time is
// within 5% of the sleep time (some variability is expected from usleep).
int main() {
  cerr << "\ttesting timing..." << flush;

  timer t;

  assert_msg(approx(t.elapsed(), 0.), er + "expected initial duration ~= 0, but "
      "got " + to_string(t.elapsed()) + "!");

  const double microseconds = 22000,
               seconds      = microseconds / 1e6,
               tolerance    = seconds * .05;
  t.start();
  usleep((int)microseconds);
  t.stop();

  assert_msg(
      approx(t.elapsed(), seconds, tolerance),
      er + "after usleep(22000), expected duration ~= " + to_string(seconds) +
      " seconds, but got " + to_string(t.elapsed()) + "!");

  cerr << "passed" << endl;
}
