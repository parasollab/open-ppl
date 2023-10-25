#include <iostream>

#include "nonstd/status.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

static const string er = "\ttest_status error: ";


int
main()
{
  cerr << "\ttesting status..." << flush;

  // Initialize a status.
  nonstd::status s;

  // Assert that s is only ready.
  string when = "after initializing, ";
  assert_msg(!s.is_started(),  er + when + "the status should not be started.");
  assert_msg(!s.is_on_hold(),  er + when + "the status should not be on hold.");
  assert_msg(!s.is_complete(), er + when + "the status should not be complete.");

  // Set s to start.
  s.start();
  when = "after starting, ";
  assert_msg(s.is_started(),   er + when + "the status should be started.");
  assert_msg(!s.is_on_hold(),  er + when + "the status should not be on hold.");
  assert_msg(!s.is_complete(), er + when + "the status should not be complete.");

  // Set s to hold.
  s.hold();
  when = "after holding, ";
  assert_msg(s.is_started(),   er + when + "the status should be started.");
  assert_msg(s.is_on_hold(),   er + when + "the status should be on hold.");
  assert_msg(!s.is_complete(), er + when + "the status should not be complete.");

  // Set s to resume.
  s.resume();
  when = "after resuming, ";
  assert_msg(s.is_started(),   er + when + "the status should be started.");
  assert_msg(!s.is_on_hold(),  er + when + "the status should not be on hold.");
  assert_msg(!s.is_complete(), er + when + "the status should not be complete.");

  // Set s to finished.
  s.complete();
  when = "after finishing, ";
  assert_msg(s.is_started(),   er + when + "the status should be started.");
  assert_msg(!s.is_on_hold(),  er + when + "the status should not be on hold.");
  assert_msg(s.is_complete(),  er + when + "the status should be complete.");

  // Reset s.
  s.reset();
  when = "after resetting, ";
  assert_msg(!s.is_started(),  er + when + "the status should not be started.");
  assert_msg(!s.is_on_hold(),  er + when + "the status should not be on hold.");
  assert_msg(!s.is_complete(), er + when + "the status should not be complete.");

  cerr << "passed" << endl;
}
