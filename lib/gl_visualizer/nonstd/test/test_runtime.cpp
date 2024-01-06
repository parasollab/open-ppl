#include <iostream>

#include "nonstd/runtime.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

int main() {
  // If assert exits the program, the test worked.
  assert_msg(false, "\ttesting runtime...passed");

  // Otherwise not so much.
  cerr << "\ttesting runtime...failed" << endl;
}
