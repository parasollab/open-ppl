#include <cstdlib>
#include <iostream>

#include "nonstd/runtime.h"

using namespace std;

namespace nonstd {

  void
  exit_msg(const string& _message) noexcept
  {
    cerr << _message << endl;
    exit(-1);
  }


  void
  assert_msg(const bool _condition, const string& _message) noexcept
  {
    if(!_condition) exit_msg(_message);
  }

}
