#include <iostream>

#include "nonstd/options_manager.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

static const string er = "\ttest_options_manager error: ";

// Example derived class for testing
class example_manager : public options_manager {

  public:

    example_manager(int _argc, char* _argv[]) : options_manager(_argc, _argv) {
      add_flag("-e", vector<string>{"qwer", "asdf"});
      add_flag("-n");
      add_flag("-r", string(R"(.*\.cpp)"));
      this->initialize();
    }
};


int main() {
  cerr << "\ttesting options_manager..." << flush;

  char* args[6] = {(char*)"./executable-name",
                   (char*)"-e", (char*)"asdf",
                   (char*)"-r", (char*)"abcd.cpp",
                   (char*)"-n"};

  example_manager om(6, args);

  assert_msg(om.get_flags() == vector<string>{"-e", "-n", "-r"}, er +
      "flags were not {'-e', '-n', '-r'} as expected!");
  assert_msg(om.get_arg("-e") == string("asdf"), er +
      "argument for flag '-e' was not 'asdf' as expected!");
  assert_msg(om.get_arg("-n") == string(), er + "argument for flag "
      "'-n' was not null as expected!");
  assert_msg(om.get_arg("-r") == string("abcd.cpp"), er + "argument "
      "for flag '-r' was not 'abcd.cpp' as expected!");

  cerr << "passed" << endl;
}
