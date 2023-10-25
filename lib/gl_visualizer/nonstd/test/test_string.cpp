#include <algorithm>
#include <iostream>

#include "nonstd/string.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

static const string er = "\ntest_string error: ";

void test_trim() {
  string when = "when testing trim, ";

  // Create test cases
  vector<string> t1 = {"  asdf",
                       "asdf  ",
                       "\nasdf",
                       "asdf\n",
                       "\t asdf",
                       "  \t\n asdf \t  \n ",
                       "asdf"};
  vector<string> t2 = {"  as df",
                       "as df  ",
                       "\nas df",
                       "as df\n",
                       "\t as df",
                       "  \t\n as df \t  \n ",
                       "as df"};

  // Assert each test case is handled correctly
  assert_msg(trim("") == "", er + when + "expected no change on empty string, "
      "but got '" + trim("") + "'!");
  assert_msg(all_of(t1.begin(), t1.end(),
      [](const string& _s) {return trim(_s) == "asdf";}), er + when +
      "one or more test cases failed to produce 'asdf'!");
  assert_msg(all_of(t2.begin(), t2.end(),
      [](const string& _s) {return trim(_s) == "as df";}), er + when +
      "one or more test cases failed to produce 'as df'!");
}


void test_tokenize() {
  string test = "asdf qwer, y,,xqz";

  // Tokenize test string with two different delimiter sets
  auto tokens1 = tokenize(test, ",");
  auto tokens2 = tokenize(test, ", ");

  string when = "when testing tokenize, splitting the test string '" + test +
      "' on the delimiter";

  // Check each tokenization to be sure delimiters are handled correctly
  assert_msg(tokens1 == vector<string>{"asdf qwer", "y", "xqz"}, er + when +
      " ',', did not get expected {\"asdf qwer\", \"y\", \"xqz\"}!");
  assert_msg(tokens2 == vector<string>{"asdf", "qwer", "y", "xqz"}, er + when +
      "s ', ', did not get expected {\"asdf\", \"qwer\", \"y\", \"xqz\"}!");
}


int main() {
  cerr << "\ttesting string..." << flush;

  test_tokenize();

  cerr << "passed" << endl;
}
