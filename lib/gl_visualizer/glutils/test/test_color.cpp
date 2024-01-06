#include <cstdio>
#include <iostream>
#include <string>

#include "glutils/color.h"
#include "nonstd/runtime.h"
#include "nonstd/io.h"

using namespace std;
using namespace glutils;
using nonstd::assert_msg;

static const string er = "\n\t\terror: ";


/// Check that writing out a color produces the correct string.
void
test_write()
{
  // Create a color and the expected output string.
  const glutils::color c(1, .2, .3, .4);
  const std::string expected = "1 0.2 0.3 0.4";

  // Write the color to a string.
  std::ostringstream output;
  output << c;

  // Verify that the written string matches the expected output.
  assert_msg(expected == output.str(), er + "color(1, .2, .3, .4) should "
      "produce output string '" + expected + "', but result was '" +
      output.str() + "'.");
}


/// Check that reading in a color works despite variations in format.
void
test_read()
{
  // Create a color.
  const glutils::color expected(1, .2, .3, .4);
  std::ostringstream expected_output;
  expected_output << expected;

  // Create a bunch of alternate string representations of this color.
  const std::string good_input("1 .2 .3 .4");
  const std::vector<std::string> bad_inputs{"{1, .2, .3, .4}",
                                            "1, .2, .3, .4",
                                            "1 .2 .3",
                                            "{1 .2 .3 .4}",
                                            "[1 .2 .3 .4]",
                                            "(1 .2 .3 .4)",
                                            "(1, .2, .3, .4)",
                                            "(1; .2; .3; .4)",
                                            "/1 .2 .3 .4|"};

  // Test that the good input produces the expected color.
  {
    // Read the input string into a color object.
    glutils::color test;
    std::istringstream input(good_input);
    input >> test;

    // Verify the produced color is equal to the expected value.
    assert_msg(test == expected, er + "input string '" + good_input + "' did not "
        "produce the expected color '" + expected_output.str() + "'.");
  }

  // Test that each bad input string produces an exception.
  for(const auto& string : bad_inputs)
  {
    // Read the input string into a color object.
    try
    {
      glutils::color test;
      std::istringstream input(string);
      input >> test;
    }
    catch(const nonstd::exception& _e)
    {
      continue;
    }

    assert_msg(false, er + "bad input string '" + string + "' did not "
        "produce an exception.");
  }
}


int
main()
{
  cerr << "\ttesting color..." << flush;

  test_write();
  test_read();

  cerr << "passed" << endl;
}
