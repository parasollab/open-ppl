#include <iostream>
#include <sstream>

#include "nonstd/io.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

static const string er = "\ttest_io error: ";


void
test_read_file_lines()
{
  // Read the test file.
  auto lines = read_file_lines("test/test_file.txt");

  string when = "when testing read_file_lines, ";
  vector<string> expected = {"the quick black cat", "jumped over",
      "the lazy dog"};

  // Check number of lines
  assert_msg(lines.size() == 3, er + when + "expected line count = 3, but got "
      "line count = " + to_string(lines.size()));

  // Check each line
  for(size_t i = 0; i < 3; ++i)
    assert_msg(lines[i] == expected[i], er + when + "expected line " +
        to_string(i) + " = '" + expected[i] + "', but got '" + lines[i] + "'!");
}


void
test_read_file()
{
  // Read the test file.
  const std::string contents = read_file("test/test_file.txt");
  const std::string expected = "the quick black cat\njumped over\nthe lazy dog";
  const std::string when     = "when testing read_file, ";

  // Check for string match
  assert_msg(contents == expected, er + when + "test file contents did not match "
      "expected value\nexpected: '" + expected + "'\ncontents: '" + contents +
      "'");
}


void
test_print_container()
{
  ostringstream out;

  out << pair<int, char>(0, 'a') << endl
      << list<float>{0., 1.1, 2.2} << endl
      << vector<int>{0, 1, 2, 3, 4} << endl
      << array<short, 4>{0, 1, 2, 3} << endl;

  string expected = "(0, a)\n"
                    "{0, 1.1, 2.2}\n"
                    "{0, 1, 2, 3, 4}\n"
                    "{0, 1, 2, 3}\n";

  assert_msg(out.str() == expected, er + "when testing print_container, "
      "expected:\n" + expected + "...but got:\n" + out.str());
}


template <typename ContainerType>
void
test_input_container(
    const ContainerType& _expected,
    const std::string& _good_input,
    const std::vector<std::string>& _bad_inputs
) {
  const std::string when = "when testing input_container, ";

  ContainerType test;

  // Test good input.
  {
    try
    {
      std::istringstream buffer(_good_input);
      buffer >> test;
    }
    catch(nonstd::exception _e)
    {
      _e << "\n" << when << "good input '" << _good_input << "' failed.";
      throw _e;
    }

    assert_msg(test == _expected, er + when + "expected good input '" +
        _good_input + "' to produce equivalent object.");
  }

  // Test bad input.
  for(const auto& string : _bad_inputs)
  {
    std::istringstream buffer(string);
    try
    {
      buffer >> test;
    }
    catch(const nonstd::exception&)
    {
      continue;
    }

    exit_msg(er + when + "expected bad input '" + string +
        "' to produce an exception.");
  }
}


void
test_input_container()
{

  // Test pair.
  {
    std::pair<int, char> expected(0, 'a');
    std::string good_input("(0, a)");
    std::vector<std::string> bad_inputs{"0, a",
                                        "(0, 12)",
                                        "(a, a)",
                                        "(0, a, a)",
                                        "(0)"};

    test_input_container(expected, good_input, bad_inputs);
  }

  // Test list.
  {
    std::list<float> expected{0., 1.1, 2.2};
    std::string good_input("{0, 1.1, 2.2}");
    std::vector<std::string> bad_inputs{"0, 1.1, 2.2",
                                        "(0, 1.1, 2.2)",
                                        "{0, a, 2.2}",
                                        "0 1.1 2.2",
                                        "{0, 1.1, 2.2"};

    test_input_container(expected, good_input, bad_inputs);
  }

  // Test vector.
  {
    std::vector<int> expected{0, 1, 2, 3, 4};
    std::string good_input("{0, 1, 2, 3, 4}");
    std::vector<std::string> bad_inputs{"0, 1, 2, 3, 4",
                                        "(0, 1, 2, 3, 4)",
                                        "{0, a, 2, 3, 4}",
                                        "0 1 2 3 4",
                                        "{0, 1, 2, 3, 4"};

    test_input_container(expected, good_input, bad_inputs);
  }

  // Test array.
  {
    std::array<short, 4> expected{0, 1, 2, 3};
    std::string good_input("{0, 1, 2, 3}");
    std::vector<std::string> bad_inputs{"0, 1, 2, 3",
                                        "(0, 1, 2, 3)",
                                        "{0, a, 2, 3}",
                                        "0 1 2 3",
                                        "{0, 1, 2, 3"};

    test_input_container(expected, good_input, bad_inputs);
  }
}


int
main()
{
  cerr << "\ttesting io..." << flush;

  test_read_file_lines();
  test_read_file();
  test_print_container();
  test_input_container();

  cerr << "passed" << endl;
}
