/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/utility/for_each_range.hpp>

#include <vector>

#include "../test_report.hpp"

void test_for_each_range_by()
{
  using person_type = std::pair<std::string, unsigned>;
  using vector_type = std::vector<person_type>;
  using iterator_type = vector_type::iterator;

  vector_type people{ { "Alice", 32 }, { "John", 17 },  { "Ricky", 17 },
                      { "Jose", 28 },  { "Marie", 28 }, { "Romeo", 10 } };

  unsigned range = 0;

  auto extractor = [](person_type const& person) { return person.second; };

  // Function to manually compare ranges
  auto func = [&](
    iterator_type begin, iterator_type end, unsigned age) mutable {

    switch (range) {
      case 0:
        STAPL_TEST_REPORT(std::distance(begin, end) == 1,
                          "Size of first range");
        STAPL_TEST_REPORT(age == 32, "Age of first range");
        STAPL_TEST_REPORT(begin->second == 32, "Age is the same");
        STAPL_TEST_REPORT(
          std::equal(begin->first.begin(), begin->first.end(), "Alice"),
          "People of first range");
        break;
      case 1:
        STAPL_TEST_REPORT(std::distance(begin, end) == 2,
                          "Size of second range");
        STAPL_TEST_REPORT(age == 17, "Age of second range");
        STAPL_TEST_REPORT(begin->second == 17, "Age is the same");
        STAPL_TEST_REPORT(
          std::equal(begin->first.begin(), begin->first.end(), "John"),
          "People of second range");
        break;
      case 2:
        STAPL_TEST_REPORT(std::distance(begin, end) == 2,
                          "Size of third range");
        STAPL_TEST_REPORT(age == 28, "Age of third range");
        STAPL_TEST_REPORT(begin->second == 28, "Age is the same");
        STAPL_TEST_REPORT(
          std::equal(begin->first.begin(), begin->first.end(), "Jose"),
          "People of third range");
        break;
      case 3:
        STAPL_TEST_REPORT(std::distance(begin, end) == 1,
                          "Size of fourth range");
        STAPL_TEST_REPORT(age == 10, "Age of fourth range");
        STAPL_TEST_REPORT(begin->second == 10, "Age is the same");
        STAPL_TEST_REPORT(
          std::equal(begin->first.begin(), begin->first.end(), "Romeo"),
          "People of fourth range");
        STAPL_TEST_REPORT(
          end == people.end(),
          "Last iterator in last range is the end of the vector");
        break;
      default:
        stapl::abort("Too many ranges");
    }
    range++;
  };

  stapl::utility::for_each_range_by(
    people.begin(), people.end(), func, extractor);

  STAPL_TEST_REPORT(range == 4, "Should be four ranges");
}


stapl::exit_code stapl_main(int, char*[])
{
  using vector_type = std::vector<int>;
  using iterator_type = vector_type::iterator;

  std::vector<int> vec{{-20,-2,4,5,6,-5,6,-7}};
  std::vector<int> out_vec;

  // Function to sum the range and add it to out_vec
  auto func = [&](iterator_type begin, iterator_type end) mutable {
    int sum = std::accumulate(begin, end, 0);
    out_vec.push_back(sum);
  };

  // Either both are negative or both are positive
  auto comp = [](int x, int y) {
    return (x < 0 && y < 0) || (x > 0 && y > 0);
  };

  stapl::utility::for_each_range(vec.begin(), vec.end(), func, comp);

  std::vector<int> expected{{-22, 15, -5, 6, -7}};

  bool passed = std::equal(out_vec.begin(), out_vec.end(), expected.begin());

  STAPL_TEST_REPORT(passed, "for_each_range sum of runs of pos/neg ints");

  ///
  /// Test for for_each_filtered_range

  auto pred = [](int x) {
    return x % 2 == 0;
  };

  out_vec.clear();
  stapl::utility::for_each_filtered_range(
    vec.begin(), vec.end(), func, comp, pred
  );

  expected = {-22, 4, 6, 6};

  passed = std::equal(out_vec.begin(), out_vec.end(), expected.begin());

  STAPL_TEST_REPORT(passed,
    "for_each_filtered_range sum of runs of even pos/neg ints");

  test_for_each_range_by();

  return EXIT_SUCCESS;
}
