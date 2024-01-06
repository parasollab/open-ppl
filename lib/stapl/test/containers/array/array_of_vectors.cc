/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <test/algorithms/test_utils.h>

#include "../../test_report.hpp"

using namespace stapl;

struct fill_offset
{
  typedef void result_type;

  template<typename T, typename Index>
  void operator()(T x, Index index)
  {
    const std::size_t n = x.size();

    std::vector<size_t> v(n);

    for (std::size_t i = 0; i < n; ++i)
      v[i] = i + index;

    x = v;
  }
};


struct all_ones
{
  typedef bool result_type;

  template<typename T>
  bool operator()(T x)
  {
    typename T::iterator it = x.begin();
    typename T::iterator eit = x.end();

    for (; it != eit; ++it)
      if (*it != static_cast<std::size_t>(1))
        return false;

    return true;
  }
};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " n" << std::endl;
    exit(1);
  }

  typedef std::vector<size_t>    value_type;
  typedef array<value_type>      array_type;
  typedef array_view<array_type> view_type;

  const std::size_t n = atol(argv[1]);

  array_type c(n, value_type(n, 0));
  view_type v(c);

  // fill the array of vectors such that the j'th element of the i'th
  // vector has the value i+j
  map_func(fill_offset(), v, counting_view<size_t>(n));

  bool passed = true;

  // serially test each vector using iterators to check if
  // the values match the numbers from the fill
  if (get_location_id() == 0)
  {
    for (std::size_t i = 0; i < n; ++i)
    {
      array_type::reference x = c[i];

      array_type::reference::iterator it = x.begin();
      array_type::reference::iterator eit = x.end();

      for (std::size_t j = 0; it != eit; ++it, ++j)
        if (*it != i+j)
          passed = false;
    }
  }

  STAPL_TEST_REPORT(passed,
    "Remotely reading nested vector through iterators");

  // serially set each vector's values to 1
  if (get_location_id() == 0)
  {
    for (std::size_t i = 0; i < n; ++i)
    {
      array_type::reference x = c[i];

      array_type::reference::iterator it = x.begin();
      array_type::reference::iterator eit = x.end();

      for (; it != eit; ++it)
        *it = 1;
    }
  }

  rmi_fence();

  // check if all of the values are 1
  passed = all_of(v, all_ones());

  STAPL_TEST_REPORT(passed,
    "Remotely writing nested vector through iterators");

  // test that the distance between the first vector's begin and end is
  // the size of the vector
  array_type::reference::iterator it = c[0].begin();
  array_type::reference::iterator eit = c[0].end();

  stapl_bool b(static_cast<std::size_t>(std::distance(it, eit)) == n);
  passed = b.reduce();

  STAPL_TEST_REPORT(passed, "Distance of remote iterators");

  // test that the distance between the last vector's begin and its next
  // element is only 1
  it = c[n-1].begin();
  eit = ++c[n-1].begin();

  b = stapl_bool(std::distance(it, eit) == 1);
  passed = b.reduce();

  STAPL_TEST_REPORT(passed, "Partial distance of remote iterators");

  // test that the distance between the last vector's end and its prev
  // element is -1
  it = c[n-1].end();
  eit = --c[n-1].end();

  b = stapl_bool(std::distance(it, eit) == -1);
  passed = b.reduce();

  STAPL_TEST_REPORT(passed,  "Negative distance of remote iterators");

  return EXIT_SUCCESS;
}
