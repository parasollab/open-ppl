/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/numeric.hpp>

#include "../../test_report.hpp"

using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  size_t n = atoi(argv[1]);

  // setting up types for a static array
  typedef int                       value_type;
  typedef static_array<value_type>  array_type;

  array_type c(n);

  size_t sum = 0;
  size_t sum_ok = (n*(n-1))/2;

  for (size_t i = 0; i < n; ++i)
    c.set_element(i, i);

  for (size_t i = 0; i < n; ++i) {
    sum += c.get_element_split(i).get();
  }

  bool b = (sum == sum_ok);
  STAPL_TEST_REPORT(b ,"Testing static_array get_element");

  sum = 0;
  for (array_type::iterator it = c.begin(); it!=c.end(); ++it  ) {
    sum += *it;
  }

  b = (sum == sum_ok);
  STAPL_TEST_REPORT(b ,"Testing static_array *iterator");


  sum = 0;
  for (array_type::const_iterator it = c.begin(); it!=c.end(); ++it  ) {
    sum += *it;
  }

  b = (sum == sum_ok);

  sum = 0;
  for (array_type::const_iterator it = c.cbegin(); it!=c.cend(); ++it  ) {
    sum += *it;
  }

  b = (sum == sum_ok);
  STAPL_TEST_REPORT(b ,"Testing static_array *const_iterator");

  for (size_t i = 0; i < n; ++i)
    c[i] = i;

  sum = 0;
  for (size_t i = 0; i < n; ++i) {
    sum += c[i];
  }

  b = (sum == sum_ok);
  STAPL_TEST_REPORT(b ,"Testing static_array operator[]");

  b = true;

  if (c.front() != 0)
    b = false;

  STAPL_TEST_REPORT(b, "Testing static_array front")

  b = true;

  if (c.back() != (n - 1))
    b = false;

  STAPL_TEST_REPORT(b, "Testing static_array back")

  array_type d(n, 1);
  array_view<array_type> v(d);
  sum = accumulate(v, 0);

  STAPL_TEST_REPORT(sum == n ,"Testing default value constructor");

  return EXIT_SUCCESS;
}
