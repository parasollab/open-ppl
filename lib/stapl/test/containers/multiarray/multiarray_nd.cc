/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
#ifndef BOOST_PP_IS_ITERATING

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>

#include <test/algorithms/test_utils.h>
#include "../../test_report.hpp"

#include <boost/preprocessor/control/if.hpp>

using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout<< "usage: " << argv[0] << " n " <<std::endl;
    exit(1);

  }

  const size_t n = atoi(argv[1]);

#define BOOST_PP_ITERATION_LIMITS (2, 5)
  #define BOOST_PP_FILENAME_1     "test/containers/multiarray/multiarray_nd.cc"
#include BOOST_PP_ITERATE()

  return EXIT_SUCCESS;
}

#else // BOOST_PP_IS_ITERATING

#define this_iteration BOOST_PP_ITERATION()

#define IDENTITY(z, blah, data) data
#define MULTIPLY_fold(z, i, data) BOOST_PP_IF(i, *n, n)
#define FOR_iterators(z, i, data) BOOST_PP_CAT(data, i)

{
  typedef double                    value_type;
  typedef multiarray<this_iteration, value_type> multiarray_type;
  typedef multiarray_type::gid_type gid_type;

  do_once([&](void) {
    std::cout << "Testing " << this_iteration << "d multiarray." << std::endl;
  });

  multiarray_type c(
    stapl::make_tuple(BOOST_PP_ENUM(this_iteration, IDENTITY, n))
  );

  STAPL_TEST_REPORT(c.size() ==
    BOOST_PP_REPEAT(this_iteration, MULTIPLY_fold, ~), "Testing size");

#define BOOST_PP_LOCAL_MACRO(i) \
  for (size_t it ## i = 0; it ## i < n; ++it ## i)
#define BOOST_PP_LOCAL_LIMITS (0, BOOST_PP_SUB(this_iteration, 1))

  do_once([&](void) {
    #include BOOST_PP_LOCAL_ITERATE()
      c.set_element(
        gid_type(BOOST_PP_ENUM(this_iteration, FOR_iterators, it)), it0*n+it1);
  });

  STAPL_TEST_REPORT(true, "Testing set_element");


#define BOOST_PP_LOCAL_LIMITS (0, BOOST_PP_SUB(this_iteration, 1))
#define BOOST_PP_LOCAL_MACRO(i) \
  for (size_t it ## i = 0; it ## i < n; ++it ## i)


  bool passed = true;
  #include BOOST_PP_LOCAL_ITERATE()
    if (static_cast<size_t>(
      c.get_element(gid_type(BOOST_PP_ENUM(this_iteration,FOR_iterators,it))))
        != it0*n+it1)
      passed = false;

  // check if set_element test passed on all locations
  stapl_bool res(passed);
  bool global_result = res.reduce();
  STAPL_TEST_REPORT(global_result, "Testing get_element");

#define BOOST_PP_LOCAL_LIMITS (0, BOOST_PP_SUB(this_iteration, 1))
#define BOOST_PP_LOCAL_MACRO(i) \
  for (size_t it ## i = 0; it ## i < n; ++it ## i)

  passed = true;
  #include BOOST_PP_LOCAL_ITERATE()
    if (static_cast<size_t>(
      c[gid_type(BOOST_PP_ENUM(this_iteration, FOR_iterators, it))])
      != it0*n+it1)
      passed = false;

  // check if operator[] test passed on all locations
  res = stapl_bool(passed);
  global_result = res.reduce();
  STAPL_TEST_REPORT(global_result, "Testing operator[]");
}

#undef IDENTITY
#undef MULTIPLY_fold
#undef FOR_iterators
#undef this_iteration

#endif // BOOST_PP_IS_ITERATING
