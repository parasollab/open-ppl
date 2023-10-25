/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE algorithms
#include "utility.h"
#include <stapl/runtime/utility/algorithm.hpp>

using namespace stapl::runtime;

BOOST_AUTO_TEST_CASE( test_find_index )
{
  std::vector<int> v1 = { 1, 2, 3, 4, 5, 6 };
  BOOST_CHECK_EQUAL( find_index(v1, 2), 1 );
  BOOST_CHECK_EQUAL( find_index(v1, 6), 5 );
  BOOST_CHECK_EQUAL( find_index(v1, 10), 7 );
}

BOOST_AUTO_TEST_CASE( test_all_unique )
{
  std::vector<int> v1 = { 1, 2, 3, 4, 5, 6 };
  BOOST_CHECK_EQUAL( all_unique(v1), true );

  v1 = { 1, 2, 3, 4, 5, 6, 1 };
  BOOST_CHECK_EQUAL( all_unique(v1), false );
}
