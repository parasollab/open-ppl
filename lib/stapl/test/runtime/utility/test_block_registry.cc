/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE block_registry
#include "utility.h"
#include <stapl/runtime/utility/block_registry.hpp>
#include <vector>

using namespace stapl::runtime;

BOOST_AUTO_TEST_CASE( test_block_registry )
{
  const unsigned int id = 3;
  const unsigned int n  = 4;

  block_registry<unsigned int, int> registry(id, n);

  std::vector<int> values;
  std::vector<int> keys;
  for (int i=1; i<20; ++i) {
    values.push_back(i);
  }

  // insert key/data pair
  for (unsigned int i = 0; i<values.size(); ++i) {
    const auto key = registry.reserve_id();
    keys.push_back(key);
    registry.insert(key, values[i]);
  }

  // check if inserted
  for (unsigned int i = 0; i<values.size(); ++i) {
    const int v = registry.retrieve(keys[i]);
    BOOST_CHECK_EQUAL(values[i], v);
  }

  // erase half of them
  for (unsigned int i = 0; i<values.size()/2; ++i) {
    registry.erase(keys[i]);
  }

  // check if gone
  for (unsigned int i = 0; i<values.size()/2; ++i) {
    int v = 0;
    BOOST_CHECK_EQUAL(registry.try_retrieve(keys[i], v), false);
  }

  // check if still there
  for (unsigned int i = values.size()/2; i<values.size(); ++i) {
    const int v = registry.retrieve(keys[i]);
    BOOST_CHECK_EQUAL(values[i], v);
  }
}
