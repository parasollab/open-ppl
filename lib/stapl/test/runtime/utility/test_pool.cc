/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE pool<T>
#include "utility.h"
#include <stapl/runtime/utility/pool.hpp>
#include <vector>

using namespace stapl::runtime;


static int count = 0;


struct Value
{
  int val;

  Value(int v = 0)
  : val(v)
  { ++count; }

  ~Value(void)
  { --count; }
};


BOOST_AUTO_TEST_CASE( test_pool )
{
  const int N = 10;

  BOOST_CHECK_EQUAL(count, 0);

  std::vector<Value*> values;
  pool registry;
  registry.reset(sizeof(Value), 1, N);

  for (int j=0; j<5; ++j) {
    // get objects and use them
    for (int i=0; i<N; ++i) {
      void* p = registry.allocate();
      BOOST_REQUIRE(p!=static_cast<void*>(0));
      Value* const v = new(p) Value(i);
      values.push_back(v);
    }

    // check objects
    for (int i=0; i<N; ++i) {
      BOOST_CHECK_EQUAL(values[i]->val, i);
    }

    // remove objects
    for (int i=0; i<N; ++i) {
      BOOST_CHECK_EQUAL(values[i]->val, i);
      values[i]->~Value();
      registry.release(values[i]);
    }
    values.clear();
  }

  registry.purge();

  BOOST_CHECK_EQUAL(count, 0);
}


BOOST_AUTO_TEST_CASE( test_pool_expanding )
{
  const int N = 10;

  BOOST_CHECK_EQUAL(count, 0);

  std::vector<Value*> values;
  pool registry;

  for (int j=0; j<5; ++j) {
    const int M = (j+1)*N;
    registry.reset(sizeof(Value), N, M);

    // get objects and use them
    for (int i=0; i<M; ++i) {
      void* p = registry.allocate();
      BOOST_REQUIRE(p!=static_cast<void*>(0));
      Value* const v = new(p) Value(i);
      values.push_back(v);
    }

    // check objects
    for (int i=0; i<M; ++i) {
      BOOST_CHECK_EQUAL(values[i]->val, i);
    }

    // remove objects
    for (int i=0; i<M; ++i) {
      BOOST_CHECK_EQUAL(values[i]->val, i);
      values[i]->~Value();
      registry.release(values[i]);
    }
    values.clear();
  }

  registry.purge();

  BOOST_CHECK_EQUAL(count, 0);
}


BOOST_AUTO_TEST_CASE( test_pool_bounded )
{
  const int N = 10;

  BOOST_CHECK_EQUAL(count, 0);

  std::vector<Value*> values;
  pool registry;
  registry.reset(sizeof(Value), 1, N);

  for (int j=0; j<5; ++j) {
    // get objects and use them
    for (int i=0; i<N; ++i) {
      void* p = registry.allocate();
      BOOST_REQUIRE(p!=static_cast<void*>(0));
      Value* const v = new(p) Value(i);
      values.push_back(v);
    }

    // check objects
    for (int i=0; i<N; ++i) {
      BOOST_CHECK_EQUAL(values[i]->val, i);
    }

    // get another one
    void* p = registry.allocate();
    BOOST_REQUIRE(p==static_cast<void*>(0));

    // remove objects
    for (int i=0; i<N; ++i) {
      BOOST_CHECK_EQUAL(values[i]->val, i);
      values[i]->~Value();
      registry.release(values[i]);
    }
    values.clear();
  }

  registry.purge();

  BOOST_CHECK_EQUAL(count, 0);
}
