/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE is_non_commutative
#include "utility.h"
#include <stapl/runtime/type_traits/is_non_commutative.hpp>
#include <functional>

using namespace stapl;

template<typename T>
void test_type(T, const bool b)
{
  // T
  BOOST_CHECK_EQUAL(is_non_commutative<T>::value,
                    b);
  BOOST_CHECK_EQUAL(is_non_commutative<const T>::value,
                    b);
  BOOST_CHECK_EQUAL(is_non_commutative<volatile T>::value,
                    b);
  BOOST_CHECK_EQUAL(is_non_commutative<const volatile T>::value,
                    b);
  BOOST_CHECK_EQUAL(is_non_commutative<T>::value,
                    b);
  BOOST_CHECK_EQUAL(is_non_commutative<const T>::value,
                    b);
  BOOST_CHECK_EQUAL(is_non_commutative<volatile T>::value,
                    b);
  BOOST_CHECK_EQUAL(is_non_commutative<const volatile T>::value,
                    b);

  // T&
  BOOST_CHECK_EQUAL(is_non_commutative<T&>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const T&>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<volatile T&>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const volatile T&>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<T&>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const T&>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<volatile T&>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const volatile T&>::value,
                    false);

  // T*
  BOOST_CHECK_EQUAL(is_non_commutative<T*>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const T*>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<volatile T*>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const volatile T*>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<T*>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const T*>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<volatile T*>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const volatile T*>::value,
                    false);

  // T* const
  BOOST_CHECK_EQUAL(is_non_commutative<T* const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const T* const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<volatile T* const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const volatile T* const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<T* const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const T* const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<volatile T* const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const volatile T* const>::value,
                    false);

  // T* volatile
  BOOST_CHECK_EQUAL(is_non_commutative<T* volatile>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const T* volatile>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<volatile T* volatile>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const volatile T* volatile>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<T* volatile>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const T* volatile>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<volatile T* volatile>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const volatile T* volatile>::value,
                    false);

  // T* const volatile
  BOOST_CHECK_EQUAL(is_non_commutative<T* volatile const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const T* volatile const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<volatile T* volatile const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const volatile T* volatile const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<T* volatile const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const T* volatile const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<volatile T* volatile const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_non_commutative<const volatile T* volatile const>::value,
                    false);
}

constexpr int foo(int x, int y)
{
  return (x*y);
}


BOOST_AUTO_TEST_CASE( test_commutative )
{
  test_type(std::plus<int>(), false);
}

BOOST_AUTO_TEST_CASE( test_non_commutative )
{
  test_type(non_commutative(std::plus<int>()), true);
}

BOOST_AUTO_TEST_CASE( test_commutative_function )
{
  test_type(&foo, false);
}

BOOST_AUTO_TEST_CASE( test_non_commutative_function )
{
  test_type(non_commutative(&foo), true);
}

