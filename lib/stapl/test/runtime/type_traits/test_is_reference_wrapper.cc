/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#define STAPL_RUNTIME_TEST_MODULE is_reference_wrapper
#include "utility.h"
#include <stapl/runtime/type_traits/is_reference_wrapper.hpp>
#include <functional>    // std::reference_wrapper
#include <boost/ref.hpp> // boost::reference_wrapper

using stapl::runtime::is_reference_wrapper;

template<typename T>
void test(T t)
{
  // ref(), cref()
  BOOST_CHECK_EQUAL(is_reference_wrapper<decltype(std::ref(t))>::value, true);
  BOOST_CHECK_EQUAL(is_reference_wrapper<decltype(std::cref(t))>::value, true);

  BOOST_CHECK_EQUAL(is_reference_wrapper<decltype(boost::ref(t))>::value, true);
  BOOST_CHECK_EQUAL(is_reference_wrapper<decltype(boost::cref(t))>::value,
                    true);

  // std::reference_wrapper
  BOOST_CHECK_EQUAL(
    is_reference_wrapper<std::reference_wrapper<T>>::value, true);
  BOOST_CHECK_EQUAL(
    is_reference_wrapper<std::reference_wrapper<const T>>::value, true);
  BOOST_CHECK_EQUAL(
    is_reference_wrapper<std::reference_wrapper<volatile T>>::value, true);
  BOOST_CHECK_EQUAL(
    is_reference_wrapper<std::reference_wrapper<const volatile T>>::value,
    true);

  // boost::reference_wrapper
  BOOST_CHECK_EQUAL(
    is_reference_wrapper<boost::reference_wrapper<T>>::value, true);
  BOOST_CHECK_EQUAL(
    is_reference_wrapper<boost::reference_wrapper<const T>>::value, true);
  BOOST_CHECK_EQUAL(
    is_reference_wrapper<boost::reference_wrapper<volatile T>>::value, true);
  BOOST_CHECK_EQUAL(
    is_reference_wrapper<boost::reference_wrapper<const volatile T>>::value,
    true);

  // other
  BOOST_CHECK_EQUAL(is_reference_wrapper<const T>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<volatile T>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<const volatile T>::value, false);

  BOOST_CHECK_EQUAL(is_reference_wrapper<T&>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<T const&>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<T volatile&>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<T const volatile&>::value, false);

  BOOST_CHECK_EQUAL(is_reference_wrapper<T*>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<const T*>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<volatile T*>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<const volatile T*>::value, false);

  BOOST_CHECK_EQUAL(is_reference_wrapper<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<const T* const>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<volatile T* const>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<const volatile T* const>::value,
                    false);

  BOOST_CHECK_EQUAL(is_reference_wrapper<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<const T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<volatile T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<const volatile T* volatile>::value,
                    false);

  BOOST_CHECK_EQUAL(is_reference_wrapper<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<const T* volatile const>::value,
                    false);
  BOOST_CHECK_EQUAL(is_reference_wrapper<volatile T* volatile const>::value,
                    false);
  BOOST_CHECK_EQUAL(
    is_reference_wrapper<const volatile T* volatile const>::value, false);
}


BOOST_AUTO_TEST_CASE( test_fundamental )
{
  test(int(10));
  test(double(42));
}


class A
{ };

struct B
{ };

union C
{ };

BOOST_AUTO_TEST_CASE( test_class )
{
  test(A());
  test(B());
  test(C());
}
