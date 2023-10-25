/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#define STAPL_RUNTIME_TEST_MODULE is_shared_ptr
#include "utility.h"
#include <stapl/runtime/type_traits/is_shared_ptr.hpp>
#include <functional>    // std::reference_wrapper
#include <boost/ref.hpp> // boost::reference_wrapper

using stapl::runtime::is_shared_ptr;

template<typename T>
void test(T t)
{
  BOOST_CHECK_EQUAL(is_shared_ptr<std::shared_ptr<T>>::value, true);
  BOOST_CHECK_EQUAL(is_shared_ptr<std::shared_ptr<const T>>::value, true);
  BOOST_CHECK_EQUAL(is_shared_ptr<std::shared_ptr<volatile T>>::value, true);
  BOOST_CHECK_EQUAL(is_shared_ptr<std::shared_ptr<const volatile T>>::value,
                    true);

  BOOST_CHECK_EQUAL(is_shared_ptr<boost::shared_ptr<T>>::value, true);
  BOOST_CHECK_EQUAL(is_shared_ptr<boost::shared_ptr<const T>>::value, true);
  BOOST_CHECK_EQUAL(is_shared_ptr<boost::shared_ptr<volatile T>>::value, true);
  BOOST_CHECK_EQUAL(is_shared_ptr<boost::shared_ptr<const volatile T>>::value,
                    true);

  BOOST_CHECK_EQUAL(is_shared_ptr<decltype(std::make_shared<T>(t))>::value,
                    true);

  // other
  BOOST_CHECK_EQUAL(is_shared_ptr<const T>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<volatile T>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<const volatile T>::value, false);

  BOOST_CHECK_EQUAL(is_shared_ptr<T&>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<T const&>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<T volatile&>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<T const volatile&>::value, false);

  BOOST_CHECK_EQUAL(is_shared_ptr<T*>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<const T*>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<volatile T*>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<const volatile T*>::value, false);

  BOOST_CHECK_EQUAL(is_shared_ptr<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<const T* const>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<volatile T* const>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<const volatile T* const>::value, false);

  BOOST_CHECK_EQUAL(is_shared_ptr<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<const T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<volatile T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<const volatile T* volatile>::value, false);

  BOOST_CHECK_EQUAL(is_shared_ptr<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<const T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<volatile T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_shared_ptr<const volatile T* volatile const>::value,
                    false);
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
