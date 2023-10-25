/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#define STAPL_RUNTIME_TEST_MODULE is_p_object
#include "utility.h"
#include <stapl/runtime/type_traits/is_p_object.hpp>
#include <stapl/runtime/p_object.hpp>

using namespace stapl;

class A
: public p_object
{ };


/// is_p_object

BOOST_AUTO_TEST_CASE( test_is_p_object )
{
  typedef p_object T;

  BOOST_CHECK_EQUAL(is_p_object<T>::value, true);
  BOOST_CHECK_EQUAL(is_p_object<const T>::value, true);
  BOOST_CHECK_EQUAL(is_p_object<volatile T>::value, true);
  BOOST_CHECK_EQUAL(is_p_object<const volatile T>::value, true);

  BOOST_CHECK_EQUAL(is_p_object<T&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile&>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile*>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile* const>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile* volatile>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object<T const volatile* volatile const>::value, false);
}


BOOST_AUTO_TEST_CASE( test_is_p_object_A )
{
  typedef A T;

  BOOST_CHECK_EQUAL(is_p_object<T>::value, true);
  BOOST_CHECK_EQUAL(is_p_object<const T>::value, true);
  BOOST_CHECK_EQUAL(is_p_object<volatile T>::value, true);
  BOOST_CHECK_EQUAL(is_p_object<const volatile T>::value, true);

  BOOST_CHECK_EQUAL(is_p_object<T&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile&>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile*>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile* const>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile* volatile>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object<T const volatile* volatile const>::value, false);
}

BOOST_AUTO_TEST_CASE( test_is_p_object_int )
{
  typedef int T;

  BOOST_CHECK_EQUAL(is_p_object<T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<const T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<volatile T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<const volatile T>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile&>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile*>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile* const>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile* volatile>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object<T const volatile* volatile const>::value, false);
}


BOOST_AUTO_TEST_CASE( test_is_p_object_int_array )
{
  typedef int T[10];

  BOOST_CHECK_EQUAL(is_p_object<T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<const T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<volatile T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<const volatile T>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile&>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile*>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile* const>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const volatile* volatile>::value, false);

  BOOST_CHECK_EQUAL(is_p_object<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T const* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object<T volatile* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object<T const volatile* volatile const>::value, false);
}


/// is_p_object_reference

BOOST_AUTO_TEST_CASE( test_is_p_object_reference )
{
  typedef p_object T;

  BOOST_CHECK_EQUAL(is_p_object_reference<T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<const T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<volatile T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<const volatile T>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T&>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const&>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile&>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const volatile&>::value, true);

  BOOST_CHECK_EQUAL(is_p_object_reference<T*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const volatile*>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile* const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const volatile* const>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile* volatile>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const volatile* volatile>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T volatile* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const volatile* volatile const>::value, false);
}


BOOST_AUTO_TEST_CASE( test_is_p_object_reference_A )
{
  typedef A T;

  BOOST_CHECK_EQUAL(is_p_object_reference<T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<const T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<volatile T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<const volatile T>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T&>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const&>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile&>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const volatile&>::value, true);

  BOOST_CHECK_EQUAL(is_p_object_reference<T*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const volatile*>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile* const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const volatile* const>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile* volatile>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const volatile* volatile>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T volatile* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const volatile* volatile const>::value, false);
}

BOOST_AUTO_TEST_CASE( test_is_p_object_reference_int )
{
  typedef int T;

  BOOST_CHECK_EQUAL(is_p_object_reference<T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<const T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<volatile T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<const volatile T>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const volatile&>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const volatile*>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile* const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const volatile* const>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile* volatile>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const volatile* volatile>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T volatile* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const volatile* volatile const>::value, false);
}


BOOST_AUTO_TEST_CASE( test_is_p_object_reference_int_array )
{
  typedef int T[10];

  BOOST_CHECK_EQUAL(is_p_object_reference<T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<const T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<volatile T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<const volatile T>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const volatile&>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const volatile*>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile* const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const volatile* const>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T const* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_reference<T volatile* volatile>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const volatile* volatile>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_reference<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T volatile* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_reference<T const volatile* volatile const>::value, false);
}



/// is_p_object_pointer

BOOST_AUTO_TEST_CASE( test_is_p_object_pointer )
{
  typedef p_object T;

  BOOST_CHECK_EQUAL(is_p_object_pointer<T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<const T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<volatile T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<const volatile T>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const volatile&>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T*>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const*>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile*>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const volatile*>::value, true);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T* const>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const* const>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile* const>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const volatile* const>::value, true);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T* volatile>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const* volatile>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile* volatile>::value, true);
  BOOST_CHECK_EQUAL(
    is_p_object_pointer<T const volatile* volatile>::value, true);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T* volatile const>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const* volatile const>::value, true);
  BOOST_CHECK_EQUAL(
    is_p_object_pointer<T volatile* volatile const>::value, true);
  BOOST_CHECK_EQUAL(
    is_p_object_pointer<T const volatile* volatile const>::value, true);
}


BOOST_AUTO_TEST_CASE( test_is_p_object_pointer_A )
{
  typedef A T;

  BOOST_CHECK_EQUAL(is_p_object_pointer<T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<const T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<volatile T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<const volatile T>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const volatile&>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T*>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const*>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile*>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const volatile*>::value, true);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T* const>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const* const>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile* const>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const volatile* const>::value, true);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T* volatile>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const* volatile>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile* volatile>::value, true);
  BOOST_CHECK_EQUAL(
    is_p_object_pointer<T const volatile* volatile>::value, true);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T* volatile const>::value, true);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const* volatile const>::value, true);
  BOOST_CHECK_EQUAL(
    is_p_object_pointer<T volatile* volatile const>::value, true);
  BOOST_CHECK_EQUAL(
    is_p_object_pointer<T const volatile* volatile const>::value, true);
}

BOOST_AUTO_TEST_CASE( test_is_p_object_pointer_int )
{
  typedef int T;

  BOOST_CHECK_EQUAL(is_p_object_pointer<T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<const T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<volatile T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<const volatile T>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const volatile&>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const volatile*>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const volatile* const>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile* volatile>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_pointer<T const volatile* volatile>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_pointer<T volatile* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_pointer<T const volatile* volatile const>::value, false);
}


BOOST_AUTO_TEST_CASE( test_is_p_object_pointer_int_array )
{
  typedef int T[10];

  BOOST_CHECK_EQUAL(is_p_object_pointer<T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<const T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<volatile T>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<const volatile T>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile&>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const volatile&>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile*>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const volatile*>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile* const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const volatile* const>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T volatile* volatile>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_pointer<T const volatile* volatile>::value, false);

  BOOST_CHECK_EQUAL(is_p_object_pointer<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_p_object_pointer<T const* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_pointer<T volatile* volatile const>::value, false);
  BOOST_CHECK_EQUAL(
    is_p_object_pointer<T const volatile* volatile const>::value, false);
}
