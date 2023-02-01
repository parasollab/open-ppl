/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE is_movable
#include "utility.h"
#include <stapl/runtime/type_traits/is_movable.hpp>
#include <array>
#include <functional>
#include <tuple>
#include <utility>
#include <vector>

using stapl::runtime::is_movable;

template<typename T>
void test_type(bool b)
{
  // T
  BOOST_CHECK_EQUAL(is_movable<T>::value, b);
  BOOST_CHECK_EQUAL(is_movable<const T>::value, b);
  BOOST_CHECK_EQUAL(is_movable<volatile T>::value, b);
  BOOST_CHECK_EQUAL(is_movable<const volatile T>::value, b);
  BOOST_CHECK_EQUAL(is_movable<T>::value, b);
  BOOST_CHECK_EQUAL(is_movable<const T>::value, b);
  BOOST_CHECK_EQUAL(is_movable<volatile T>::value, b);
  BOOST_CHECK_EQUAL(is_movable<const volatile T>::value, b);

  // T&
  BOOST_CHECK_EQUAL(is_movable<T&>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const T&>::value, false);
  BOOST_CHECK_EQUAL(is_movable<volatile T&>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const volatile T&>::value, false);
  BOOST_CHECK_EQUAL(is_movable<T&>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const T&>::value, false);
  BOOST_CHECK_EQUAL(is_movable<volatile T&>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const volatile T&>::value, false);

  // T*
  BOOST_CHECK_EQUAL(is_movable<T*>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const T*>::value, false);
  BOOST_CHECK_EQUAL(is_movable<volatile T*>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const volatile T*>::value, false);
  BOOST_CHECK_EQUAL(is_movable<T*>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const T*>::value, false);
  BOOST_CHECK_EQUAL(is_movable<volatile T*>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const volatile T*>::value, false);

  // T* const
  BOOST_CHECK_EQUAL(is_movable<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const T* const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<volatile T* const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const volatile T* const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const T* const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<volatile T* const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const volatile T* const>::value, false);

  // T* volatile
  BOOST_CHECK_EQUAL(is_movable<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_movable<volatile T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const volatile T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_movable<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_movable<volatile T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const volatile T* volatile>::value, false);

  // T* const volatile
  BOOST_CHECK_EQUAL(is_movable<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<volatile T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const volatile T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<volatile T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_movable<const volatile T* volatile const>::value, false);
}


void foo(void)
{ }

struct A
{
  void foo(void)
  { }
};

BOOST_AUTO_TEST_CASE( test_fundamentals )
{
  test_type<int>(false);
  test_type<double>(false);
  test_type<std::nullptr_t>(false);

  test_type<decltype(&foo)>(false);
  void (*f)(void) = &foo;
  test_type<decltype(&f)>(false);

  test_type<decltype(&A::foo)>(false);
  void (A::*pmf)(void) = &A::foo;
  test_type<decltype(&pmf)>(false);
}


// empty struct: basic
struct basic1
{ };

// struct of basic types: basic
struct basic2
{
  typedef std::tuple<int, float, basic1> member_types;

  int    m_i;
  float  m_f;
  basic1 m_e;
};

BOOST_AUTO_TEST_CASE( test_basic_structures )
{
  test_type<basic1>(false);
  test_type<basic2>(false);
}



// non-empty struct: non-basic
struct non_basic1
{
  int i;
};

// POD struct of non-basic types: non-basic
struct non_basic2
{
  typedef std::tuple<int*, char> member_types;

  int *m_i;
  char m_c;
};

// non-basic data members: non-basic
struct non_basic3
{
  int *m_i;
  char m_c;
};

BOOST_AUTO_TEST_CASE( test_non_basic_structures )
{
  test_type<non_basic1>(false);
  test_type<non_basic2>(false);
  test_type<non_basic3>(false);
}



BOOST_AUTO_TEST_CASE( test_pair )
{
  typedef std::pair<int, int> pair1;
  test_type<pair1>(false);

  typedef std::pair<int[10], int> pair2;
  test_type<pair2>(false);

  typedef std::pair<int, int[10]> pair3;
  test_type<pair3>(false);

  typedef std::pair<basic1, basic2> pair4;
  test_type<pair4>(false);

  typedef std::pair<std::pair<basic1, basic2>, std::pair<int,int>> pair5;
  test_type<pair5>(false);

  typedef std::pair<int*, int> pair6;
  test_type<pair6>(false);

  typedef std::pair<std::vector<int>, int> pair7;
  test_type<pair7>(true);

  typedef std::pair<int, std::pair<std::vector<int>, int>> pair8;
  test_type<pair8>(true);
}



BOOST_AUTO_TEST_CASE( test_tuple )
{
  typedef std::tuple<int, int, int> tuple1;
  test_type<tuple1>(false);

  typedef std::tuple<int[10], int, char[2]> tuple2;
  test_type<tuple2>(false);

  typedef std::tuple<basic1, basic2, non_basic1, non_basic2> tuple4;
  test_type<tuple4>(false);

  typedef std::tuple<int, std::vector<double>> tuple5;
  test_type<tuple5>(true);

  typedef std::tuple<int, std::tuple<std::vector<int>, int>> tuple6;
  test_type<tuple6>(true);

  typedef std::tuple<int> tuple7;
  test_type<tuple7>(false);

  typedef std::tuple<std::vector<int>> tuple8;
  test_type<tuple8>(true);
}



BOOST_AUTO_TEST_CASE( test_std_array )
{
  typedef std::array<int, 5> array1;
  test_type<array1>(false);

  typedef std::array<std::vector<int>, 5> array2;
  test_type<array2>(true);
}


// wrapper around std::vector<int>
struct vector1_struct
{
  std::vector<int> v;
};

// wrapper around std::vector<int> without move constructor
struct vector2_struct
{
  std::vector<int> v;

  vector2_struct(void) = default;
  vector2_struct(vector2_struct const&) = default;
  vector2_struct(vector2_struct&&) = delete;
};

// wrapper around wrappers of std::vector<int>
struct vector3_struct
{
  vector1_struct v1;
  vector2_struct v2;
};

BOOST_AUTO_TEST_CASE( test_vector )
{
  test_type<std::vector<int>>(true);
  test_type<std::tuple<std::vector<int>, int>>(true);
  test_type<vector1_struct>(true);
  test_type<vector2_struct>(false);
  test_type<vector3_struct>(true);
}

BOOST_AUTO_TEST_CASE( test_reference_wrapper )
{
  std::vector<int> v;
  test_type<decltype(std::ref(v))>(false);
  test_type<decltype(std::cref(v))>(false);
}


BOOST_AUTO_TEST_CASE( test_pointers )
{
  test_type<int*>(false);
  test_type<std::array<std::vector<int>, 5>*>(false);
  test_type<std::vector<int>*>(false);
  test_type<vector1_struct*>(false);
  test_type<vector3_struct*>(false);
}
