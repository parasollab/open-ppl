/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE is_basic
#include "utility.h"
#include <stapl/runtime/type_traits/is_basic.hpp>
#include <typeinfo>
#include <tuple>
#include <utility>

using stapl::is_basic;

template<typename T>
void test_type(bool b)
{
  // T
  BOOST_CHECK_EQUAL(is_basic<T>::value, b);
  BOOST_CHECK_EQUAL(is_basic<const T>::value, b);
  BOOST_CHECK_EQUAL(is_basic<volatile T>::value, b);
  BOOST_CHECK_EQUAL(is_basic<const volatile T>::value, b);
  BOOST_CHECK_EQUAL(is_basic<T>::value, b);
  BOOST_CHECK_EQUAL(is_basic<const T>::value, b);
  BOOST_CHECK_EQUAL(is_basic<volatile T>::value, b);
  BOOST_CHECK_EQUAL(is_basic<const volatile T>::value, b);

  // T&
  BOOST_CHECK_EQUAL(is_basic<T&>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const T&>::value, false);
  BOOST_CHECK_EQUAL(is_basic<volatile T&>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const volatile T&>::value, false);
  BOOST_CHECK_EQUAL(is_basic<T&>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const T&>::value, false);
  BOOST_CHECK_EQUAL(is_basic<volatile T&>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const volatile T&>::value, false);

  // T*
  BOOST_CHECK_EQUAL(is_basic<T*>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const T*>::value, false);
  BOOST_CHECK_EQUAL(is_basic<volatile T*>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const volatile T*>::value, false);
  BOOST_CHECK_EQUAL(is_basic<T*>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const T*>::value, false);
  BOOST_CHECK_EQUAL(is_basic<volatile T*>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const volatile T*>::value, false);

  // T* const
  BOOST_CHECK_EQUAL(is_basic<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const T* const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<volatile T* const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const volatile T* const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<T* const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const T* const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<volatile T* const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const volatile T* const>::value, false);

  // T* volatile
  BOOST_CHECK_EQUAL(is_basic<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_basic<volatile T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const volatile T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_basic<T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_basic<volatile T* volatile>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const volatile T* volatile>::value, false);

  // T* const volatile
  BOOST_CHECK_EQUAL(is_basic<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<volatile T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const volatile T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<volatile T* volatile const>::value, false);
  BOOST_CHECK_EQUAL(is_basic<const volatile T* volatile const>::value, false);
}


// empty struct: basic
struct basic1
{ };

// enum: basic
enum basic2
{ Romanian, French, American, Greek, Dutch, Italian };

// empty union: basic
union basic3
{ };

STAPL_IS_BASIC_TYPE(basic3) // unions cannot be detected automatically

// struct of basic types: basic
struct basic4
{
  typedef std::tuple<int, float, basic2> member_types;

  int    m_i;
  float  m_f;
  basic2 m_e;
};

// struct that inherits from basic type: basic
struct basic5
: public basic4
{
  typedef std::tuple<basic4> member_types;
};

// struct that inherits from basic type and has basic members: basic
struct basic6
: public basic5
{
  typedef std::tuple<basic5, int, basic4> member_types;

  int    m_i;
  basic4 m_b;
};

// union of basic types: basic
union basic7
{
  typedef std::tuple<int, float> member_types;

  int   m_i;
  float m_f;
};

// struct that declared explicitly basic: basic
struct basic8
{
  union
  {
    unsigned int m_ui;
    double       m_d;
  };
  int* m_i;
  char c;
};

STAPL_IS_BASIC_TYPE(basic8)

// struct with arrays of basic types: basic
struct basic9
{
  typedef std::tuple<int[5], char[5][7], basic6[10][100]> member_types;

  int    m_a[5];
  char   m_c[5][7];
  basic6 m_b[10][100];
};



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
  typedef std::tuple<non_basic2> member_types;

  non_basic2 m_nb;
};

// inherits from non-basic: non-basic
struct non_basic4
: public non_basic1
{
  typedef std::tuple<non_basic1> member_types;
};

// inherits from non-basic and has basic data member: non-basic
struct non_basic5
: public non_basic1
{
  typedef std::tuple<non_basic1, int> member_types;

  int m_i;
};

// inherits from basic and has non-basic data member: non-basic
struct non_basic6
: public basic4
{
  typedef std::tuple<basic4, non_basic3> member_types;

  non_basic3 m_b;
};

// has reference: non-basic
struct non_basic7
{
  typedef std::tuple<int&> member_types;

  int& m_i;

  explicit non_basic7(int& t)
  : m_i(t)
  { }
};

void foo(void)
{ }

struct A
{
  void foo(void)
  { }
};

BOOST_AUTO_TEST_CASE( test_fundamentals )
{
  test_type<int>(true);
  test_type<int[]>(true);
  test_type<int[10]>(true);
  test_type<int[10][11]>(true);
  test_type<int[10][11][12]>(true);

  test_type<double>(true);
  test_type<double[]>(true);
  test_type<double[10]>(true);
  test_type<double[10][11]>(true);
  test_type<double[10][11][12]>(true);

  test_type<std::nullptr_t>(true);

  test_type<decltype(&foo)>(true);

  void (*pf)(void) = &foo;
  test_type<decltype(&pf)>(false);

  test_type<decltype(&A::foo)>(true);
  void (A::*pmf)(void) = &A::foo;
  test_type<decltype(&pmf)>(false);
}


BOOST_AUTO_TEST_CASE( test_basic_structures )
{
  test_type<basic1>(true);
  test_type<basic1[]>(true);
  test_type<basic1[10]>(true);
  test_type<basic1[10][11]>(true);
  test_type<basic1[10][11][12]>(true);

  test_type<basic2>(true);
  test_type<basic2[]>(true);
  test_type<basic2[10]>(true);
  test_type<basic2[10][11]>(true);
  test_type<basic2[10][11][12]>(true);

  test_type<basic3>(true);
  test_type<basic3[]>(true);
  test_type<basic3[10]>(true);
  test_type<basic3[10][11]>(true);
  test_type<basic3[10][11][12]>(true);

  test_type<basic4>(true);
  test_type<basic4[]>(true);
  test_type<basic4[10]>(true);
  test_type<basic4[10][11]>(true);
  test_type<basic4[10][11][12]>(true);

  test_type<basic5>(true);
  test_type<basic5[]>(true);
  test_type<basic5[10]>(true);
  test_type<basic5[10][11]>(true);
  test_type<basic5[10][11][12]>(true);

  test_type<basic6>(true);
  test_type<basic6[]>(true);
  test_type<basic6[10]>(true);
  test_type<basic6[10][11]>(true);
  test_type<basic6[10][11][12]>(true);

  test_type<basic7>(true);
  test_type<basic7[]>(true);
  test_type<basic7[10]>(true);
  test_type<basic7[10][11]>(true);
  test_type<basic7[10][11][12]>(true);

  test_type<basic8>(true);
  test_type<basic8[]>(true);
  test_type<basic8[10]>(true);
  test_type<basic8[10][11]>(true);
  test_type<basic8[10][11][12]>(true);

  test_type<basic9>(true);
  test_type<basic9[]>(true);
  test_type<basic9[10]>(true);
  test_type<basic9[10][11]>(true);
  test_type<basic9[10][11][12]>(true);
}


BOOST_AUTO_TEST_CASE( test_non_basic_structures )
{
  test_type<non_basic1>(false);
  test_type<non_basic1[]>(false);
  test_type<non_basic1[10]>(false);
  test_type<non_basic1[10][11]>(false);
  test_type<non_basic1[10][11][12]>(false);

  test_type<non_basic2>(false);
  test_type<non_basic2[]>(false);
  test_type<non_basic2[10]>(false);
  test_type<non_basic2[10][11]>(false);
  test_type<non_basic2[10][11][12]>(false);

  test_type<non_basic3>(false);
  test_type<non_basic3[]>(false);
  test_type<non_basic3[10]>(false);
  test_type<non_basic3[10][11]>(false);
  test_type<non_basic3[10][11][12]>(false);

  test_type<non_basic4>(false);
  test_type<non_basic4[]>(false);
  test_type<non_basic4[10]>(false);
  test_type<non_basic4[10][11]>(false);
  test_type<non_basic4[10][11][12]>(false);

  test_type<non_basic5>(false);
  test_type<non_basic5[]>(false);
  test_type<non_basic5[10]>(false);
  test_type<non_basic5[10][11]>(false);
  test_type<non_basic5[10][11][12]>(false);

  test_type<non_basic6>(false);
  test_type<non_basic6[]>(false);
  test_type<non_basic6[10]>(false);
  test_type<non_basic6[10][11]>(false);
  test_type<non_basic6[10][11][12]>(false);

  test_type<non_basic7>(false);
  test_type<non_basic7[]>(false);
  test_type<non_basic7[10]>(false);
  test_type<non_basic7[10][11]>(false);
  test_type<non_basic7[10][11][12]>(false);
}

BOOST_AUTO_TEST_CASE( test_basic_pair )
{
  typedef std::pair<int, int> pair1;

  test_type<pair1>(true);
  test_type<pair1[]>(true);
  test_type<pair1[10]>(true);
  test_type<pair1[10][11]>(true);
  test_type<pair1[10][11][12]>(true);

  typedef std::pair<int[10], int> pair2;

  test_type<pair2>(true);
  test_type<pair2[]>(true);
  test_type<pair2[10]>(true);
  test_type<pair2[10][11]>(true);
  test_type<pair2[10][11][12]>(true);

  typedef std::pair<int, int[10]> pair3;

  test_type<pair3>(true);
  test_type<pair3[]>(true);
  test_type<pair3[10]>(true);
  test_type<pair3[10][11]>(true);
  test_type<pair3[10][11][12]>(true);

  typedef std::pair<basic9, basic8> pair4;

  test_type<pair4>(true);
  test_type<pair4[]>(true);
  test_type<pair4[10]>(true);
  test_type<pair4[10][11]>(true);
  test_type<pair4[10][11][12]>(true);

  typedef std::pair<basic2, basic6> pair5;

  test_type<pair5>(true);
  test_type<pair5[]>(true);
  test_type<pair5[10]>(true);
  test_type<pair5[10][11]>(true);
  test_type<pair5[10][11][12]>(true);

  typedef std::pair<std::pair<basic2, basic6>, std::pair<int,int>> pair6;

  test_type<pair6>(true);
  test_type<pair6[]>(true);
  test_type<pair6[10]>(true);
  test_type<pair6[10][11]>(true);
  test_type<pair6[10][11][12]>(true);
}


BOOST_AUTO_TEST_CASE( test_non_basic_pair )
{
  typedef std::pair<int*, int> pair1;

  test_type<pair1>(false);
  test_type<pair1[]>(false);
  test_type<pair1[10]>(false);
  test_type<pair1[10][11]>(false);
  test_type<pair1[10][11][12]>(false);

  typedef std::pair<int, int*> pair2;

  test_type<pair2>(false);
  test_type<pair2[]>(false);
  test_type<pair2[10]>(false);
  test_type<pair2[10][11]>(false);
  test_type<pair2[10][11][12]>(false);

  typedef std::pair<non_basic1, basic1> pair3;

  test_type<pair3>(false);
  test_type<pair3[]>(false);
  test_type<pair3[10]>(false);
  test_type<pair3[10][11]>(false);
  test_type<pair3[10][11][12]>(false);

  typedef std::pair<non_basic7[10], basic8> pair4;

  test_type<pair4>(false);
  test_type<pair4[]>(false);
  test_type<pair4[10]>(false);
  test_type<pair4[10][11]>(false);
  test_type<pair4[10][11][12]>(false);

  typedef std::pair<non_basic2, non_basic6> pair5;

  test_type<pair5>(false);
  test_type<pair5[]>(false);
  test_type<pair5[10]>(false);
  test_type<pair5[10][11]>(false);
  test_type<pair5[10][11][12]>(false);

  typedef std::pair<std::pair<basic2, basic6>, std::pair<int*,int>> pair6;

  test_type<pair6>(false);
  test_type<pair6[]>(false);
  test_type<pair6[10]>(false);
  test_type<pair6[10][11]>(false);
  test_type<pair6[10][11][12]>(false);
}

BOOST_AUTO_TEST_CASE( test_basic_tuple )
{
  typedef std::tuple<int, int, int> tuple1;

  test_type<tuple1>(true);
  test_type<tuple1[]>(true);
  test_type<tuple1[10]>(true);
  test_type<tuple1[10][11]>(true);
  test_type<tuple1[10][11][12]>(true);

  typedef std::tuple<int[10], int, char[2]> tuple2;

  test_type<tuple2>(true);
  test_type<tuple2[]>(true);
  test_type<tuple2[10]>(true);
  test_type<tuple2[10][11]>(true);
  test_type<tuple2[10][11][12]>(true);

  typedef std::tuple<int, int[10], int, double> tuple3;

  test_type<tuple3>(true);
  test_type<tuple3[]>(true);
  test_type<tuple3[10]>(true);
  test_type<tuple3[10][11]>(true);
  test_type<tuple3[10][11][12]>(true);

  typedef std::tuple<basic9, basic8, basic6, basic7> tuple4;

  test_type<tuple4>(true);
  test_type<tuple4[]>(true);
  test_type<tuple4[10]>(true);
  test_type<tuple4[10][11]>(true);
  test_type<tuple4[10][11][12]>(true);

  typedef std::tuple<basic2, basic6, basic1> tuple5;

  test_type<tuple5>(true);
  test_type<tuple5[]>(true);
  test_type<tuple5[10]>(true);
  test_type<tuple5[10][11]>(true);
  test_type<tuple5[10][11][12]>(true);

  typedef std::tuple<
            std::tuple<basic2, basic6, basic7>,
            std::tuple<int,int,double>
          > tuple6;

  test_type<tuple6>(true);
  test_type<tuple6[]>(true);
  test_type<tuple6[10]>(true);
  test_type<tuple6[10][11]>(true);
  test_type<tuple6[10][11][12]>(true);
}


BOOST_AUTO_TEST_CASE( test_non_basic_tuple )
{
  typedef std::tuple<int*, int> tuple1;

  test_type<tuple1>(false);
  test_type<tuple1[]>(false);
  test_type<tuple1[10]>(false);
  test_type<tuple1[10][11]>(false);
  test_type<tuple1[10][11][12]>(false);

  typedef std::tuple<int, int*> tuple2;

  test_type<tuple2>(false);
  test_type<tuple2[]>(false);
  test_type<tuple2[10]>(false);
  test_type<tuple2[10][11]>(false);
  test_type<tuple2[10][11][12]>(false);

  typedef std::tuple<non_basic1, basic1, double> tuple3;

  test_type<tuple3>(false);
  test_type<tuple3[]>(false);
  test_type<tuple3[10]>(false);
  test_type<tuple3[10][11]>(false);
  test_type<tuple3[10][11][12]>(false);

  typedef std::tuple<non_basic7[10], basic8, float[10]> tuple4;

  test_type<tuple4>(false);
  test_type<tuple4[]>(false);
  test_type<tuple4[10]>(false);
  test_type<tuple4[10][11]>(false);
  test_type<tuple4[10][11][12]>(false);

  typedef std::tuple<non_basic2, non_basic6> tuple5;

  test_type<tuple5>(false);
  test_type<tuple5[]>(false);
  test_type<tuple5[10]>(false);
  test_type<tuple5[10][11]>(false);
  test_type<tuple5[10][11][12]>(false);

  typedef std::tuple<
            std::tuple<basic2, int, basic6>,
            std::tuple<int*,int>
          > tuple6;

  test_type<tuple6>(false);
  test_type<tuple6[]>(false);
  test_type<tuple6[10]>(false);
  test_type<tuple6[10][11]>(false);
  test_type<tuple6[10][11][12]>(false);
}
