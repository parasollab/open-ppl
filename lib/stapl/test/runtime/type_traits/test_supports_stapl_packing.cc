/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#define STAPL_RUNTIME_TEST_MODULE supports_stapl_packing
#include "utility.h"
#include <stapl/runtime/type_traits/supports_stapl_packing.hpp>
#include <stapl/runtime/serialization.hpp>
#include <stapl/runtime/p_object.hpp>
#include <tuple>

using stapl::runtime::supports_stapl_packing;

// basic type -- supports STAPL packing

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_int )
{
  typedef int T;

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}

// function type -- supports STAPL packing

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_fn_ptr )
{
  typedef void (*T)(int,int);

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}

// p_object -- supports STAPL packing

class A
: public stapl::p_object
{ };

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_A )
{
  typedef A T;

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}


// regular class -- does not support STAPL packing

class B
{
private:
  int i;
public:
  B(void)
  : i(0)
  { }
  int get(void) const
  { return i; }
};

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_B )
{
  typedef B T;

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}


// class with define_type -- supports STAPL packing

class C
{
public:
  void define_type(stapl::typer&)
  { }
};

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_C )
{
  typedef C T;

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}


// classes with member types list -- support STAPL packing

struct D
{ };

struct E
{
  typedef std::tuple<int, D> member_types;

  int i;
  D   d;
};

struct F
{
  typedef std::tuple<int*, E> member_types;
  int *i;
  E   e;
};

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_D )
{
  typedef D T;

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_E )
{
  typedef E T;

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_F )
{
  typedef F T;

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}


// user-set POD

struct G
{
  int i;
};
STAPL_IS_BASIC_TYPE(G)

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_G )
{
  typedef G T;

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}

// empty class
struct H
{ };

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_H )
{
  typedef H T;

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}


// typer_traits specialization
struct I { int *i; };

template<typename T>
struct I1
{
  T t;
};

template<typename T1, typename T2>
struct I2
{
  T1 t1;
  T2 t2;
};

template<typename T1, typename T2, typename T3>
struct I3
{
  T1 t1;
  T2 t2;
  T3 t3;
};

namespace stapl {

template<>
class typer_traits<I>
{ };

template<typename T>
class typer_traits<I1<T> >
{ };

template<typename T1, typename T2>
class typer_traits<I2<T1, T2> >
{ };

template<typename T1, typename T2, typename T3>
class typer_traits<I3<T1, T2, T3> >
{ };

template<>
struct typer_traits_specialization<I>
: public std::true_type
{ };

template<typename T>
struct typer_traits_specialization<I1<T> >
: public std::true_type
{ };

template<typename T1, typename T2>
struct typer_traits_specialization<I2<T1, T2> >
: public std::true_type
{ };

template<typename T1, typename T2, typename T3>
struct typer_traits_specialization<I3<T1,T2,T3> >
: public std::true_type
{ };

} // namespace stapl

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_I )
{
  typedef I T;

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_I1 )
{
  typedef I1<int> T;

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_I2 )
{
  typedef I2<int,int> T;

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}

BOOST_AUTO_TEST_CASE( test_supports_stapl_packing_I3 )
{
  typedef I3<int,int,int> T;

  BOOST_CHECK_EQUAL( supports_stapl_packing<T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T*>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T*>::value,
                     true );

  BOOST_CHECK_EQUAL( supports_stapl_packing<T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<volatile T* const>::value,
                     true );
  BOOST_CHECK_EQUAL( supports_stapl_packing<const volatile T* const>::value,
                     true );
}
