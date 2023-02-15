/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#define STAPL_RUNTIME_TEST_MODULE has_define_type
#include "utility.h"
#include <stapl/runtime/type_traits/has_define_type.hpp>

using stapl::runtime::has_define_type;

// primitive type
BOOST_AUTO_TEST_CASE( test_has_define_type_int )
{
  typedef int T;

  BOOST_CHECK_EQUAL( has_define_type<T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T*>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* volatile>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const volatile>::value,
                     false );
}


// array type
BOOST_AUTO_TEST_CASE( test_has_define_type_int_array )
{
  typedef int T[10];

  BOOST_CHECK_EQUAL( has_define_type<T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T*>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* volatile>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const volatile>::value,
                     false );
}


// class without define_type()
class A
{ };

BOOST_AUTO_TEST_CASE( test_has_define_type_A )
{
  typedef A T;

  BOOST_CHECK_EQUAL( has_define_type<T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T*>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* volatile>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const volatile>::value,
                     false );
}


// class with define_type()
class B
{
public:
  void define_type(stapl::typer&)
  { }
};

BOOST_AUTO_TEST_CASE( test_has_define_type_B )
{
  typedef B T;

  BOOST_CHECK_EQUAL( has_define_type<T>::value,
                     true );
  BOOST_CHECK_EQUAL( has_define_type<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( has_define_type<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( has_define_type<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T*>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* volatile>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const volatile>::value,
                     false );
}


// class reimplements define_type()
class C
: public B
{
public:
  void define_type(stapl::typer&)
  { }
};

BOOST_AUTO_TEST_CASE( test_has_define_type_C )
{
  typedef C T;

  BOOST_CHECK_EQUAL( has_define_type<T>::value,
                     true );
  BOOST_CHECK_EQUAL( has_define_type<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( has_define_type<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( has_define_type<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T*>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* volatile>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const volatile>::value,
                     false );
}


// class inherits define_type()
class D
: public B
{ };

BOOST_AUTO_TEST_CASE( test_has_define_type_D )
{
  typedef D T;

  BOOST_CHECK_EQUAL( has_define_type<T>::value,
                     true );
  BOOST_CHECK_EQUAL( has_define_type<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( has_define_type<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( has_define_type<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T*>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* volatile>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const volatile>::value,
                     false );
}


// class with incorrect signature in define_type()
class E
{
  void define_type(void)
  { }
};

BOOST_AUTO_TEST_CASE( test_has_define_type_E )
{
  typedef E T;

  BOOST_CHECK_EQUAL( has_define_type<T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T*>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* volatile>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const volatile>::value,
                     false );
}


// class with deleted define_type()
class F
{
public:
  void define_type(void) = delete;
};

BOOST_AUTO_TEST_CASE( test_has_define_type_F )
{
  typedef F T;

  BOOST_CHECK_EQUAL( has_define_type<T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T*>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* volatile>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const volatile>::value,
                     false );
}


// class with private define_type()
class G
{
private:
  void define_type(void);
};

BOOST_AUTO_TEST_CASE( test_has_define_type_G )
{
  typedef G T;

  BOOST_CHECK_EQUAL( has_define_type<T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T*>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* volatile>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const volatile>::value,
                     false );
}


// class with external define_type()
class H
{ };

namespace stapl {

template<>
struct define_type_provider<H>
{
  class wrapper_H
  : public H
  {
  public:
    void define_type(typer&)
    { }
  };

  static wrapper_H& apply(H& t) noexcept
  { return static_cast<wrapper_H&>(t); }
};

} // namespace stapl

BOOST_AUTO_TEST_CASE( test_has_define_type_H )
{
  typedef H T;

  BOOST_CHECK_EQUAL( has_define_type<T>::value,
                     true );
  BOOST_CHECK_EQUAL( has_define_type<const T>::value,
                     true );
  BOOST_CHECK_EQUAL( has_define_type<volatile T>::value,
                     true );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T>::value,
                     true );

  BOOST_CHECK_EQUAL( has_define_type<T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T&>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T&>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T*>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T*>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* volatile>::value,
                     false );

  BOOST_CHECK_EQUAL( has_define_type<T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<volatile T* const volatile>::value,
                     false );
  BOOST_CHECK_EQUAL( has_define_type<const volatile T* const volatile>::value,
                     false );
}
