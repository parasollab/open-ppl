/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE lazy_storage
#include "utility.h"
#include <stapl/runtime/type_traits/lazy_storage.hpp>

template<typename T>
union wrapper
{
  stapl::lazy_storage<T> storage;
  int                    i;
};

struct empty
{ };

constexpr bool operator==(empty const&, empty const&) noexcept
{ return true; }

BOOST_AUTO_TEST_CASE( test_empty )
{
  typedef empty value_type;

  wrapper<value_type> u;

  u.storage.construct(value_type());
  value_type v = u.storage.moveout();

  BOOST_CHECK( v==value_type() );
}

BOOST_AUTO_TEST_CASE( test_int )
{
  typedef int value_type;

  wrapper<value_type> u;

  u.storage.construct(42);
  value_type v = u.storage.moveout();

  BOOST_CHECK_EQUAL( v, 42 );
}


class A
{
public:
  static int constructed;

  int i;

public:
  A(void)
  : i(42)
  { ++constructed; }

  A(A const& other)
  : i(other.i)
  { ++constructed; }

  ~A(void)
  { --constructed; }
};
int A::constructed = 0;

constexpr bool operator==(A const & a0, A const& a1)
{ return (a0.i==a1.i); }

BOOST_AUTO_TEST_CASE( test_A )
{
  typedef A value_type;

  BOOST_CHECK_EQUAL( A::constructed, 0 );
  {
    const value_type iv;
    BOOST_CHECK_EQUAL( A::constructed, 1 );

    wrapper<value_type> u;
    BOOST_CHECK_EQUAL( A::constructed, 1 );

    u.storage.construct(iv);
    BOOST_CHECK_EQUAL( A::constructed, 2 );

    value_type v = u.storage.moveout();
    BOOST_CHECK_EQUAL( A::constructed, 2 );

    BOOST_CHECK( v==iv );
  }
  BOOST_CHECK_EQUAL( A::constructed, 0 );
}
