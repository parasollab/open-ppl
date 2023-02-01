/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE type_id
#include "utility.h"
#include <stapl/runtime/type_traits/type_id.hpp>
#include <stapl/runtime/type_traits/polymorphic.hpp>

struct A
{ };

struct B
{ };

class Base
{
public:
  virtual ~Base(void) = default;

  STAPL_POLYMORPHIC_TYPE()
};

class Derived1
: public Base
{
public:
  STAPL_POLYMORPHIC_TYPE()
};

class Derived2
: public Base
{
public:
  STAPL_POLYMORPHIC_TYPE()
};


using namespace stapl;

BOOST_AUTO_TEST_CASE( test_equal )
{
  A a, b;
  BOOST_CHECK_EQUAL(get_type_id<A>(), get_type_id(a));
  BOOST_CHECK_EQUAL(get_type_id(a),   get_type_id(b));
}

BOOST_AUTO_TEST_CASE( test_not_equal )
{
  A a;
  B b;
  BOOST_CHECK_NE(get_type_id<A>(), get_type_id<B>());
  BOOST_CHECK_NE(get_type_id(a),   get_type_id(b));
}

BOOST_AUTO_TEST_CASE( test_polymorphic )
{
  BOOST_CHECK_NE(get_type_id<Derived1>(), get_type_id<Base>());
  BOOST_CHECK_NE(get_type_id<Derived2>(), get_type_id<Base>());
  BOOST_CHECK_NE(get_type_id<Derived1>(), get_type_id<Derived2>());
}

BOOST_AUTO_TEST_CASE( test_polymorphic_equal )
{
  Derived1 a, b;
  BOOST_CHECK_NE(get_type_id<Base>(),        get_type_id(a));
  BOOST_CHECK_EQUAL(get_type_id<Derived1>(), get_type_id(a));
  BOOST_CHECK_EQUAL(get_type_id(a),          get_type_id(b));
}

BOOST_AUTO_TEST_CASE( test_polymorphic_not_equal )
{
  Derived1 a;
  Derived2 b;
  BOOST_CHECK_NE(get_type_id<Base>(),        get_type_id(a));
  BOOST_CHECK_NE(get_type_id<Base>(),        get_type_id(b));
  BOOST_CHECK_EQUAL(get_type_id<Derived1>(), get_type_id(a));
  BOOST_CHECK_EQUAL(get_type_id<Derived2>(), get_type_id(b));
  BOOST_CHECK_NE(get_type_id<Derived1>(),    get_type_id<Derived2>());
  BOOST_CHECK_NE(get_type_id(a),             get_type_id(b));
}
