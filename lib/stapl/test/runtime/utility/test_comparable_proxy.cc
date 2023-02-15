/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE comparable_proxy
#include "utility.h"
#include <stapl/runtime/utility/comparable_proxy.hpp>
#include <tuple>
#include <utility>

using stapl::runtime::comparable_proxy;


struct empty_A
{ };


struct empty_B
{ };


BOOST_AUTO_TEST_CASE( test_same_empty )
{
  empty_A a1, a2;
  comparable_proxy ca1(a1), ca2(a2);
  BOOST_REQUIRE(ca1==a1);
  BOOST_REQUIRE(ca2==a2);
  BOOST_REQUIRE(ca1==ca2);
}


BOOST_AUTO_TEST_CASE( test_diff_empty )
{
  empty_A a;
  empty_B b;
  comparable_proxy ca(a), cb(b);
  BOOST_REQUIRE(ca==a);
  BOOST_REQUIRE(cb==b);
  BOOST_REQUIRE(ca!=cb);
}



struct A
{
  typedef std::tuple<int> member_types;
  int i;
};

constexpr bool operator==(A const& t1, A const& t2) noexcept
{ return (t1.i==t2.i); }


struct B
{
  typedef std::tuple<int> member_types;
  int i;
};

constexpr bool operator==(B const& t1, B const& t2) noexcept
{ return (t1.i==t2.i); }


BOOST_AUTO_TEST_CASE( test_same_value )
{
  A a1 = { 10 }, a2 = { 10 };
  comparable_proxy ca1(a1), ca2(a2);
  BOOST_REQUIRE(ca1==a1);
  BOOST_REQUIRE(ca2==a2);
  BOOST_REQUIRE(ca1==ca2);
}


BOOST_AUTO_TEST_CASE( test_diff_value )
{
  A a1 = { 10 }, a2 = { 11 };
  comparable_proxy ca1(a1), ca2(a2);
  BOOST_REQUIRE(ca1==a1);
  BOOST_REQUIRE(ca2==a2);
  BOOST_REQUIRE(ca1!=ca2);
}


BOOST_AUTO_TEST_CASE( test_diff_type )
{
  A a = { 10 };
  B b = { 10 };
  comparable_proxy ca(a), cb(b);
  BOOST_REQUIRE(ca==a);
  BOOST_REQUIRE(cb==b);
  BOOST_REQUIRE(ca!=cb);
}



void fooA(void)
{ }


void fooB(void)
{ }


BOOST_AUTO_TEST_CASE( test_same_function )
{
  comparable_proxy ca1(&fooA), ca2(&fooA);
  BOOST_REQUIRE(ca1==&fooA);
  BOOST_REQUIRE(ca2==&fooA);
  BOOST_REQUIRE(ca1==ca2);
}


BOOST_AUTO_TEST_CASE( test_diff_function )
{
  comparable_proxy ca(&fooA), cb(&fooB);
  BOOST_REQUIRE(ca==&fooA);
  BOOST_REQUIRE(cb==&fooB);
  BOOST_REQUIRE(ca!=cb);
}



BOOST_AUTO_TEST_CASE( test_same_pair )
{
  comparable_proxy ca(std::make_pair(empty_A{}, B{10}));
  comparable_proxy cb(std::make_pair(empty_A{}, B{10}));
  comparable_proxy cc(std::make_pair(empty_A{}, B{1}));
  BOOST_REQUIRE(ca==cb);
  BOOST_REQUIRE(ca!=cc);
}

BOOST_AUTO_TEST_CASE( test_diff_pair )
{
  comparable_proxy ca(std::make_pair(empty_A{}, B{10}));
  comparable_proxy cb(std::make_pair(empty_A{}, B{1}));
  comparable_proxy cc(std::make_pair(empty_A{}, empty_B{}));

  BOOST_REQUIRE(ca!=cb);
  BOOST_REQUIRE(ca!=cc);
}
