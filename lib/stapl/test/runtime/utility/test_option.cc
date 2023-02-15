/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE option
#include "utility.h"
#include <stapl/runtime/utility/option.hpp>
#include <cstdlib>

using stapl::option;


void fake_main(int argc, char* argv[])
{
  option opts;
  BOOST_CHECK_EQUAL( opts.has_argv(), false );

  opts = opts & option(argc, argv);

  BOOST_CHECK_EQUAL( opts.has_argv(), true );
  BOOST_REQUIRE_EQUAL( opts.get_argc(), 2 );
  BOOST_CHECK_EQUAL( opts.get_argv()[0], argv[0] );
  BOOST_CHECK_EQUAL( opts.get_argv()[1], argv[1] );
}


BOOST_AUTO_TEST_CASE( test_option_main )
{
  int argc = 2;
  char a0[6] = "hello";
  char a1[6] = "world";
  char* argv[] = { a0, a1 };
  fake_main(argc, argv);
}


BOOST_AUTO_TEST_CASE( test_option_1 )
{
  option opts;

  int i = 0;
  bool r1 = opts.try_get("i", i);
  BOOST_CHECK( !r1 );
  BOOST_CHECK_EQUAL( i, 0);

  double d = 0.0;
  bool r2 = opts.try_get("d", d);
  BOOST_CHECK( !r2 );
  BOOST_CHECK_EQUAL( d, 0.0);

  opts = opts & option("i", 42) & option("d", 43.5);

  bool r3 = opts.try_get("i", i);
  BOOST_CHECK( r3 );
  BOOST_CHECK_EQUAL( i, 42);

  bool r4 = opts.try_get("d", d);
  BOOST_CHECK( r4 );
  BOOST_CHECK_EQUAL( d, 43.5);
}


BOOST_AUTO_TEST_CASE( test_option_2 )
{
  option opts;

  int r1 = opts.get("i", 10);
  BOOST_CHECK_EQUAL( r1, 10);

  opts = opts & option("i", 42);
  int r2 = opts.get("i", 10);
  BOOST_CHECK_EQUAL( r2, 42);

  std::string r3 = opts.get("PATH", std::string("empty"));
  BOOST_CHECK( r3!=std::string("empty") );

  std::string r4 = opts.get("WHYONEARTHWOULDYOUDEFINETHAT",
                            std::string("awesome"));
  BOOST_CHECK( r4==std::string("awesome") );
}
