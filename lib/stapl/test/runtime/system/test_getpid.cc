/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE getpid
#include "utility.h"
#include <stapl/runtime/system.hpp>

BOOST_AUTO_TEST_CASE( test_gettpid )
{
  BOOST_CHECK( stapl::runtime::getpid()!=0 );
}

