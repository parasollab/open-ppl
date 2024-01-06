/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE demangle
#include "utility.h"
#include <stapl/runtime/system.hpp>
#include <string>
#include <typeinfo>

namespace A { namespace B { namespace C {

class Class { };

} } }

BOOST_AUTO_TEST_CASE( test_demangle )
{
  std::string unmangled(stapl::runtime::demangle(typeid(A::B::C::Class).name()).c_str());
  BOOST_CHECK_EQUAL( std::string("A::B::C::Class"), unmangled );
}

