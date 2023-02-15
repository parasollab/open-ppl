/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TEST_CONCURRENCY_UTILITY_H
#define STAPL_RUNTIME_TEST_CONCURRENCY_UTILITY_H

#define BOOST_TEST_MODULE STAPL_RUNTIME_TEST_MODULE

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-variable"
#endif

#ifdef BOOST_TEST_USE_INCLUDED
# include <boost/test/included/unit_test.hpp>
#else
# define BOOST_TEST_DYN_LINK
# include <boost/test/unit_test.hpp>
#endif

#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <cstdlib>
#include <iostream>

namespace stapl {

namespace runtime {

void assert_fail(const char* assertion,
                 const char* file, unsigned int line, const char* function)
{
  std::cerr << assertion
            << " (file: "     << file
            << ", function: " << function
            << ", line: "     << line << ")\n";
  std::abort();
}

void assert_fail(const char* assertion, const char* function)
{
  std::cerr << assertion << " (function: " << function << ")\n";
  std::abort();
}

} // namespace runtime

} // namespace stapl

#include <stapl/runtime/concurrency/config.hpp>
#if defined(STAPL_RUNTIME_USE_OMP)
# include <omp.h>
#else
# include <thread>
#endif

#endif
