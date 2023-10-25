/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Test for correct alignment in STAPL-RTS internal structures.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <stapl/runtime/request/header.hpp>
#include <algorithm>
#include <cstddef>
#include <iostream>
#include "test_utils.h"

using namespace stapl::runtime;

const std::size_t alignment = STAPL_RUNTIME_DEFAULT_ALIGNMENT;


void check_alignment(message_ptr m)
{
  STAPL_RUNTIME_TEST_CHECK(m->header_size() % alignment, 0);
  STAPL_RUNTIME_TEST_CHECK(std::uintptr_t(m->data()) % alignment, 0);
  STAPL_RUNTIME_TEST_CHECK(std::uintptr_t(&(m->header())) % alignment, 0);
  auto p = m->payload();
  STAPL_RUNTIME_TEST_CHECK(std::uintptr_t(&(*p.begin())) % alignment, 0);
}


struct unaligned_header
{
  char c;
};

stapl::exit_code stapl_main(int, char*[])
{
  check_alignment(message_ptr{message::construct()});
  check_alignment(message::create(3));
  check_alignment(message::create(header::INVALID, 3));
  check_alignment(message::create(header::INVALID, 0, unaligned_header{}));


#ifndef _TEST_QUIET
  std::cout << stapl::get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}
