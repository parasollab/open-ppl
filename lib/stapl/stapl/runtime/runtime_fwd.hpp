/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RUNTIME_FWD_HPP
#define STAPL_RUNTIME_RUNTIME_FWD_HPP

#include "constants.hpp"
#include "executor/runnable_base.hpp"

namespace stapl {

process_id get_process_id(void) noexcept;

process_id get_num_processes(void) noexcept;

unsigned int get_location_id(void) noexcept;

unsigned int get_num_locations(void) noexcept;

} // namespace stapl

#endif
