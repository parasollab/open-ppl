/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_HEAP_TRACKER_HPP
#define STAPL_RUNTIME_HEAP_TRACKER_HPP

#include <new>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Nifty counter idiom to initialize the heap tracker and keep it alive
///        until the last object file that uses it is finalized.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
static struct heap_tracker_init
{
  heap_tracker_init(void) noexcept;
  ~heap_tracker_init(void);
} hpi;

} // namespace runtime

} // namespace stapl

#endif
