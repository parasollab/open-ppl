/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_INSTRUMENTATION_TAU_HPP
#define STAPL_RUNTIME_INSTRUMENTATION_TAU_HPP

#include <TAU.h>

namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Returns a TAU group for the runtime.
///
/// This function is used to get the same TAU group to annotate runtime
/// functions so that proper integration with TAU is in place.
///
/// @ingroup instrumentationImpl
////////////////////////////////////////////////////////////////////
inline TauGroup_t const& get_tau_group(void)
{
  static TauGroup_t grp;
  return grp;
}

} // namespace runtime

} // namespace stapl


////////////////////////////////////////////////////////////////////
/// @brief Calls the TAU profiler with the given arguments.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_CALL_TAU(s) \
 TAU_PROFILE((s), " ", stapl::runtime::get_tau_group());

#endif
