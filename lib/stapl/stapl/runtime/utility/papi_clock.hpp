/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_PAPI_CLOCK_HPP
#define STAPL_RUNTIME_UTILITY_PAPI_CLOCK_HPP

#include <chrono>
#include <papi.h>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Represents a clock based on @c PAPI_get_real_usec().
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
class papi_clock
{
public:
  using duration   = std::chrono::microseconds;
  using rep        = duration::rep;
  using period     = duration::period;
  using time_point = std::chrono::time_point<papi_clock, duration>;

  static constexpr bool is_steady = true;

  static time_point now(void) noexcept
  {
    return time_point{duration(PAPI_get_real_usec())};
  }
};

} // namespace runtime

} // namespace stapl

#endif
