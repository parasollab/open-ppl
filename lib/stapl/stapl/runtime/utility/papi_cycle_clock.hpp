/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_PAPI_CYCLE_CLOCK_HPP
#define STAPL_RUNTIME_UTILITY_PAPI_CYCLE_CLOCK_HPP

#include <chrono>
#include <papi.h>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Represents a clock based on @c PAPI_get_real_cyc() and @c MHz.
///
/// This clock is approximate but fast.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
class papi_cycle_clock
{
public:
  using duration   = std::chrono::microseconds;
  using rep        = duration::rep;
  using period     = duration::period;
  using time_point = std::chrono::time_point<papi_cycle_clock, duration>;

  static constexpr bool is_steady = false;

private:
  static unsigned long get_processor_mhz(void) noexcept
  {
    const PAPI_hw_info_t* const hwinfo = PAPI_get_hardware_info();
    return hwinfo->cpu_max_mhz;
  }

public:
  static time_point now(void) noexcept
  {
    static const auto mhz = get_processor_mhz();
    return time_point{duration(PAPI_get_real_cyc() / mhz)};
  }
};

} // namespace runtime

} // namespace stapl

#endif
