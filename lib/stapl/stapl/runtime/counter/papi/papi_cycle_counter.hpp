/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COUNTER_PAPI_PAPI_CYCLE_COUNTER_HPP
#define STAPL_RUNTIME_COUNTER_PAPI_PAPI_CYCLE_COUNTER_HPP

#include "../../exception.hpp"
#include <papi.h>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Processor cycle counter that uses PAPI.
///
/// @ingroup counters
//////////////////////////////////////////////////////////////////////
class papi_cycle_counter
{
public:
  typedef long_long raw_value_type;
  typedef long_long value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the name of the counter as a C string.
  //////////////////////////////////////////////////////////////////////
  static constexpr const char* name(void) noexcept
  { return "PAPI_get_real_cyc()"; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the raw value from the counter.
  //////////////////////////////////////////////////////////////////////
  static value_type read(void) noexcept
  { return PAPI_get_real_cyc(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Normalizes the given raw value to cycles.
  //////////////////////////////////////////////////////////////////////
  static constexpr value_type normalize(const raw_value_type v) noexcept
  { return v; }

private:
  raw_value_type m_v;

public:
  constexpr papi_cycle_counter(void) noexcept
  : m_v(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Starts the counter.
  //////////////////////////////////////////////////////////////////////
  void start(void) noexcept
  { m_v = read(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Stops the counter and returns the difference from @ref start() in
  ///        cycles.
  //////////////////////////////////////////////////////////////////////
  value_type stop(void) const noexcept
  { return normalize(read() - m_v); }
};

} // namespace stapl

#endif
