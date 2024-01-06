/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COUNTER_GENERIC_PROCESS_MEMORY_COUNTER_HPP
#define STAPL_RUNTIME_COUNTER_GENERIC_PROCESS_MEMORY_COUNTER_HPP

#include "../../system.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Memory usage counter that uses @ref get_used_physical_memory().
///
/// This counter reports the memory usage per process.
///
/// @ingroup counters
//////////////////////////////////////////////////////////////////////
class process_memory_usage_counter
{
public:
  typedef std::size_t raw_value_type;
  typedef int         value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the name of the counter as a C string.
  //////////////////////////////////////////////////////////////////////
  static constexpr const char* name(void) noexcept
  { return "stapl::get_used_physical_memory()"; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the raw value from the counter.
  //////////////////////////////////////////////////////////////////////
  static raw_value_type read(void) noexcept
  { return runtime::get_used_physical_memory(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Normalizes the given raw value to seconds.
  //////////////////////////////////////////////////////////////////////
  static constexpr value_type normalize(const raw_value_type v) noexcept
  { return v; }

private:
  raw_value_type m_v;

public:
  constexpr process_memory_usage_counter(void) noexcept
  : m_v()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Starts the counter.
  //////////////////////////////////////////////////////////////////////
  void start(void) noexcept
  { m_v = read(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Stops the counter and returns the difference from @ref start() in
  ///        bytes.
  //////////////////////////////////////////////////////////////////////
  value_type stop(void) const noexcept
  {
    const raw_value_type v = read();
    return ( (v>m_v) ? normalize(v - m_v) : -normalize(m_v - v) );

  }
};

} // namespace stapl

#endif
