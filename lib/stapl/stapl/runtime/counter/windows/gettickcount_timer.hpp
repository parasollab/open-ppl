/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COUNTER_WINDOWS_GETTICKCOUNT_TIMER_HPP
#define STAPL_RUNTIME_COUNTER_WINDOWS_GETTICKCOUNT_TIMER_HPP

#include <Windows.h>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Time counter that uses @c GetTickCount() in Windows systems.
///
/// @ingroup counters
//////////////////////////////////////////////////////////////////////
class gettickcount_timer
{
public:
  typedef DWORD  raw_value_type;
  typedef double value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the name of the counter as a C string.
  //////////////////////////////////////////////////////////////////////
  static constexpr const char* name(void) noexcept
  { return "GetTickCount()"; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the raw value from the counter.
  //////////////////////////////////////////////////////////////////////
  static raw_value_type read(void) noexcept
  { return GetTickCount(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Normalizes the given raw value to seconds.
  //////////////////////////////////////////////////////////////////////
  static constexpr value_type normalize(const raw_value_type v) noexcept
  { return (v * 1.0E-6); }

private:
  raw_value_type m_v;

public:
  constexpr gettickcount_timer(void) noexcept
  : m_v()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Starts the counter.
  //////////////////////////////////////////////////////////////////////
  void start(void) noexcept
  { m_v = read(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Stops the counter and returns the difference from @ref start() in
  ///        seconds.
  //////////////////////////////////////////////////////////////////////
  value_type stop(void) const noexcept
  { return normalize(read() - m_v); }
};

} // namespace stapl

#endif
