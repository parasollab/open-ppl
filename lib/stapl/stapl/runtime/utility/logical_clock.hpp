/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_LOGICAL_CLOCK_HPP
#define STAPL_RUNTIME_UTILITY_LOGICAL_CLOCK_HPP

#include "../exception.hpp"
#include <limits>
#include <tuple>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Implements a logical clock for managing epochs.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
class logical_clock
{
public:
  typedef unsigned int          time_type;
  typedef std::tuple<time_type> member_types;

  static const time_type no_time    = 0;
  static const time_type start_time = 1;

private:
  time_type m_time;

public:
  constexpr explicit logical_clock(const time_type t = start_time) noexcept
  : m_time(t)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Increases the time.
  //////////////////////////////////////////////////////////////////////
  time_type tick(void) noexcept
  {
    const time_type t = ++m_time;
    if (t == std::numeric_limits<time_type>::max())
      STAPL_RUNTIME_ERROR("Logical clock overflowed.");
    return t;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the current time.
  //////////////////////////////////////////////////////////////////////
  constexpr time_type time(void) const noexcept
  { return m_time; }

  void reset(const time_type t = start_time) noexcept
  { m_time = t; }

  friend constexpr bool operator==(logical_clock const& x,
                                   logical_clock const& y) noexcept
  {
    return (x.m_time==y.m_time);
  }

  friend constexpr bool operator!=(logical_clock const& x,
                                   logical_clock const& y) noexcept
  {
    return !(x==y);
  }
};

} // namespace runtime

} // namespace stapl

#endif
