/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COUNTER_POSIX_GETTIMEOFDAY_TIMER_HPP
#define STAPL_RUNTIME_COUNTER_POSIX_GETTIMEOFDAY_TIMER_HPP

#include "../../exception.hpp"
#include <sys/time.h>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Time counter that uses @c gettimeofday() in POSIX systems.
////
/// @ingroup counters
/////////////////////////////////////////////////////////////////////
class gettimeofday_timer
{
public:
  typedef struct timeval raw_value_type;
  typedef double         value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the name of the counter as a C string.
  //////////////////////////////////////////////////////////////////////
  static constexpr const char* name(void) noexcept
  { return "gettimeofday()"; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the raw value from the counter.
  //////////////////////////////////////////////////////////////////////
  static raw_value_type read(void) noexcept
  {
    struct timeval  tv;
    struct timezone zone;
    STAPL_RUNTIME_CHECK((gettimeofday(&tv, &zone)==0), "gettimeofday() failed");
    return tv;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Normalizes the given raw value to seconds.
  //////////////////////////////////////////////////////////////////////
  static constexpr value_type normalize(raw_value_type const& v) noexcept
  { return (double(v.tv_sec) + double(v.tv_usec)*1.0E-6); }

private:
  raw_value_type m_v;

public:
  constexpr gettimeofday_timer(void) noexcept
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
  {
    const raw_value_type v = read();
    const raw_value_type diff = { (v.tv_sec - m_v.tv_sec),
                                  (v.tv_usec - m_v.tv_usec) };
    return normalize(diff);
  }
};

} // namespace stapl

#endif
