/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COUNTER_POSIX_CLOCK_GETTIME_TIMER_HPP
#define STAPL_RUNTIME_COUNTER_POSIX_CLOCK_GETTIME_TIMER_HPP

#include "../../exception.hpp"
#if defined(__APPLE)
# include <sys/time.h>
#else
# include <time.h>
#endif

#ifndef STAPL_USE_CLOCK_GETTIME_ID
# if defined(CLOCK_MONOTONIC_RAW)
// best for Linux (since Linux 2.6.28)
#  define STAPL_USE_CLOCK_GETTIME_ID CLOCK_MONOTONIC_RAW
# elif defined(_POSIX_MONOTONIC_CLOCK) || defined(CLOCK_MONOTONIC)
// second best for Linux and best for Solaris
#  define STAPL_USE_CLOCK_GETTIME_ID CLOCK_MONOTONIC
# elif defined(CLOCK_SGI_CYCLE)
// SGI
#  define STAPL_USE_CLOCK_GETTIME_ID CLOCK_SGI_CYCLE
# else
#  error "Automatic clock id discovery for clock_gettime() failed. " \
         "Please provide one through the STAPL_USE_CLOCK_GETTIME_ID flag."
# endif
#endif

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Time counter that uses @c clock_gettime() in POSIX systems.
///
/// It will try to identify the best possible clock id at compile time and can
/// be overridden only by setting the flag
/// @c STAPL_USE_CLOCK_GETTIME_ID=system-provided-value.
///
/// In GNU/Linux you have to link against @c -lrt.
///
/// @ingroup counters
//////////////////////////////////////////////////////////////////////
class clock_gettime_timer
{
public:
  typedef struct timespec raw_value_type;
  typedef double          value_type;

  static const clockid_t clk_id = STAPL_USE_CLOCK_GETTIME_ID;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the name of the counter as a C string.
  //////////////////////////////////////////////////////////////////////
  static const char* name(void) noexcept
  {
    switch (clk_id) {
#if defined(CLOCK_MONOTONIC_RAW)
    case CLOCK_MONOTONIC_RAW:
      return "clock_gettime(CLOCK_MONOTONIC_RAW)";
#endif
#if defined(CLOCK_MONOTONIC)
    case CLOCK_MONOTONIC:
      return "clock_gettime(CLOCK_MONOTONIC)";
#endif
#if defined(CLOCK_SGI_CYCLE)
    case CLOCK_SGI_CYCLE:
      return "clock_gettime(CLOCK_SGI_CYCLE)";
#endif
#if defined(CLOCK_REALTIME)
    case CLOCK_REALTIME:
      return "clock_gettime(CLOCK_REALTIME)";
#endif
#if defined(CLOCK_PROCESS_CPUTIME_ID)
    case CLOCK_PROCESS_CPUTIME_ID:
      return "clock_gettime(CLOCK_PROCESS_CPUTIME_ID)";
#endif
#if defined(CLOCK_THREAD_CPUTIME_ID)
    case CLOCK_THREAD_CPUTIME_ID:
      return "clock_gettime(CLOCK_THREAD_CPUTIME_ID)";
#endif
    default:
      return "clock_gettime(unknown)";
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the raw value from the counter.
  //////////////////////////////////////////////////////////////////////
  static raw_value_type read(void) noexcept
  {
    struct timespec nt = timespec();
    STAPL_RUNTIME_CHECK((clock_gettime(clk_id, &nt)==0),
                        "clock_gettime() failed");
    return nt;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Normalizes the given raw value to seconds.
  //////////////////////////////////////////////////////////////////////
  static constexpr value_type normalize(raw_value_type const& v) noexcept
  { return (double(v.tv_sec) + double(v.tv_nsec)*1.0E-9); }

private:
  raw_value_type m_v;

public:
  constexpr clock_gettime_timer(void) noexcept
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
                                  (v.tv_nsec - m_v.tv_nsec) };
    return normalize(diff);
  }
};

} // namespace stapl

#endif
