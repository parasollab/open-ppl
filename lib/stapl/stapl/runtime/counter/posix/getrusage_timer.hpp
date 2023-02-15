/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COUNTER_POSIX_GETRUSAGE_TIMER_HPP
#define STAPL_RUNTIME_COUNTER_POSIX_GETRUSAGE_TIMER_HPP

#include "../../exception.hpp"
#include <sys/time.h>
#include <sys/resource.h>

namespace stapl {


inline struct timeval&
operator+=(struct timeval& x, struct timeval const& y) noexcept
{
  x.tv_sec  += y.tv_sec;
  x.tv_usec += y.tv_usec;
  if (x.tv_usec>=1000000000) {
    x.tv_sec  += 1;
    x.tv_usec -= 1000000000;
  }
  return x;
}


inline struct timeval&
operator-=(struct timeval& x, struct timeval const& y) noexcept
{
  x.tv_sec  -= y.tv_sec;
  x.tv_usec -= y.tv_usec;
  if (x.tv_usec<0) {
    x.tv_sec  -= 1;
    x.tv_usec += 1000000000;
  }
  return x;
}


inline struct rusage&
operator+=(struct rusage& x, struct rusage const& y) noexcept
{
  x.ru_utime    += y.ru_utime;    // user time used
  x.ru_stime    += y.ru_stime;    // system time used
  x.ru_maxrss   += y.ru_maxrss;   // maximum resident set size
  x.ru_ixrss    += y.ru_ixrss;    // integral shared memory size
  x.ru_idrss    += y.ru_idrss;    // integral unshared data size
  x.ru_isrss    += y.ru_isrss;    // integral unshared stack size
  x.ru_minflt   += y.ru_minflt;   // page reclaims
  x.ru_majflt   += y.ru_majflt;   // page faults
  x.ru_nswap    += y.ru_nswap;    // swaps
  x.ru_inblock  += y.ru_inblock;  // block input operations
  x.ru_oublock  += y.ru_oublock;  // block output operations
  x.ru_msgsnd   += y.ru_msgsnd;   // messages sent
  x.ru_msgrcv   += y.ru_msgrcv;   // messages received
  x.ru_nsignals += y.ru_nsignals; // signals received
  x.ru_nvcsw    += y.ru_nvcsw;    // voluntary context switches
  x.ru_nivcsw   += y.ru_nivcsw;   // involuntary context switches
  return x;
}


inline struct rusage&
operator-=(struct rusage& x, struct rusage const& y) noexcept
{
  x.ru_utime    -= y.ru_utime;    // user time used
  x.ru_stime    -= y.ru_stime;    // system time used
  x.ru_maxrss   -= y.ru_maxrss;   // maximum resident set size
  x.ru_ixrss    -= y.ru_ixrss;    // integral shared memory size
  x.ru_idrss    -= y.ru_idrss;    // integral unshared data size
  x.ru_isrss    -= y.ru_isrss;    // integral unshared stack size
  x.ru_minflt   -= y.ru_minflt;   // page reclaims
  x.ru_majflt   -= y.ru_majflt;   // page faults
  x.ru_nswap    -= y.ru_nswap;    // swaps
  x.ru_inblock  -= y.ru_inblock;  // block input operations
  x.ru_oublock  -= y.ru_oublock;  // block output operations
  x.ru_msgsnd   -= y.ru_msgsnd;   // messages sent
  x.ru_msgrcv   -= y.ru_msgrcv;   // messages received
  x.ru_nsignals -= y.ru_nsignals; // signals received
  x.ru_nvcsw    -= y.ru_nvcsw;    // voluntary context switches
  x.ru_nivcsw   -= y.ru_nivcsw;   // involuntary context switches
  return x;
}


//////////////////////////////////////////////////////////////////////
/// @brief Counter that uses @c getrusage() in POSIX systems.
///
/// @ingroup counters
///
/// @todo The return type needs more refinement.
//////////////////////////////////////////////////////////////////////
class getrusage_timer
{
public:
  typedef struct rusage raw_value_type;
  typedef struct rusage value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the name of the counter as a C string.
  //////////////////////////////////////////////////////////////////////
  static constexpr const char* name(void) noexcept
  { return "getrusage()"; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the raw value from the counter.
  //////////////////////////////////////////////////////////////////////
  static value_type read(void) noexcept
  {
    struct rusage nt;
    STAPL_RUNTIME_CHECK((getrusage(RUSAGE_SELF, &nt)==0), "getrusage() failed");
    return nt;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Normalizes the given raw value.
  //////////////////////////////////////////////////////////////////////
  static constexpr value_type const& normalize(raw_value_type const& v) noexcept
  { return v; }

private:
  raw_value_type m_v;

public:
  constexpr getrusage_timer(void) noexcept
  : m_v()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Starts the counter.
  //////////////////////////////////////////////////////////////////////
  void start(void) noexcept
  { m_v = read(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Stops the counter and returns the difference from @ref start() as
  ///        an @c rusage object.
  //////////////////////////////////////////////////////////////////////
  value_type stop(void) const noexcept
  {
    raw_value_type v = read();
    v -= m_v;
    return normalize(v);
  }
};

} // namespace stapl

#endif
