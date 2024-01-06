/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COUNTER_CONFIG_HPP
#define STAPL_RUNTIME_COUNTER_CONFIG_HPP

//////////////////////////////////////////////////////////////////////
/// @file
/// Chooses a suitable set of counters based on user preference and platform.
///
/// The user can override the platform-preferred counters by defining
/// @c STAPL_USE_TIMER at compile-time.
///
/// @ingroup counters
//////////////////////////////////////////////////////////////////////

#include "../config/platform.hpp"

#define STAPL_PAPI_TIMER           1
#define STAPL_MPI_WTIME_TIMER      2
#define STAPL_CLOCK_GETTIME_TIMER  3
#define STAPL_GETTIMEOFDAY_TIMER   4
#define STAPL_GETRUSAGE_TIMER      5
#define STAPL_GETTICKCOUNT_TIMER   6

#ifndef STAPL_USE_TIMER
// if no timer specified, attempt to autodetect one
# if defined(STAPL_USE_PAPI)
// PAPI exists, use that one
#  define STAPL_USE_TIMER STAPL_PAPI_TIMER
# elif defined(STAPL_RUNTIME_WINDOWS_TARGET)
// Microsoft Windows VisualC++ - use Windows High Performance Counters
#  define STAPL_USE_TIMER STAPL_GETTICKCOUNT_TIMER
# elif defined(STAPL_RUNTIME_BG_TARGET)
// BlueGene P/Q have gettimeofday() but not clock_gettime()
#  define STAPL_USE_TIMER STAPL_GETTIMEOFDAY_TIMER
# else
// assume a POSIX system, try to detect if clock_gettime() exists
#  include <unistd.h> // for _POSIX_TIMERS (POSIX.1-2001)
#  if _POSIX_TIMERS>0
#   define STAPL_USE_TIMER STAPL_CLOCK_GETTIME_TIMER
#  else
#   if defined(__APPLE) // for CLOCK_REALTIME
#    include <sys/time.h>
#   else
#    include <time.h>
#   endif
#   if defined(CLOCK_REALTIME)
#    define STAPL_USE_TIMER STAPL_CLOCK_GETTIME_TIMER
#   endif
#  endif // _POSIX_TIMERS
# endif // STAPL_USE_PAPI
#endif // STAPL_USE_TIMER

#ifndef STAPL_USE_TIMER
// if autodetection failed, fall back to gettimeofday()
# define STAPL_USE_TIMER STAPL_GETTIMEOFDAY_TIMER
#endif

#endif
