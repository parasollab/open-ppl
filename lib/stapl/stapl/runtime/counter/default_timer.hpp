/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COUNTER_DEFAULT_TIMER_HPP
#define STAPL_RUNTIME_COUNTER_DEFAULT_TIMER_HPP

#include "config.hpp"

#if STAPL_USE_TIMER==STAPL_PAPI_TIMER
# include "papi/papi_timer.hpp"

namespace stapl {

typedef papi_timer default_timer;

} // namespace stapl

#elif STAPL_USE_TIMER==STAPL_MPI_WTIME_TIMER
# include "mpi/mpi_wtime_timer.hpp"

namespace stapl {

typedef mpi_wtime_timer default_timer;

} // namespace stapl

#elif STAPL_USE_TIMER==STAPL_CLOCK_GETTIME_TIMER
# include "posix/clock_gettime_timer.hpp"

namespace stapl {

typedef clock_gettime_timer default_timer;

} // namespace stapl

#elif STAPL_USE_TIMER==STAPL_GETTIMEOFDAY_TIMER
# include "posix/gettimeofday_timer.hpp"

namespace stapl {

typedef gettimeofday_timer default_timer;

} // namespace stapl

#elif STAPL_USE_TIMER==STAPL_GETRUSAGE_TIMER
# include "posix/getrusage_timer.hpp"

namespace stapl {

typedef getrusage_timer default_timer;

} // namespace stapl

#elif STAPL_USE_TIMER==STAPL_GETTICKCOUNT_TIMER
# include "windows/gettickcount_timer.hpp"

namespace stapl {

typedef gettickcount_timer default_timer;

} // namespace stapl

#endif


#endif
