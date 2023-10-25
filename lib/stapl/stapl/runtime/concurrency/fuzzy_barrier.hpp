/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_FUZZY_BARRIER_HPP
#define STAPL_RUNTIME_CONCURRENCY_FUZZY_BARRIER_HPP

#include "config.hpp"

#if defined(STAPL_RUNTIME_SLOW_BARRIER)
// Slow barrier
# include "generic/centralized_fuzzy_barrier.hpp"
#else
// Everything else
# include "generic/fuzzy_barrier.hpp"
#endif

#endif
