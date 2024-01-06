/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONFIG_USER_HPP
#define STAPL_RUNTIME_CONFIG_USER_HPP

// Disables request combining
// #define STAPL_RUNTIME_DISABLE_COMBINING

// Disables Boost.Serialization support
// #define STAPL_DONT_USE_BOOST_SERIALIZATION

// Enables callbacks
// #define STAPL_RUNTIME_ENABLE_CALLBACKS

// Enables the use of the MPE logging facilities
// #define STAPL_RUNTIME_USE_MPE

// Enables the use of vampirtrace tracing
// #define STAPL_RUNTIME_USE_VAMPIR

// Enables the use of TAU
// #define STAPL_RUNTIME_USE_TAU

// Enables STAPL RTS instrumentation
// #define STAPL_RUNTIME_ENABLE_INSTRUMENTATION

// Enables STAPL Heap Tracking and Profiling
// #define STAPL_ENABLE_HEAP_TRACKING

// Enables managed heap allocation statistics
// #define STAPL_ENABLE_MANAGED_ALLOC_STATISTICS

#endif
