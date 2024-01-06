/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_INSTRUMENTATION_VAMPIR_HPP
#define STAPL_RUNTIME_INSTRUMENTATION_VAMPIR_HPP

#include <vampir_user.h>

// if vampirtrace is supposed to be used in STAPL, check if it is enabled
// at compile-time by the user
#ifndef VTRACE
# warning "vampirtrace support enabled, but you are not compiling with -DVTRACE"
#endif


////////////////////////////////////////////////////////////////////
/// @brief Calls the vampir profiler with the given arguments.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_CALL_VAMPIR(s) \
 VT_TRACER((s));

#endif
