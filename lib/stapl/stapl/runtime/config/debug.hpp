/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

//////////////////////////////////////////////////////////////////////
/// @file
/// Detects if the runtime should be compiled in debug mode, in which case
/// @c STAPL_RUNTIME_DEBUG is defined to the value of @c 1.
//////////////////////////////////////////////////////////////////////

#ifndef STAPL_RUNTIME_CONFIG_DEBUG_HPP
#define STAPL_RUNTIME_CONFIG_DEBUG_HPP

#if !defined(STAPL_RUNTIME_DEBUG)  && \
    !defined(STAPL_RUNTIME_NDEBUG) && \
    !defined(STAPL_NDEBUG)         && \
    !defined(NDEBUG)
// Enable runtime debugging
# define STAPL_RUNTIME_DEBUG 1
#endif

#endif
