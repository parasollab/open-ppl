/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_CONFIG_HPP
#define STAPL_RUNTIME_CONCURRENCY_CONFIG_HPP

#include "../config/find_user_config.hpp"
#include "../config/platform.hpp"
#include "../config/compiler.hpp"
#include "../config/suffix.hpp"

#if defined(STAPL_RUNTIME_USE_THREAD)
// native threads backend is used
# if defined(STAPL_RUNTIME_THREADING_BACKEND_DEFINED)
#  error "Multiple multi-threading backends defined."
# endif
# define STAPL_RUNTIME_THREADING_BACKEND_DEFINED 1
#endif

#if defined(STAPL_RUNTIME_USE_TBB)
// Intel Threading Building Blocks backend is used
# if defined(STAPL_RUNTIME_THREADING_BACKEND_DEFINED)
#  error "Multiple multi-threading backends defined."
# endif
# include <tbb/tbb_stddef.h>
# if TBB_INTERFACE_VERSION<4002
#  error "Intel TBB version is too old. Please use TBB 3.0 or newer."
# endif
# define STAPL_RUNTIME_THREADING_BACKEND_DEFINED 1
#endif

#if defined(STAPL_RUNTIME_USE_OMP)
// OpenMP backend is used
# if defined(STAPL_RUNTIME_THREADING_BACKEND_DEFINED)
#  error "Multiple multi-threading backends defined."
# endif
# if _OPENMP<200805
#  error "OpenMP version is too old. Please use OpenMP 3.0 or newer."
# endif
# define STAPL_RUNTIME_THREADING_BACKEND_DEFINED 1
#endif


#if !defined(STAPL_RUNTIME_THREADING_BACKEND_DEFINED)
// multithreaded backend not defined, default to C++11 threading support
# define STAPL_RUNTIME_USE_THREAD
# define STAPL_RUNTIME_THREADING_BACKEND_DEFINED 1
#endif

#if !defined(STAPL_RUNTIME_TBB_AVAILABLE) && defined(BOOST_INTEL)
// Intel Threading Building Blocks is available, use its containers
# define STAPL_RUNTIME_TBB_AVAILABLE
#endif

#endif
