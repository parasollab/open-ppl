/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_THREAD_LOCAL_STORAGE_HPP
#define STAPL_RUNTIME_CONCURRENCY_THREAD_LOCAL_STORAGE_HPP

#include "config.hpp"

#if defined(STAPL_RUNTIME_ANDROID_TARGET) || \
    defined(__IBMCPP__)                   || \
    defined(__SUNPRO_CC)
// Android linker and compilers that do not support thread_local
# include "boost/thread_local_storage.hpp"
#else
// thread_local supported
# include "generic/thread_local_storage.hpp"
#endif


//////////////////////////////////////////////////////////////////////
/// @def STAPL_RUNTIME_THREAD_LOCAL_SPEC
/// @brief Thread-local specifier.
///
/// @ingroup runtimeConfig
//////////////////////////////////////////////////////////////////////
#if (defined(__clang__) && __clang_major__==3 && __clang_minor__<6)
# define STAPL_RUNTIME_THREAD_LOCAL_SPEC __thread
#else
# define STAPL_RUNTIME_THREAD_LOCAL_SPEC thread_local
#endif

#endif
