/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONFIG_PLATFORM_HPP
#define STAPL_RUNTIME_CONFIG_PLATFORM_HPP

//////////////////////////////////////////////////////////////////////
/// @file
/// Detects the platform.
///
/// Defined macros for supported platforms are:
/// -# @c STAPL_RUNTIME_LINUX_TARGET for GNU/Linux
/// -# @c STAPL_RUNTIME_WINDOWS_TARGET for Windows
/// -# @c STAPL_RUNTIME_SOLARIS_TARGET for Oracle Solaris
/// -# @c STAPL_RUNTIME_ANDROID_TARGET for Android
/// -# @c STAPL_RUNTIME_CRAY_TARGET for Cray
/// -# @c STAPL_RUNTIME_CRAY_XE_TARGET for Cray XE/XK
/// -# @c STAPL_RUNTIME_CRAY_XC30_TARGET for Cray XC30
/// -# @c STAPL_RUNTIME_BG_TARGET for IBM Blue Gene
/// -# @c STAPL_RUNTIME_BGP_TARGET for IBM Blue Gene/P
/// -# @c STAPL_RUNTIME_BGQ_TARGET for IBM Blue Gene/Q
///
/// @c STAPL_RUNTIME_DEFAULT_ALIGNMENT has the default alignment value for the
/// current platform.
///
/// @c STAPL_RUNTIME_CACHELINE_SIZE has the value for the cache-line size.
//////////////////////////////////////////////////////////////////////

#include <boost/config/select_platform_config.hpp>

// IBM BlueGene platform
#ifdef __bg__
# define STAPL_RUNTIME_BG_TARGET
# ifdef __bgp__
// IBM BlueGene P
#  define STAPL_RUNTIME_BGP_TARGET
#  define STAPL_RUNTIME_MSG_ALIGNMENT  32
#  define STAPL_RUNTIME_CACHELINE_SIZE 32
# elif __bgq__
// IBM BlueGene Q
#  define STAPL_RUNTIME_BGQ_TARGET
#  define STAPL_RUNTIME_MSG_ALIGNMENT  32
#  define STAPL_RUNTIME_CACHELINE_SIZE 32
# else
#  error "Unrecognized IBM BlueGene platform."
# endif

// Cray platform
#elif defined(__CRAYXE)
# define STAPL_RUNTIME_CRAY_TARGET
# ifdef __CRAYXT_COMPUTE_LINUX_TARGET
// Cray XE/XK
#  define STAPL_RUNTIME_CRAY_XE_TARGET
# else
// Cray XC30
#  define STAPL_RUNTIME_CRAY_XC30_TARGET
# endif

// Android platform
#elif defined(ANDROID) || defined(__ANDROID__)
# define STAPL_RUNTIME_ANDROID_TARGET

// Linux platform
#elif defined(__linux) || defined(__linux__) || defined(linux) || \
      defined(__gnu_linux__)
# define STAPL_RUNTIME_LINUX_TARGET

// Solaris platform
#elif defined(__sun) || defined(__sun__) || defined(sun)
# define STAPL_RUNTIME_SOLARIS_TARGET

// Windows platform
#elif defined(BOOST_WINDOWS)
# define STAPL_RUNTIME_WINDOWS_TARGET
#endif


#ifndef STAPL_RUNTIME_DEFAULT_ALIGNMENT
// Default alignment is sizeof(void*) bytes, i.e. machine word size.
// Setting the alignment to a wrong value can cause bus errors in specific
// machines (e.g. SPARC) or sub-optimal performance on others (e.g x86).
# define STAPL_RUNTIME_DEFAULT_ALIGNMENT (sizeof(void*))
#endif


#ifndef STAPL_RUNTIME_CACHELINE_SIZE
// Cache line size, used to pad shared variables to prevent false sharing
# define STAPL_RUNTIME_CACHELINE_SIZE 64
#endif

#endif
