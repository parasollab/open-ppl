/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONFIG_SUFFIX_HPP
#define STAPL_RUNTIME_CONFIG_SUFFIX_HPP

//////////////////////////////////////////////////////////////////////
/// @file
/// Defines higher-level macros and all macros that were not detected.
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
/// @def STAPL_RUNTIME_CACHELINE_ALIGNED
/// Cache-line alignment directive.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_CACHELINE_ALIGNED \
  alignas(STAPL_RUNTIME_CACHELINE_SIZE)

//////////////////////////////////////////////////////////////////////
/// @def STAPL_RUNTIME_SPMD_REGISTRY_SIZE
/// Default @ref stapl::runtime::spmd_registry size.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////

#ifndef STAPL_RUNTIME_SPMD_REGISTRY_SIZE
// Default size of spmd_registry
# define STAPL_RUNTIME_SPMD_REGISTRY_SIZE 20
#endif

#endif
