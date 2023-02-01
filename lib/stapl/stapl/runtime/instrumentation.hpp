/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_INSTRUMENTATION_HPP
#define STAPL_RUNTIME_INSTRUMENTATION_HPP

#include "config.hpp"
#include "primitive_traits.hpp"


#ifdef STAPL_RUNTIME_ENABLE_CALLBACKS
// callbacks support is requested
# include "instrumentation/callback.hpp"
#else
////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Dummy version when @ref stapl::callback support is not enabled.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
# define STAPL_RUNTIME_CALL_CALLBACKS(x)
#endif


#ifdef STAPL_RUNTIME_ENABLE_INSTRUMENTATION
// instrument support is requested
# include "instrumentation/instrument.hpp"
#else
////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Dummy version when @ref stapl::runtime::instrument support is not
///        enabled.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
# define STAPL_RUNTIME_CALL_INSTRUMENT(x)
////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Dummy version when @ref stapl::runtime::instrument support is not
///        enabled.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
# define STAPL_RUNTIME_STATISTICS(c, n) static_cast<void>(0)
#endif


#ifdef STAPL_RUNTIME_USE_MPE
// MPE integration requested
# include "instrumentation/mpe.hpp"
#else
# define STAPL_RUNTIME_CALL_MPE(x,y)
#endif


#ifdef STAPL_RUNTIME_ENABLE_NO_COMM_GUARD
// no_comm_guard support is requested
# include "instrumentation/no_comm_guard.hpp"
#else
////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Dummy version when @ref stapl::no_comm_guard support is not enabled.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
# define STAPL_RUNTIME_CALL_NO_COMM_GUARD(x)
#endif


#ifdef STAPL_RUNTIME_USE_VAMPIR
// vampir integration requested
# include "instrumentation/vampir.hpp"
#else
////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Dummy version when TAU support is not enabled.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
# define STAPL_RUNTIME_CALL_VAMPIR(x)
#endif


#ifdef STAPL_RUNTIME_USE_TAU
// TAU integration requested
# include "instrumentation/tau.hpp"
#else
////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Dummy version when TAU support is not enabled.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
# define STAPL_RUNTIME_CALL_TAU(x)
#endif


////////////////////////////////////////////////////////////////////
/// @brief Calls all requested instrumentation tools.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_PROFILE(s, traits)   \
  STAPL_RUNTIME_CALL_INSTRUMENT(s)         \
  STAPL_RUNTIME_CALL_CALLBACKS(s)          \
  STAPL_RUNTIME_CALL_NO_COMM_GUARD(traits) \
  STAPL_RUNTIME_CALL_MPE(s, traits)        \
  STAPL_RUNTIME_CALL_VAMPIR(s)             \
  STAPL_RUNTIME_CALL_TAU(s)                \
  static_cast<void>(0)

#endif
