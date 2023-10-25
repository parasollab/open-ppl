/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_INSTRUMENTATION_INSTRUMENT_HPP
#define STAPL_RUNTIME_INSTRUMENTATION_INSTRUMENT_HPP

#include "../utility/option.hpp"

namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Provides instrumentation capabilities for runtime functions.
///
/// This is the in-house instrumentation tool that allows accumulation of values
/// using a key.
///
/// The default output file has the name @c stapl_instrumentation.pid where
/// @p pid is the process id. The name can be overridden with the environment
/// variable @c STAPL_INSTRUMENTATION_FILE.
///
/// @ingroup instrumentationImpl
////////////////////////////////////////////////////////////////////
class instrument
{
public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Initializes instrumentation.
  ////////////////////////////////////////////////////////////////////
  static void initialize(option const&);

  ////////////////////////////////////////////////////////////////////
  /// @brief Finalizes instrumentation and outputs everything to the log file.
  ////////////////////////////////////////////////////////////////////
  static void finalize(void);

  ////////////////////////////////////////////////////////////////////
  /// @brief Increases the counter for the given key.
  ////////////////////////////////////////////////////////////////////
  static void accumulate(const char*);

  ////////////////////////////////////////////////////////////////////
  /// @brief Pushes back the value for the given key.
  ////////////////////////////////////////////////////////////////////
  static void push_back(const char*, int);

  ////////////////////////////////////////////////////////////////////
  /// @brief Clears all accumulated data.
  ////////////////////////////////////////////////////////////////////
  static void clear(void);
};

} // namespace runtime

} // namespace stapl


////////////////////////////////////////////////////////////////////
/// @brief Calls the @ref stapl::runtime::instrument::accumulate() with the
///        given arguments.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_CALL_INSTRUMENT(s) \
 stapl::runtime::instrument::accumulate((s));


////////////////////////////////////////////////////////////////////
/// @brief Calls the @ref stapl::runtime::instrument::push_back() with the given
///        arguments.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_STATISTICS(s, n) \
 stapl::runtime::instrument::push_back((s),(n))

#endif
