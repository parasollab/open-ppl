/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_CONCURRENCY_HPP
#define STAPL_RUNTIME_CONCURRENCY_CONCURRENCY_HPP

#include "../config.hpp"
#include "../affinity.hpp"
#include "../utility/option.hpp"
#include "mutex.hpp"
#include <functional>
#include <thread>
#include <vector>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Initializes and controls threads for shared-memory concurrency.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
class concurrency
{
public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Initializes the concurrency layer.
  ///
  /// The only option currently supported for the @ref option object is
  /// @c STAPL_PROC_HIERARCHY that is a comma-separated value that describes
  /// the shared memory node hierarchy (e.g STAPL_PROC_HIERARCHY=2,4,2 means 2
  /// sockets, with 4 cores each and 2 threads each).
  ///
  /// @param opts  Options to pass for initialization.
  /// @param pid   Current process id.
  /// @param npids Total number of processes.
  /// @param nppn  Number of processes per node.
  ////////////////////////////////////////////////////////////////////
  static void initialize(option const& opts,
                         const process_id pid,
                         const unsigned npids,
                         const unsigned int nppn);

  ////////////////////////////////////////////////////////////////////
  /// @brief Finalizes the concurrency layer.
  ////////////////////////////////////////////////////////////////////
  static void finalize(void);

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of threads available to the runtime.
  ///
  /// @warning This value is just a hint; there might be more or less threads in
  ///          absolute value.
  ////////////////////////////////////////////////////////////////////
  static unsigned int hardware_concurrency(void) noexcept;

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the id of the master thread of this process.
  ////////////////////////////////////////////////////////////////////
  static std::thread::id get_master_thread_id(void) noexcept;

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the parallelism level widths.
  ////////////////////////////////////////////////////////////////////
  static std::vector<unsigned int> get_level_widths(void);

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the available parallelism levels.
  ///
  /// After each call of @ref fork, the number of available levels decreases by
  /// @p n, where @p n is the last argument in @ref fork.
  ////////////////////////////////////////////////////////////////////
  static unsigned int available_levels(void) noexcept;

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the affinity of the calling thread.
  ////////////////////////////////////////////////////////////////////
  static affinity_tag get_affinity(void) noexcept;

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the @ref hierarchical_mutex tied to the calling thread.
  ////////////////////////////////////////////////////////////////////
  static hierarchical_mutex& get_mutex(void) noexcept;

  ////////////////////////////////////////////////////////////////////
  /// @brief Consumes the next @p n available parallelism levels and executes
  ///        the given function on them.
  ///
  /// @p n is either @p n or @ref available_levels(), whichever is smaller. The
  /// @p init function is run from the main thread and is passed the size of
  /// the new parallel section while @p f is being passed each thread's id.
  ///
  /// @param init Function to invoke only on the main thread.
  /// @param f    Function to invoke on all threads in the new parallel section.
  /// @param n    Number of parallelism levels to run @p f on.
  ////////////////////////////////////////////////////////////////////
  static void fork(std::function<void(unsigned int)> init,
                   std::function<void(unsigned int)> f,
                   const unsigned int n = 1);
};

} // namespace runtime

} // namespace stapl

#endif
