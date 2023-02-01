/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COMMUNICATOR_BARRIER_HPP
#define STAPL_RUNTIME_COMMUNICATOR_BARRIER_HPP

#include "reduce.hpp"
#include <functional>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Performs a barrier over the processes specified by the @ref topology
///        object.
///
/// @see collective
/// @ingroup runtimeCollectives
///
/// @todo Use platform optimized barrier implementation.
//////////////////////////////////////////////////////////////////////
class barrier
{
private:
  reduce<bool, std::logical_and<bool>> m_red;

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Constructs a @ref barrier object.
  ///
  /// @param gid Id of the gang the collective executes in.
  /// @param cid Collective operation id.
  /// @param t   @ref topology object associated with the gang.
  /// @param f   Function to call when all processes have reached the barrier.
  ////////////////////////////////////////////////////////////////////
  template<typename Function>
  barrier(const gang_id gid,
          const collective_id cid,
          topology const& t,
          Function&& f)
  : m_red(gid, cid, t, [f](bool) { f(); })
  { }

  void operator()(void)
  { m_red(true); }
};

} // namespace runtime

} // namespace stapl

#endif
