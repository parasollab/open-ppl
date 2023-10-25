/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_RUNNABLE_BASE_HPP
#define STAPL_RUNTIME_EXECUTOR_RUNNABLE_BASE_HPP

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Common base for all entries that can be scheduled and executed,
///        such as tasks and executors.
///
/// @ingroup executors
//////////////////////////////////////////////////////////////////////
class runnable_base
{
public:
  /// Return values for function operator.
  enum status_type
  {
    /// Runnable object is active.
    Active,
    /// Runnable object is idle.
    Idle,
    /// Runnable object has finished.
    Finished
  };

protected:
  virtual ~runnable_base(void) = default;
};

} // namespace stapl

#endif
