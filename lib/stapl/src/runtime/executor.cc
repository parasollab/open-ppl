/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime/executor/executor_base.hpp>
#include <stapl/runtime/executor/anonymous_executor.hpp>
#include <stapl/runtime/exception.hpp>
#include <stapl/runtime/this_context.hpp>
#include <stapl/runtime/concurrency/thread_local_storage.hpp>
#include <atomic>
#include <iostream>

namespace stapl {

/// @ref anonymous_executor instance.
static STAPL_RUNTIME_THREAD_LOCAL(anonymous_executor*, anon_exec)

/// @ref anonymous_executor shared among threads.
static std::atomic<anonymous_executor*> shared_anon_exec(nullptr);


// Returns the anonymous_executor from the stack
anonymous_executor& get_anonymous_executor(void)
{
  if (!anon_exec.get())
    anon_exec.get() = shared_anon_exec;
  return *(anon_exec.get());
}

anonymous_executor::anonymous_executor(void)
: m_handle(this, no_aggregation | no_fence_information)
{
  STAPL_RUNTIME_ASSERT(!anon_exec.get());
  anon_exec.get() = this;
  // if there is no shared global executor, register my anonymous_executor
  anonymous_executor* t = nullptr;
  shared_anon_exec.compare_exchange_strong(t, this);
}

anonymous_executor::~anonymous_executor(void)
{
  // if I own the shared global executor, unregister it
  shared_anon_exec.compare_exchange_strong(anon_exec.get(), nullptr);
  anon_exec.get() = nullptr;
}


namespace runtime {

executor_base& get_executor(const gang_id gid)
{
  auto* const l = this_context::try_get_location_md(gid);
  if (!l) {
    // for now, this should always succeed - how to treat it if not?
    STAPL_RUNTIME_ERROR("Location metadata not found.");
  }

  // add executor recursively to its parent if it is not already bound
  // gang 0 executor cannot be bound to any other executor
  auto& ex = l->get_executor();
  if (gid!=0 && !ex.is_bound())
    ex.bind_to(l->get_gang_md().get_parent_id());
  return ex;
}

} // namespace runtime


executor_base::~executor_base(void)
{
  STAPL_RUNTIME_ASSERT(idle());
  retire_chunker_entry();
}

std::ostream& operator<<(std::ostream& os, executor_base& ex)
{
  ex.print(os);
  return os << std::boolalpha
            << " is_bound("              << ex.is_bound()                << ')'
            << " has_runnable_notifier(" << bool(ex.m_runnable_notifier) << ')'
            << " has_finished_notifier(" << bool(ex.m_finished_notifier) << ')'
            << std::noboolalpha;
}

} // namespace stapl
