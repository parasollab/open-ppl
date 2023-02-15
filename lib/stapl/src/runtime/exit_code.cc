/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/exit_code.hpp>
#include <stapl/runtime/runtime.hpp>
#include <stapl/runtime/synchronization.hpp>

namespace stapl {

// Returns from an application
exit_code::exit_code(int code)
: m_code(code)
{
  using namespace runtime;

  if (m_code==EXIT_SUCCESS) {
    // finish all RMIs
    rmi_fence(this_context::get());
  }
}


// Barrier for all locations exiting an application
void exit_code::wait(void) const
{
  using namespace runtime;

  // unsuccessful execution
  if (m_code!=EXIT_SUCCESS)
    return;

  // termination detection and fence for world executor
  auto& ctx = this_context::get();
  auto& l   = ctx.get_location_md();
  rmi_fence(ctx, [&l]
                 {
                   auto* const ex = l.try_get_executor();
                   if (ex) {
                     while ((*ex)()==runnable_base::Active);
                   }
                   return true;
                 });
}

} // namespace stapl
