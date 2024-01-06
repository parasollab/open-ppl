/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/context.hpp>
#include <stapl/runtime/exception.hpp>
#include <stapl/runtime/non_rmi/external.hpp>

namespace stapl {

// Returns the location ids that are going to make the external call
std::set<unsigned int> external_callers(void)
{
  using namespace runtime;

  std::set<unsigned int> ec;

  context* const ctx = this_context::try_get();
  if (!ctx) {
    // only one location here
    ec.insert(0);
  }
  else {
    // we know we have specific number of processes, we are searching for the
    // leader locations
    gang_md const& g  = ctx->get_gang_md();
    const auto nlocs  = g.size();
    const auto nprocs = g.get_description().get_num_processes();
    std::set<process_id> pids;
    for (gang_md::size_type i = 0; i<nlocs && ec.size()!=nprocs; ++i) {
      const process_id pid = g.get_process_id(i);
      if (pids.find(pid)==pids.end()) {
        pids.insert(pid);
        ec.insert(i);
      }
    }
    STAPL_RUNTIME_ASSERT(ec.size()==nprocs);
  }
  return ec;
}

} // namespace stapl
