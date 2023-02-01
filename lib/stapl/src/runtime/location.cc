/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/location_md.hpp>
#include <stapl/runtime/executor/gang_executor.hpp>
#include <stapl/runtime/executor/scheduler/sched.hpp>
#include <stapl/runtime/type_traits/aligned_storage.hpp>

namespace stapl {

namespace runtime {

// Total required size in bytes for a location_md object and its runqueue::impl
const auto sizeof_location_md =
  aligned_size(sizeof(location_md), runqueue::required_alignment());


// Returns a properly aligned pointer to construct a runqueue::impl
void* location_md::get_ptr(void) noexcept
{
  return (reinterpret_cast<char*>(this) + sizeof_location_md);
}


// Allocates a new gang_executor for the location_md
std::unique_ptr<executor_base> location_md::make_executor(void)
{
  using executor_type = gang_executor<default_gang_scheduler>;
  return std::unique_ptr<executor_base>{new executor_type(*this)};
}


// Allocates space for a location_md object and its runqueue::impl
void* location_md::operator new(std::size_t)
{
  return ::operator new(sizeof_location_md + runqueue::required_size());
}


// Frees the space allocated for a location_md object and its runqueue::impl
void location_md::operator delete(void* p)
{
  ::operator delete(p);
}

} // namespace runtime

} // namespace stapl
