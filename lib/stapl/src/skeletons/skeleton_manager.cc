/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
#include <stapl/runtime/serialization/typer_traits.hpp>
#include <stapl/skeletons/executors/skeleton_manager.hpp>

namespace stapl {
namespace skeletons {

skeleton_manager::skeleton_manager(void)
  : m_is_done(false)
{ }

//////////////////////////////////////////////////////////////////////
/// @brief This method resumes the spawning process by spawning the
/// element in the front of the memento deque if it is not a lazy
/// element.
///
/// If all the elements of the memento double-ended queue are already
/// resumed and there is nothing else left to spawn, the skeleton manager
/// assumes it is done with the spawning process and will not be invoked
/// anymore by the @c paragraph.
//////////////////////////////////////////////////////////////////////
void skeleton_manager::resume()
{
  if (m_memento_stack.is_empty()) {
    this->m_is_done = true;
  } else {
    m_memento_stack.resume();
  }
}

} // namespace skeletons
} // namespace stapl