/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/skeletons/executors/memento.hpp>

namespace stapl {
namespace skeletons {

memento::memento()
  : callbacks(new internal_stack_t())
{ }

//////////////////////////////////////////////////////////////////////
/// @brief the front element of the memento stack at times needs to
/// stays untouched until some criteria is met. The entities that can
/// remain untouched are called lazy. This method checks if the
/// element is lazy.
///
/// @return true only if the element on top of the memento stack is
///              lazy
//////////////////////////////////////////////////////////////////////
bool memento::front_is_lazy()
{
  return callbacks->front().is_lazy();
}

std::size_t memento::size() const
{
  return callbacks->size();
}

bool memento::is_empty()
{
  return callbacks->empty();
}

void memento::pop()
{
  stapl_assert(callbacks->size() > 0,
               "Pop is called on an empty memento stack");
  callbacks->pop_front_and_dispose(
    skeletons_impl::memento_element_disposer());
}

//////////////////////////////////////////////////////////////////////
/// @brief This method resumes the spawning process of the front
/// element of the memento dequeue as long as there is nothing else to
/// do and the front element is not lazy. If all the elements of the
/// memento double-ended queue are already resumed and there is nothing
/// else left to do nothing will be done.
///
/// The skeleton manager which holds this memento stack will finish
/// the spawning process if there is nothing left to spawn in this
/// memento stack.
///
/// @param ignore_lazyness if true, continue the spawning process even
///                        if the front element of the memento is lazy
//////////////////////////////////////////////////////////////////////
void memento::resume(bool ignore_lazyness)
{
  if ((ignore_lazyness || !this->front_is_lazy()) &&
      !is_empty()) {
    auto&& f = callbacks->front(); //store f as f() may modify the memento
    this->pop_keep();
    f();
    skeletons_impl::memento_element_disposer()(&f);
  }
}

void memento::pop_keep()
{
  stapl_assert(callbacks->size() > 0,
               "Pop is called on an empty memento stack");
  callbacks->pop_front();
}

void memento::push_back(element_type* element, bool is_lazy)
{
  element->set_is_lazy(is_lazy);
  callbacks->push_back(*element);
}

void memento::push_front(element_type* element, bool is_lazy)
{
  element->set_is_lazy(is_lazy);
  callbacks->push_front(*element);
}

} // namespace skeletons
} // namespace stapl