/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/paragraph/paragraph.hpp>

namespace stapl {

namespace paragraph_impl {

auto termination_value(task_graph& tg)
  -> decltype(tg.termination_value())
{
  return tg.termination_value();
}

} // namespace paragraph_impl

} // namespace stapl
