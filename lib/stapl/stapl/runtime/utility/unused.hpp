/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_UNUSED_HPP
#define STAPL_RUNTIME_UTILITY_UNUSED_HPP

#include "../exception.hpp"
#include <algorithm>
#include <typeinfo>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Function to remove warnings on unused variables.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename... T>
void unused(T&&...)
{ }

} // namespace runtime

} // namespace stapl

#endif
