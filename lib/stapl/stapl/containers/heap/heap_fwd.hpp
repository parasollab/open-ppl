/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_HEAP_FWD_HPP
#define STAPL_CONTAINERS_HEAP_FWD_HPP

#include <stapl/utility/use_default.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @ingroup pheap
//////////////////////////////////////////////////////////////////////
template<typename T,
         typename Comp   = use_default,
         typename PS     = use_default,
         typename M      = use_default,
         typename Traits = use_default>
class heap;

} // namespace stapl

#endif // STAPL_CONTAINERS_HEAP_FWD_HPP
