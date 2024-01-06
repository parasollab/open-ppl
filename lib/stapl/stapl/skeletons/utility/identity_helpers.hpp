/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_IDENTITY_HELPERS_HPP
#define STAPL_SKELETONS_UTILITY_IDENTITY_HELPERS_HPP

#include <vector>
#include <stapl/skeletons/utility/utility.hpp>

namespace stapl {

struct identity_op;

//////////////////////////////////////////////////////////////////////
/// @brief An identity workfunction used in the skeleton framework can
/// be both typed and typeless. In most of the cases if the
/// workfunction used in a parametric dependency is strictly
/// typed, meaning that it has defined @c result_type, the corresponding
/// identity workfunction should be typed with the same type.
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <typename WF, bool has_result_type = has_result_type<WF>::value>
struct identity_selector
{
  using type = identity<typename WF::result_type>;
};


template <typename WF>
struct identity_selector<WF, false>
{
  using type = identity_op;
};

} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_IDENTITY_HELPERS_HPP
