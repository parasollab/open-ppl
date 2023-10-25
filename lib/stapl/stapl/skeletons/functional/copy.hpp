/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_COPY_HPP
#define STAPL_SKELETONS_FUNCTIONAL_COPY_HPP

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/functional/skeleton_traits.hpp>
#include <stapl/utility/use_default.hpp>
#include <stapl/algorithms/functional.hpp>
#include "zip.hpp"

namespace stapl {
namespace skeletons {
namespace result_of {

template <typename ValueType,
          typename SkeletonTraits>
using copy = skeletons_impl::zip<
               2,
               stapl::assign<ValueType>,
               SkeletonTraits>;

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief A copy skeleton is used when writes to views are happening.
/// The second flow to this skeleton will be writable.
///
/// @tparam ValueType the type of each element to be copied
/// @param  traits    the traits to be used (default = default_skeleton_traits)
///
/// @return a @c copy skeleton
///
/// @see zip
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename ValueType,
          typename SkeletonTraits = skeletons_impl::default_skeleton_traits>
result_of::copy<ValueType, SkeletonTraits>
copy(SkeletonTraits&& traits = SkeletonTraits())
{
  return skeletons::zip<2>(stapl::assign<ValueType>(),
                           std::forward<SkeletonTraits>(traits));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_COPY_HPP
