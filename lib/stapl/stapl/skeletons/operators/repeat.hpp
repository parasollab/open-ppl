/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_OPERATORS_REPEAT_HPP
#define STAPL_SKELETONS_OPERATORS_REPEAT_HPP

#include <type_traits>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/flows/repeat_flows.hpp>
#include "repeat_impl.hpp"

namespace stapl {
namespace skeletons {
namespace result_of {

template <typename Flows, typename S, typename SizeF>
using repeat = skeletons_impl::repeat<
                 typename std::decay<S>::type,
                 typename std::decay<SizeF>::type,
                 stapl::default_type<Flows, flows::repeat_flows::piped>>;

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief Repeat skeleton repeats an enclosed skeleton for the
/// lazy size provided by @c size_functor. You can see examples of
/// its usage in @c tree, @c reverse_tree, @c butterfly, etc.
///
/// @tparam Flows        the flow to be used for the @c repeat. The
///                      default flow for a repeat skeleton is piped
/// @param  skeleton     the enclosed skeleton to be repeated
/// @param  size_functor the size of a repeat skeleton is not known
///                      until the input size is known. This parameter
///                      is a functor that given the input size
///                      determines the number of iterations of this
///                      repeat skeleton
/// @return a repeat skeleton with a given lazy size functor
///
/// @see skeletons_impl::repeat
/// @see flows::repeat_flows::piped
///
/// @ingroup skeletonsOperators
//////////////////////////////////////////////////////////////////////
template <typename Flows = stapl::use_default,
          typename S, typename SizeF>
result_of::repeat<Flows, S, SizeF>
repeat(S&& skeleton, SizeF&& size_functor)
{
  return result_of::repeat<Flows, S, SizeF>(
           std::forward<S>(skeleton),
           std::forward<SizeF>(size_functor));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_OPERATORS_REPEAT_HPP
