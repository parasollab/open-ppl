/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_BROADCAST_TO_LOCS_HPP
#define STAPL_SKELETONS_FUNCTIONAL_BROADCAST_TO_LOCS_HPP

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/spans/per_location.hpp>
#include "broadcast.hpp"

namespace stapl {
namespace skeletons {
namespace skeletons_impl {


template <typename Flows, typename Tag, bool SetResult>
struct broadcast_to_locs
  : public skeletons_impl::broadcast<
             stapl::identity_op, Flows, spans::per_location, Tag, SetResult>


{
  using skeleton_tag_type = tags::broadcast_to_locs;
  using op_type          = stapl::identity_op;
  using base_type =
    skeletons_impl::broadcast<
      stapl::identity_op, Flows, spans::per_location, Tag, SetResult>;

  broadcast_to_locs(void)
    : base_type(stapl::identity_op())
  { }

  op_type get_op(void) const
  {
    return stapl::identity_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

} // namespace skeletons_impl


namespace result_of {

template <typename Tag,
          typename Flows,
          bool SetResult>
using broadcast_to_locs = skeletons_impl::
                            broadcast_to_locs<Flows, Tag, SetResult>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief This broadcast skeleton is used when the result of the
/// broadcast should have a representative on each location.
/// Therefore, a @c span::per_location is used in this skeleton.
/// @c Flows are still allowed to be customized for this skeleton.
///
/// @tparam SetResult whether the skeleton should set the task
///                   results on the pg edge container or not
/// @tparam Flows     the customized flow to be used in the
///                   @c reverse_tree
/// @tparam Tag       determines the type of the broadcast skeleton
/// @return a broadcast skeleton that broadcasts one element to each
///         location with custom flows
///
/// @see broadcast
/// @see spans::per_location
///
/// @ingroup skeletonsFunctionalBroadcast
//////////////////////////////////////////////////////////////////////
template <bool SetResult  = false,
          typename Tag    = stapl::use_default,
          typename Flows  = stapl::use_default>
inline result_of::broadcast_to_locs<Tag, Flows, SetResult>
broadcast_to_locs(void)
{
  return result_of::broadcast_to_locs<
           Tag, Flows, SetResult>();
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_BROADCAST_TO_LOCS_HPP
