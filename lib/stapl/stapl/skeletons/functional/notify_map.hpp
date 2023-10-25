/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_NOTIFY_MAP_HPP
#define STAPL_SKELETONS_FUNCTIONAL_NOTIFY_MAP_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/param_deps/notify_map_pd.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a notify_map skeleton
/// by exposing only the necessary information in its representation.
///
/// A notify_map skeleton is similar to a @c map skeleton, but it also
/// receives @c NotifCount notifications from its first @C NotifCount
/// flow.
///
/// This skeleton is used when consecutive maps are non-side-effect-free
/// and work on the same set of inputs.
///
/// This abstraction not only makes the reconstruction of a
/// a notify_map skeleton easier, but also reduces the symbol size for a
/// notify_map skeleton, hence, reducing the total compilation time.
///
/// @tparam Op          the workfunction to be used in each notify_map
///                     parametric dependency.
/// @tparam NotifCount  the number of notification flows to notify_map
/// @tparam Span        the iteration space for the elements in the skeleton.
/// @tparam Flows       the flow between the elements in the skeleton.
/// @tparam Filter      the filter to be applied on the result produced
///                     by the operation.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, int NotifCount, typename Span, typename Flows,
          typename Filter>
struct notify_map
  : public decltype(
             skeletons::elem<Span, Flows>(
               skeletons::notify_map_pd<NotifCount>(
                 std::declval<Op>(), std::declval<Filter>())))
{
  using skeleton_tag_type = tags::notify_map<NotifCount>;
  using base_type = decltype(
                      skeletons::elem<Span, Flows>(
                      skeletons::notify_map_pd<NotifCount>(
                        std::declval<Op>(), std::declval<Filter>())));

  notify_map(Op const& op, Filter const& filter)
    : base_type(
        skeletons::elem<Span, Flows>(
          skeletons::notify_map_pd<NotifCount>(op, filter))
      )
  { }

  Op get_op(void) const
  {
    return base_type::nested_skeleton().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

}

namespace result_of {

template <int NotifCount,
          typename Span,
          typename Flows,
          typename Op,
          typename Filter>
using notify_map = skeletons_impl::notify_map<
                     typename std::decay<Op>::type,
                     NotifCount, Span, Flows,
                     typename std::decay<Filter>::type>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief A notify_map is similar to @c map skeleton but it also depends
/// on a notification from the first @c NotifCount flows passed to it.
///
/// @tparam NotifCount the number of notification flows to notify_map
/// @tparam Span       the iteration space for the elements in this
///                    skeleton
/// @tparam Flows      the flow to be used for this skeleton
/// @param  op         the workfunction to be used in each notify_map
///                    parametric dependency.
/// @param  filter     the filter function to be used on the producer side
///                    before sending data to a parametric dependency
///
/// @return a notify_map skeleton with a filter on the incoming edges
///
/// @see notify_map
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <int NotifCount  = 1,
          typename Span   = stapl::use_default,
          typename Flows  = stapl::use_default,
          typename Op,
          typename Filter = skeletons::no_filter>
result_of::notify_map<NotifCount, Span, Flows, Op, Filter>
notify_map(Op&& op, Filter&& filter = Filter())
{
  return result_of::notify_map<NotifCount, Span, Flows, Op, Filter>(
           std::forward<Op>(op),
           std::forward<Filter>(filter));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_NOTIFY_MAP_HPP
