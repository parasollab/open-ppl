/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_NOTIFY_MAP_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_NOTIFY_MAP_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>
#include <boost/utility/result_of.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief A notify_map parametric dependency is similar to a @c map_pd,
/// however, it waits for @c NotifCount notifications from the first
/// @c NotifCount flows.
///
/// Example - the inputs to a spawned element created by this skeleton
/// would be:
/// @li in<0>[idx]           {notification-only}
/// @li in<1>[idx]           {notification-only}
/// @li in<2>[idx]           {notification-only}
/// @li ...
/// @li in<NotifCount-1>[idx]{notifciation-only}
/// @li in<NotifCount>[idx]
///
/// @tparam Op          the workfunction to be applied on each element
/// @tparam NotifCount  the number of notification-only flows
///
/// @see map_pd
/// @see notify_map
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, int NotifCount, typename F>
class notify_map_pd
  : public param_deps_defaults
{
  Op m_op;
  F  m_filter;

public:
  static constexpr std::size_t in_port_size = NotifCount + 1;
  static constexpr std::size_t op_arity     = 1;

  using op_type = Op;

  notify_map_pd(Op op, F f)
    : m_op(std::move(op)),
      m_filter(std::move(f))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <idx, ...> it wraps the @c Op with the
  /// following inputs and sends it to the visitor along with the
  /// @c m_op
  /// @li in<0>[idx]
  /// @li in<1>[idx]
  /// @li ...
  ///
  /// @param coord        <NotifCount, j, k, ...> where NotifCount < n,
  ///                     j < m, k < p
  /// @param visitor      the information about Op and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord const& /*skeleton_size*/, Coord const& coord,
               Visitor& visitor, In&& in_flow) const
  {
    auto&& idx = tuple_ops::front(coord);

    apply_case_of(
      idx, visitor, std::forward<In>(in_flow),
      stapl::make_index_sequence<NotifCount>()
    );
  }

private:
  template <typename Index, typename Visitor, typename In,
            std::size_t... Indices>
  void apply_case_of(Index&& index, Visitor& visitor, In&& in_flow,
                     stapl::index_sequence<Indices...>&&) const
  {

    visitor(
      visitor.notification_list({
        stapl::get<Indices>(in_flow).depend_on(
          make_tuple(std::forward<Index>(index)))...}),
      m_op,
      no_mapper(),
      stapl::get<sizeof...(Indices)>(in_flow).consume_from(
        make_tuple(std::forward<Index>(index)), m_filter));
  }
public:

  //////////////////////////////////////////////////////////////////////
  /// @brief determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of
  ///
  /// @tparam FlowIndex the flow index to which this request is sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  std::size_t consumer_count(Size const&  /*skeleton_size*/,
                             Coord const& /*producer_coord*/,
                             FlowIndex) const
  {
    return 1;
  }

public:
  Op get_op() const
  {
    return m_op;
  }

  F get_filter() const
  {
    return m_filter;
  }

  void define_type(typer& t)
  {
    t.member(m_op);
    t.member(m_filter);
  }
};

} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates a zip parametric dependency given a @c op. This method
/// is used whenever filtering is needed on the input edge.
///
/// @copybrief skeletons_impl::notify_map_pd
///
/// @ingroup skeletonsParamDeps
//////////////////////////////////////////////////////////////////////
template <int NotifCount = 1,
          typename Op,
          typename F    = skeletons::no_filter>
skeletons_impl::notify_map_pd<Op, NotifCount, F>
notify_map_pd(Op const& op, F const& f = F())
{
  return skeletons_impl::notify_map_pd<Op, NotifCount, F>(op, f);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_NOTIFY_MAP_PD_HPP
