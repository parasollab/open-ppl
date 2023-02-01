/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_PRE_BROADCAST_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_PRE_BROADCAST_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>
#include <stapl/utility/integer_sequence.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Creates a set_result parametric dependency given a @c op.
///        Basically this parametric dependency is only used for
///        sinking values of skeletons which couldn't currently
///        set their results directly without using sink_value skeleton.
///
/// @tparam Arity      the arity of set_result
/// @tparam  Op         the workfunction to be used in each set_result
///                    parametric dependency
/// @tparam Span       the span for the previous skeleton that is passed
///                    to set result for detecting if the corresponding task
///                    is spanned or not
/// @tparam SetResult  whether put the task result on the result container
///                    or not

/// @ingroup skeletonsParamDeps
//////////////////////////////////////////////////////////////////////
template <std::size_t Arity, typename Op, typename Span, bool SetResult>
class pre_broadcast_pd
  : public param_deps_defaults
{
  Op        m_op;
public:
  static constexpr std::size_t in_port_size = Arity;
  static constexpr std::size_t op_arity     = Arity;

  using op_type      = Op;

  explicit pre_broadcast_pd(Op op)
    : m_op(std::move(op))
  { }

private:
  template <typename Coord, typename Visitor, typename In,
            std::size_t... Indices>
  void apply_case_of(Coord const& skeleton_size, Visitor& visitor, In&& in_flow,
                     index_sequence<Indices...>&&) const
  {
    visitor.template operator()<SetResult>(
      m_op,
      no_mapper(),
      stapl::get<Indices>(in_flow).consume_from(
        make_tuple(skeleton_size - 1))...
    );
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <idx, ...> it wraps the @c Op with the
  /// following inputs and sends it to the visitor along with the
  /// @c m_op
  /// @li in<0>[idx]
  /// @li in<1>[idx]
  /// @li ...
  ///
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about Op and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord const& skeleton_size, Coord const&/* coord*/,
               Visitor& visitor, In&& in_flow) const
  {
    apply_case_of(
      tuple_ops::front(skeleton_size),
      visitor, std::forward<In>(in_flow),
      stapl::make_index_sequence<Arity>());
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of
  ///
  /// @param  skeleton_size  the size of skeleton dimension
  /// @param  coord          the producer coordination
  /// @tparam FlowIndex      the flow index to which this request is sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  std::size_t consumer_count(Size const&  skeleton_size,
                             Coord const& coord,
                             FlowIndex const& /*flow_idx*/) const
  {
    return tuple_ops::front(coord) == tuple_ops::front(skeleton_size) - 1 ? 1
                                                                          : 0;
  }

  Op get_op() const
  {
    return m_op;
  }

  void define_type(typer& t)
  {
    t.member(m_op);
  }
};

} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates a set_result parametric dependency given a @c op.
///        Basically this parametric dependency is only used for
///        sinking values of skeletons which couldn't currently
///        set their results directly without using sink_value skeleton.
///
/// @tparam Arity      the arity of set_result
/// @tparam Span       the span for the previous skeleton that is passed
///                    to set result for detecting if the corresponding task
///                    is spanned or not
/// @tparam SetResult  whether put the task result on the result container
///                    or not
/// @param  op         the workfunction to be used in each set_result
///                    parametric dependency
///
/// @ingroup skeletonsParamDeps
//////////////////////////////////////////////////////////////////////
template <std::size_t Arity, typename Span, bool SetResult, typename Op>
skeletons_impl::pre_broadcast_pd<Arity, Op, Span, SetResult>
pre_broadcast_pd(Op const& op)
{
  return skeletons_impl::pre_broadcast_pd<Arity, Op, Span, SetResult>(op);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_PRE_BROADCAST_PD_HPP
