/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_SET_RESULT_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_SET_RESULT_PD_HPP

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
/// @tparam Arity      the number of inputs are passed to the skeleton
/// @tparam SetResult  whether the skeleton should set the task
///                    results on the pg edge container or not
/// @tparam Span       the span for the previous skeleton that is passed
///                    to @c set result_pd for detecting if the corresponding
///                    corresponding task is spanned or not
/// @tparam  Op        the workfunction to be used in each set_result
///                    parametric dependency
/// @ingroup skeletonsParamDeps
//////////////////////////////////////////////////////////////////////
template <std::size_t Arity, typename Op, typename Span, bool SetResult>
class set_result_pd
  : public param_deps_defaults
{
  Op        m_op;
  Span      m_span;
public:
  static constexpr std::size_t in_port_size = Arity;
  static constexpr std::size_t op_arity     = Arity;

  using op_type = Op;

  explicit set_result_pd(Op op)
    : m_op(std::move(op)),
      m_span(Span())
  { }

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
  void case_of(Coord const&, Coord const& coord,
               Visitor& visitor, In&& in_flow) const
  {
    apply_case_of(
      tuple_ops::front(coord),
      visitor, std::forward<In>(in_flow),
      stapl::make_index_sequence<op_arity>());
  }

private:
  template <typename Index, typename Visitor, typename In,
            std::size_t... Indices>
  void apply_case_of(Index&& index,
                     Visitor& visitor, In&& in_flow,
                     index_sequence<Indices...>&&) const
  {
    visitor.template operator()<SetResult>(
      m_op,
      no_mapper(),
      stapl::get<Indices>(in_flow).consume_from(
        make_tuple(std::forward<Index>(index))
      )...
    );
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of
  ///
  /// @param  producer_coord the producer coordination
  /// @param  skeleton_size  the size of skeleton dimension
  /// @tparam FlowIndex      the flow index to which this request is sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  std::size_t consumer_count(Size const&  skeleton_size,
                             Coord const& producer_coord,
                             FlowIndex const& /*flow_idx*/) const
  {
    return m_span.should_spawn(skeleton_size, producer_coord) ? 1 : 0;
  }

  template <typename Coord>
  int get_result_id(Coord const&, Coord const&) const
  {
    return 0;
  }

  Op get_op() const
  {
    return m_op;
  }

  void define_type(typer& t)
  {
    t.member(m_op);
    t.member(m_span);
  }
};

} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates a set_result parametric dependency given a @c op.
///        Basically this parametric dependency is only used for
///        sinking values of skeletons which couldn't currently
///        set their results directly without using sink_value skeleton.
///
/// @tparam Arity      the number of inputs are passed to the skeleton
/// @tparam SetResult  whether the skeleton should set the task
///                    results on the pg edge container or not
/// @tparam Span       the span for the previous skeleton that is passed
///                    to @c set result_pd for detecting if the corresponding
///                    corresponding task is spanned or not
/// @param  op         the workfunction to be used in each set_result
///                    parametric dependency
/// @ingroup skeletonsParamDeps
//////////////////////////////////////////////////////////////////////
template <std::size_t Arity, typename Span, bool SetResult, typename Op>
skeletons_impl::set_result_pd<Arity, Op, Span, SetResult>
set_result_pd(Op const& op)
{
  return skeletons_impl::set_result_pd<Arity, Op, Span, SetResult>(op);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_SET_RESULT_PD_HPP
