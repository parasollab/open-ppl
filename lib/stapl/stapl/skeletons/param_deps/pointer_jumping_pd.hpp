/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_POINTER_JUMPING_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_POINTER_JUMPING_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/filters.hpp>
#include <stapl/skeletons/utility/identity_helpers.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief A pointer-jumping parametric dependency can be used in various
/// skeletons. One good example is @c hillis_steele_scan in which scan
/// is done in a non-work optimal but smaller tree.
///
/// @tparam Op the operation that is used in each element
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op>
class pointer_jumping_pd
  : public param_deps_defaults
{
  Op m_op;

public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = 2;

  using consumer_count_filter_type = skeletons::filters::filter<2>;
  using op_type                    = Op;

  explicit pointer_jumping_pd(Op op)
    : m_op(std::move(op))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <i, h, ...> it wraps the @c WF with the
  /// following inputs and sends it to the visitor along with the @c m_wf
  /// if (i >= 2<sup>h</sup>) :
  /// @li in<0>[i]
  /// @li in<0>[i - 2<sup>h</sup>]
  ///
  /// otherwise :
  /// @li in<0>[i]
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                     potentially multi-dimensional.
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about WF and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord const& skeleton_size, Coord const& coord,
               Visitor& visitor, In&& in_flow) const
  {
    using stapl::get;
    std::size_t idx = tuple_ops::front(coord);
    std::size_t level = get<1>(coord);
    std::size_t t = 1ul << level;
    if (idx >= t) {
      visitor(m_op,
              no_mapper(),
              get<0>(in_flow).consume_from(make_tuple(idx, level - 1)),
              get<0>(in_flow).consume_from(make_tuple(idx - t, level - 1)));
    }
    else {
      visitor(typename stapl::identity_selector<Op>::type(),
              no_mapper(),
              get<0>(in_flow).consume_from(make_tuple(idx, level - 1)));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of.
  ///
  /// @param  skeleton_size  the size of this skeleton
  /// @param  producer_coord the coordinate of the producer element
  ///                        which is providing data to this parametric
  ///                        dependency
  /// @tparam FlowIndex      the flow index on which this request is
  ///                        sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  std::size_t consumer_count(Size const& skeleton_size,
                             Coord const& producer_coord,
                             FlowIndex const& /*flow_index*/) const {
    std::size_t idx = tuple_ops::front(producer_coord);
    //remember requester is one level before
    std::size_t h = stapl::get<1>(producer_coord) + 1;
    std::size_t t = tuple_ops::front(skeleton_size) - (1ul << h);
    return (idx < t) ? 2 : 1;
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
/// @brief Creates a pointer-jumping parametric dependency given a @c op.
///
/// @copybrief skeletons_impl::pointer_jumping_pd
///
/// @ingroup skeletonsParamDepsExchange
//////////////////////////////////////////////////////////////////////
template <typename Op>
skeletons_impl::pointer_jumping_pd<Op>
pointer_jumping_pd(Op const& op) {
  return skeletons_impl::pointer_jumping_pd<Op>(op);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_POINTER_JUMPING_PD_HPP
