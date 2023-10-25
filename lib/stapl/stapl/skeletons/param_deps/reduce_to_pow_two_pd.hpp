/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_REDUCE_TO_POW_TWO_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_REDUCE_TO_POW_TWO_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/identity_helpers.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This parametric dependency is used in the preprocessing phase
/// of various algorithms that only accept power-of-two input sizes.
///
/// This method of size reduction is known as 2-to-1 preprocessing
/// phase which is used in reduction algorithm. Basically, if the
/// input size is n and the closest smaller power-of-two is m. The
/// first 2*(n - m) = 2*r elements are combined given the @c Op in
/// order to form r elements. The other r elements are just copied
/// using the @c stapl::identity.
///
/// @tparam Op  the workfunction to be applied on each pair of elements
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op>
class reduce_to_pow_two_pd
  : public param_deps_defaults
{
  Op m_op;

public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = 2;

  using op_type = Op;

  reduce_to_pow_two_pd(Op op)
    : m_op(std::move(op))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <i, ...> it wraps the @c Op and sends it to the
  /// visitor. If q is the closest smaller power of two to n (r = n - q)
  /// and i < 2*r the following inputs are used:
  /// @li in<0>[i*2]
  /// @li in<0>[i*2 +1]
  ///
  /// Otherwise identity is used with the following input:
  /// @li in<0>[i+r]
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                     potentially multi-dimensional.
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about Op and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord const& skeleton_size, Coord const& coord,
               Visitor& visitor, In&& in_flow) const
  {
    std::size_t i = tuple_ops::front(coord);
    std::size_t n = tuple_ops::front(skeleton_size);
    std::size_t nearest_pow2 = 1;
    while (n != 1) {
      nearest_pow2 <<= 1;
      n >>= 1;
    }
    const std::size_t r = tuple_ops::front(skeleton_size) - nearest_pow2;

    if ((std::size_t)tuple_ops::front(coord) < r) {
      visitor(
        m_op,
        no_mapper(),
        stapl::get<0>(in_flow).consume_from(make_tuple(i * 2)),
        stapl::get<0>(in_flow).consume_from(make_tuple(i * 2 + 1)));
    }
    else {
      visitor(typename stapl::identity_selector<Op>::type(),
              no_mapper(),
              stapl::get<0>(in_flow).consume_from(make_tuple(i + r)));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of.
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
/// @brief Creates a reduce to power of two parametric dependency given a
/// @c op.
///
/// @copybrief skeletons_impl::reduce_to_pow_two_pd
///
/// @param op the workfunction to be used in the reducing nodes
///
/// @ingroup skeletonsParamDepsResize
//////////////////////////////////////////////////////////////////////

template <typename Op>
skeletons_impl::reduce_to_pow_two_pd<Op>
reduce_to_pow_two_pd(Op const& op)
{
  return skeletons_impl::reduce_to_pow_two_pd<Op>(op);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_REDUCE_TO_POW_TWO_PD_HPP
