/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_EXPAND_FROM_POW_TWO_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_EXPAND_FROM_POW_TWO_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/identity_helpers.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>


namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This parametric dependency is used in the postprocessing phase
/// of various algorithms that only accept power-of-two input sizes.
///
/// This parametric dependency is used in various algorithms such as
/// @c broadcast to propagate the results of a power-of-two skeleton
/// to an arbitrary size.
///
/// @tparam Op           the workfunction to be applied on each pair
///                      of elements
/// @tparam F            the filter to be applied on the data before
///                      expanding
/// @tparam is_pos_aware whether the workfunction applied needs to know
///                      the position of the splitter in the dependence
///                       graph
/// @see reduce_to_pow_two_pd
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename F, bool is_position_aware, bool SetResult>
class expand_from_pow_two_pd
  : public param_deps_defaults
{
  using result_mapper_t =
    typename sink_traits<tags::expand_from_pow_two, 1>::result_mapper_type;

  Op m_op;
  F  m_filter;
public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = 1;

  using op_type      = Op;

  expand_from_pow_two_pd(Op op, F filter)
    : m_op(std::move(op)),
      m_filter(std::move(filter))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <i, ...> it wraps the @c Op and sends it to
  /// the visitor. If q is the closest smaller power of two to n
  /// (r = n - q)
  /// and i < 2*r the following input is used:
  /// @li in<0>[i/2]
  ///
  /// Otherwise the following input is used:
  /// @li in<0>[i-r]
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
    std::size_t n = tuple_ops::front(skeleton_size);
    std::size_t nearest_pow_2 = 1;
    while (n != 1) {
      nearest_pow_2 <<= 1;
      n >>= 1;
    }
    const std::size_t i = tuple_ops::front(coord);
    const std::size_t r = tuple_ops::front(skeleton_size) - nearest_pow_2;

    bool is_downedge = (i < 2 * r) and (i % 2 != 0);

    apply_set_position(
      std::integral_constant<bool, is_position_aware>(),
      m_op, i, is_downedge);

    apply_set_position(
      std::integral_constant<
        bool,
        is_position_aware and
        (not std::is_same<F, skeletons::no_filter>::value)>(),
      m_filter, i, is_downedge);

    if (i < 2 * r) {
      visitor. template operator()<SetResult>(m_op, no_mapper(),
              stapl::get<0>(in_flow).consume_from(make_tuple(i / 2), m_filter));
    }
    else {
     visitor. template operator()<SetResult>(
       typename stapl::identity_selector<Op>::type(),
       no_mapper(),
       stapl::get<0>(in_flow).consume_from(make_tuple(i - r)));
    }
  }

private:
  template <typename Oper>
  void
  apply_set_position(
    std::true_type,
    Oper const& op, std::size_t cur_index, bool is_downedge) const
  {
    const_cast<Oper&>(op).set_position(cur_index, is_downedge);
  }

  template <typename Oper>
  void
  apply_set_position(
    std::false_type,
    Oper const& op, std::size_t cur_index, bool is_downedge) const
  { }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of.
  ///
  /// @tparam FlowIndex the flow index to which this request is sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  std::size_t consumer_count(Size const&  skeleton_size,
                             Coord const& producer_coord,
                             FlowIndex) const
  {
    std::size_t n = tuple_ops::front(skeleton_size);
    std::size_t nearest_pow_2 = 1;
    while (n != 1) {
      nearest_pow_2 <<= 1;
      n >>= 1;
    }
    std::size_t i = tuple_ops::front(producer_coord);
    std::size_t r = tuple_ops::front(skeleton_size) - nearest_pow_2;
    return (i < r) ? 2 : 1;
  }

private:
  template <typename Coord>
  int get_result_id_helper(std::true_type,
                           Coord const& skeleton_size,
                           Coord const& coord) const
  {
    return result_mapper_t()(tuple_ops::front(coord));
  }

  template <typename Coord>
  int get_result_id_helper(std::false_type,
                           Coord const&,
                           Coord const&) const
  {
    return -1;
  }

public:
  template <typename Coord>
  int get_result_id(Coord const& skeleton_size, Coord const& coord) const
  {
    return get_result_id_helper(std::integral_constant<bool, SetResult>(),
                                skeleton_size, coord);
  }

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
/// @brief Creates a expand from power of two parametric dependencies given
/// a @c op.
///
/// @copybrief skeletons_impl::expand_from_pow_two_pd
///
/// @param  op        the workfunction to be applied on the inputs
/// @tparam pos_aware whether @c op needs to know the position of the
///                   created element in the dependence graph
///
/// @ingroup skeletonsParamDepsResize
//////////////////////////////////////////////////////////////////////
template <bool pos_aware,
          bool SetResult = false,
          typename Op,
          typename Filter = skeletons::no_filter>
skeletons_impl::expand_from_pow_two_pd<Op, Filter, pos_aware, SetResult>
expand_from_pow_two_pd(Op const& op, Filter const& filter = Filter())
{
  return skeletons_impl::expand_from_pow_two_pd<
                           Op, Filter, pos_aware, SetResult>(op, filter);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_EXPAND_FROM_POW_TWO_PD_HPP
