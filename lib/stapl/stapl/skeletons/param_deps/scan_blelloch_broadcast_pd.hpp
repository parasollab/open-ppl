/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_SCAN_BLELLOCH_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_SCAN_BLELLOCH_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/identity_helpers.hpp>
#include <stapl/skeletons/utility/filters.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Tis parametric dependency is used in the broadcast tree of
/// Blelloch scan, in which each element applies the operation on
/// inputs from the previous level of the tree and inputs from the
/// reduction tree. Blelloch scan is an exclusive scan algorithm.
///
/// @tparam Op the operation that is used in each element
/// @tparam T  the type of the initial value
///
/// @see Prefix Sums and Their Applications, Guy E. Blelloch
/// @see scan_blelloch
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename T>
class scan_blelloch_broadcast_pd
  : public param_deps_defaults
{
  Op      m_op;
  T const m_initial_value;

public:
  static constexpr std::size_t in_port_size = 3;
  static constexpr std::size_t op_arity     = 2;

  using consumer_count_filter_type = skeletons::filters::filter<2>;
  using op_type                    = Op;

  scan_blelloch_broadcast_pd(Op op, T  initial_value)
    : m_op(std::move(op)),
      m_initial_value(std::move(initial_value))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <i, level, ...> and skeleton_size is <n, m, p, ...>
  /// it wraps the @c WF with the following inputs and sends it to the
  /// visitor:
  /// if (i + 1) % 2<sup>m-level+1</sup> == 0 then the given @c m_op is used:
  /// @li level == 0 ? constant(initial_value) : in<0>(i)
  /// @li in<1>[(i - 2<sup>m - level</sup>, m-level-1)]
  ///
  /// else @c stapl::identity is used instead of @c m_op
  /// @li level == 0 ? constant(initial_value) : in<0>(i + 2<sup>m-level</sup>)
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
    std::size_t i = tuple_ops::front(coord);
    std::size_t level = get<1>(coord);
    std::size_t max_level = get<1>(skeleton_size) - 1;
    std::size_t dist = std::size_t(1) << (max_level - level);

    //if it is the R element
    if ((i+1) % (dist << 1) == 0){
      if (level == 0) {
        visitor(
          m_op,
          no_mapper(),
          constant_input(m_initial_value),
          get<1>(in_flow).consume_from(
            make_tuple(i - dist, max_level - level - 1)));
      }
      else {
        visitor(
          m_op,
          no_mapper(),
          get<0>(in_flow).consume_from(
            make_tuple(i, level - 1)),
          get<1>(in_flow).consume_from(
            make_tuple(i - dist, max_level - level - 1)));
      }
    }
    //if it is the L element
    else {
      typedef typename identity_selector<Op>::type identity_op_t;
      if (level == 0) {
        visitor(identity_op_t(),
                no_mapper(),
                constant_input(m_initial_value));
      }
      else {
        visitor(identity_op_t(),
                no_mapper(),
                get<0>(in_flow).consume_from(make_tuple(i + dist, level - 1)));
      }
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
  std::size_t consumer_count(Size const&  skeleton_size,
                             Coord const& producer_coord,
                             FlowIndex) const
  {
    //the reduction result is ignored in Blelloch algorithm
    if (stapl::get<0>(skeleton_size) ==
        (std::size_t(1) << (stapl::get<1>(producer_coord) + 1))) {
      return 0;
    }
    else {
      // remember the input will have a (x, -1) coordinate
      std::size_t i = stapl::get<0>(producer_coord);
      std::size_t dist = (std::size_t(1) <<
                            (stapl::get<1>(producer_coord) + 2));
      std::size_t offset = (dist / 2) - 1;

      return ((i - offset) % dist == 0) ? 1 : 0;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copybrief consumer_count
  ///
  /// This specialization is used when the request is only sent to the
  /// first input flow of the current skeleton.
  ///
  /// @tparam FlowIndex the flow index to which this request is sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord>
  std::size_t consumer_count(Size const&  /*skeleton_size*/,
                             Coord const& /*producer_coord*/,
                             std::integral_constant<int, 0>) const
  {
    return 2;
  }

private:
  template <typename Coord>
  std::size_t
  crossover_size(Coord const& skeleton_size, Coord const& coord) const
  {
    return 1ul << (stapl::get<1>(skeleton_size) - stapl::get<1>(coord) - 1);
  }

public:
  Op get_op() const
  {
    return m_op;
  }

  void define_type(typer& t)
  {
    t.member(m_op);
    t.member(m_initial_value);
  }
};

} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates a parametric dependency for the broadcast phase of
/// the exclusive Blelloch scan using the given Op.
///
/// @copybrief skeletons_impl::scan_blelloch_broadcast_pd
///
/// @ingroup skeletonsParamDepsAlgorithm
//////////////////////////////////////////////////////////////////////
template <typename Op, typename T>
skeletons_impl::scan_blelloch_broadcast_pd<Op, T>
scan_blelloch_broadcast_pd(Op const& sum_wf, T const& initial_value)
{
  return skeletons_impl::scan_blelloch_broadcast_pd<Op, T>(sum_wf,
                                                          initial_value);
}

} // namespace skeletons
} // namespace stapl
#endif // STAPL_SKELETONS_PARAM_DEPS_SCAN_BLELLOCH_PD_HPP
