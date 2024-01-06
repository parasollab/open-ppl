/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_SCAN_BROADCAST_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_SCAN_BROADCAST_PD_HPP

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
/// @brief One way to do an inclusive scan is to use the algorithm
/// mentioned in <b>An Introduction to Parallel Algorithms</b> by
/// Joseph Jaja, page 48. This parametric dependency represents the
/// broadcast (top-down) traversal in the above mentioned algorithm.
///
/// @tparam Op the operation that is used in each element
///
/// @see scan
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op>
class scan_broadcast_pd
  : public param_deps_defaults
{
  Op        m_op;
public:
  static constexpr std::size_t in_port_size = 3;
  static constexpr std::size_t op_arity     = 2;

  using consumer_count_filter_type = skeletons::filters::filter<2>;
  using op_type                    = Op;

  explicit scan_broadcast_pd(Op op_wf)
    : m_op(op_wf)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <i, level, ...> and skeleton_size is <n, m, p, ...>
  /// it wraps the @c WF with the following inputs and sends it to the
  /// visitor:
  /// if i % 2 == 1 then the given @c stapl::identity is used:
  /// @li level == 0 ? in<1>[(i/2, m)] ? in<0>[i/2]
  ///
  /// else if i == 0 still @c stapl::identity is used
  /// @li in<1>[(i, m-level-1)]
  ///
  /// else @c m_op is used
  /// @li in<0>[(i-1)/2]
  /// @li in<1>[(i, m-level-1)]
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
    typedef typename stapl::identity_selector<Op>::type identity_op_t;
    if (i % 2 == 1) {
      //in this case get the data from the previous iteration
      if (level == 0) {
        if (max_level == 0) {
        visitor(identity_op_t(),
                no_mapper(),
                get<0>(in_flow).consume_from(make_tuple(i/2, max_level)));
      }
      else {
        visitor(identity_op_t(),
                no_mapper(),
                get<1>(in_flow).consume_from(make_tuple(i/2, max_level)));
        }
      }
      else {
        visitor(identity_op_t(),
                no_mapper(),
                get<0>(in_flow).consume_from(make_tuple(i/2, level - 1)));
      }
    }
    else {
      if (i == 0) {
        visitor(
          identity_op_t(),
          no_mapper(),
          get<1>(in_flow).consume_from(make_tuple(i, max_level - level - 1)));
      }
      else {
        visitor(
          m_op,
          no_mapper(),
          get<0>(in_flow).consume_from(make_tuple((i-1) / 2, level - 1)),
          get<1>(in_flow).consume_from(make_tuple(i, max_level - level - 1)));
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of.
  ///
  /// @param skeleton_size   the size of this skeleton
  /// @param producer_coord  the coordinate of the producer element
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
    if (FlowIndex::value == 1 && tuple_ops::front(skeleton_size) == 1) {
      return 0;
    }
    else {
      return tuple_ops::front(producer_coord) % 2 == 0;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copybrief consumer_count
  ///
  /// This specialization is used when the request is only sent to the
  /// first input flow of the current skeleton.
  ///
  /// @param producer_coord  the coordinate of the producer element
  ///                        which is providing data to this parametric
  ///                        dependency
  /// @tparam FlowIndex      the flow index on which this request is
  ///                        sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord>
  std::size_t consumer_count(Size const&  /*skeleton_size*/,
                             Coord const& producer_coord,
                             std::integral_constant<int, 0>) const
  {
    std::size_t level = stapl::get<1>(producer_coord);
    return tuple_ops::front(producer_coord) == ((1ul << (level + 1)) - 1) ? 1
                                                                          : 2;
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
/// @brief Creates a parametric dependency for the broadcast phase of
/// the inclusive scan in Jaja book using the given Op.
///
/// @copybrief skeletons_impl::scan_broadcast_pd
///
/// @ingroup skeletonsParamDepsAlgorithm
//////////////////////////////////////////////////////////////////////
template <typename Op>
skeletons_impl::scan_broadcast_pd<Op>
scan_broadcast_pd(Op const& sum_wf) {
  return skeletons_impl::scan_broadcast_pd<Op>(sum_wf);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_SCAN_BROADCAST_PD_HPP
