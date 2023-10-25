/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_SCAN_EXPAND_FROM_POW_TWO_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_SCAN_EXPAND_FROM_POW_TWO_PD_HPP

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
/// @brief An parametric dependency used in the postprocessing phase of
/// arbitrary size scan skeletons.
///
/// Many scan algorithms can only accept inputs with power-of-two
/// sizes. With the help of this parametric dependency and
/// @c reduce_to_pow_two all these skeletons can be extended to be used
/// for inputs with non-power-of-two sizes.
///
/// Since the postprocessing phase for inclusive and exclusive scans
/// are slightly different, the required parametric dependency is
/// selected based on the given tag (@c tags::inclusive_scan,
/// @c tags::exclusive_scan).
///
/// @tparam Op  the operation to be applied on each pair of elements
/// @tparam Tag to specify which type of expansion to be used
///
/// @see scan
/// @see tags::exclusive_scan
/// @see tags::inclusive_scan
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Tag>
class scan_expand_from_pow_two_pd;


//////////////////////////////////////////////////////////////////////
/// @brief A scan expansion parametric dependency used for the postprocessing
/// phase of inclusive scan skeletons.
///
/// @tparam Op  the operation to be applied on each pair of elements
/// @see scan
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Tag>
class scan_expand_from_pow_two_pd<Op, tags::scan<Tag, tags::inclusive> >
  : public param_deps_defaults
{
  Op m_op;
public:
  static constexpr std::size_t in_port_size = 2;
  static constexpr std::size_t op_arity     = 2;

  using op_type = Op;

  explicit scan_expand_from_pow_two_pd(Op op)
    : m_op(std::move(op))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <i, ...> it wraps the @c Op and sends it to
  /// the visitor. If q is the closest smaller power of two to n
  /// (r = n - q):
  ///
  /// @li if (i == 0)                   -> identity(in<1>[i])
  /// @li else if (i < 2*r && i%2 == 0) -> op(in<0>[i/2], in<1>[i])
  /// @li else if (i < 2*r && i%2 == 1) -> identity(in<0>[i/2])
  /// @li else if (i > 2*r)             -> identity(in<0>[i - r])
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
    const std::size_t i = tuple_ops::front(coord);
    // the first element is always identity
    std::size_t n = tuple_ops::front(skeleton_size);
    std::size_t nearest_pow_2 = 1;
    while (n != 1) {
      nearest_pow_2 <<= 1;
      n >>= 1;
    }
    typedef typename identity_selector<Op>::type identity_op_t;

    const std::size_t r = tuple_ops::front(skeleton_size) - nearest_pow_2;
    if (r != 0 && i == 0) {
      visitor(identity_op_t(),
              no_mapper(),
              stapl::get<1>(in_flow).consume_from(make_tuple(i)));
    }
    else {
      if (i < 2 * r) {
        if (i % 2 == 0) {
          visitor(m_op,
                  no_mapper(),
                  stapl::get<0>(in_flow).consume_from(make_tuple((i - 1)/2)),
                  stapl::get<1>(in_flow).consume_from(make_tuple(i)));
        }
        else {
          visitor(identity_op_t(),
                  no_mapper(),
                  stapl::get<0>(in_flow).consume_from(make_tuple(i/2)));
        }
      }
      else {
        visitor(identity_op_t(),
                no_mapper(),
                stapl::get<0>(in_flow).consume_from(make_tuple(i - r)));
      }
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
  std::size_t consumer_count(Size const&  skeleton_size,
                             Coord const& producer_coord,
                             FlowIndex flow_index) const
  {
    std::size_t n = tuple_ops::front(skeleton_size);
    std::size_t nearest_pow_2 = 1;
    while (n != 1) {
      nearest_pow_2 <<= 1;
      n >>= 1;
    }
    std::size_t i = tuple_ops::front(producer_coord);
    std::size_t r = tuple_ops::front(skeleton_size) - nearest_pow_2;

    if (flow_index.value == 1) {
      return (r != 0) && (i < 2*r) && (i % 2 == 0) ? 1 : 0;
    }
    else {
      return (i + 1 < r) ? 2 : 1;
    }
  }

  template <typename Coord>
  int get_result_id(Coord const&, Coord const&) const
  {
    return -1;
  }

  Op get_op() const
  {
    return m_op;
  }

  template <typename Coord, typename Span>
  void configure
(Coord&& cur_coord, Span&& span)
  { }

  void define_type(typer& t)
  {
    t.member(m_op);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A scan expansion parametric dependency used for the postprocessing
/// phase of exclusive scan skeletons.
///
/// @tparam Op the operation to be applied on each pair of elements
/// @see scan
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Tag>
class scan_expand_from_pow_two_pd<Op, tags::scan<Tag, tags::exclusive> >
  : public param_deps_defaults
{
  Op                  m_op;
public:
  static constexpr std::size_t in_port_size = 2;
  static constexpr std::size_t op_arity     = 2;

  using op_type = Op;

  explicit scan_expand_from_pow_two_pd(Op op)
    : m_op(std::move(op))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <i, ...> it wraps the @c Op and sends it to
  /// the visitor. If q is the closest smaller power of two to n
  /// (r = n - q):
  ///
  /// @li if (r !=0 && i == 0)          -> identity(in<0>[i])
  /// @li else if (i < 2*r && i%2 == 0) -> identity(in<0>[i/2])
  /// @li else if (i < 2*r && i%2 == 1) -> op(in<0>[i/2], in<1>[i-1])
  /// @li else if (i > 2*r)             -> identity(in<0>[i - r])
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
    const std::size_t i = tuple_ops::front(coord);
    // the first element is always identity
    std::size_t n = tuple_ops::front(skeleton_size);
    std::size_t nearest_pow_2 = 1;
    while (n != 1) {
      nearest_pow_2 <<= 1;
      n >>= 1;
    }

    typedef typename identity_selector<Op>::type identity_op_t;

    const std::size_t r = tuple_ops::front(skeleton_size) - nearest_pow_2;
    if (r != 0 && i == 0) {
      visitor(identity_op_t(),
              no_mapper(),
              stapl::get<0>(in_flow).consume_from(make_tuple(i)));
    }
    else {
      if (i < 2 * r) {
        if (i % 2 == 1) {
          visitor(m_op,
                  no_mapper(),
                  stapl::get<0>(in_flow).consume_from(make_tuple(i/2)),
                  stapl::get<1>(in_flow).consume_from(make_tuple(i - 1)));
        }
        else {
          visitor(identity_op_t(),
                  no_mapper(),
                  stapl::get<0>(in_flow).consume_from(make_tuple(i/2)));
        }
      }
      else {
        visitor(identity_op_t(),
                no_mapper(),
                stapl::get<0>(in_flow).consume_from(make_tuple(i - r)));
      }
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
  std::size_t consumer_count(Size const&  skeleton_size,
                             Coord const& producer_coord,
                             FlowIndex flow_index) const
  {
    std::size_t n = tuple_ops::front(skeleton_size);
    std::size_t nearest_pow_2 = 1;
    while (n != 1) {
      nearest_pow_2 <<= 1;
      n >>= 1;
    }
    std::size_t i = tuple_ops::front(producer_coord);
    std::size_t r = tuple_ops::front(skeleton_size) - nearest_pow_2;

    if (flow_index.value == 1) {
      return (r != 0) && (i < 2*r) && (i % 2 == 0) ? 1 : 0;
    }
    else {
      return (i < r) ? 2 : 1;
    }
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
/// @brief Creates a expand from power of two parametric dependency for
/// scan algorithms given an operation @c op.
///
/// @copybrief skeletons_impl::scan_expand_from_pow_two_pd
///
/// @param op  the operation to be applied on the inputs
/// @param tag determines the type of expansion to be used
///
/// @ingroup skeletonsParamDepsAlgorithm
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Tag>
skeletons_impl::scan_expand_from_pow_two_pd<Op, Tag>
scan_expand_from_pow_two_pd(Op const& op, Tag)
{
  return skeletons_impl::scan_expand_from_pow_two_pd<Op, Tag>(op);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_SCAN_EXPAND_FROM_POW_TWO_PD_HPP
