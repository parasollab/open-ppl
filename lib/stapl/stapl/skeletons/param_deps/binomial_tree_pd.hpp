/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_BINOMIAL_TREE_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_BINOMIAL_TREE_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/identity_helpers.hpp>
#include <stapl/skeletons/utility/filters.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This reduce parametric dependency is one of the several
/// possible parametric dependencies that can be used in @c reduction.
///
/// This parametric dependency is used with @c tree in order to create
/// reduction trees. Other possible choices for reduction dependence
/// skeletons are @c right_binomial_tree_pd and @c left_binomial_tree_pd.
///
/// @tparam Op  the workfunction to be applied on each pair of elements
/// @tparam Tag determines the type of the reduce parametric dependency
/// @see reduce
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Tag>
class binomial_tree_pd
  : public param_deps_defaults
{
  Op m_op;

public:
  static constexpr std::size_t in_port_size =
    (std::is_same<Tag, tags::up_phase>::value ? 1 : 3);
  static constexpr std::size_t op_arity     = 2;

  using consumer_count_filter_type = skeletons::filters::filter<2>;
  using op_type                    = Op;

  explicit binomial_tree_pd(Op op)
    : m_op(std::move(op))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Dispatches the case_of request to the appropriate
  /// implementation based on the given tag.
  //////////////////////////////////////////////////////////////////////
  template <typename... Args>
  void case_of(Args&&... args) const
  {
    apply_case_of(std::forward<Args>(args)..., Tag());
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <i, j, ...> it wraps the @c Op with the following
  /// inputs and sends it to the visitor along with the @c m_op
  /// @li in<0>[i - dist] where dist = 2^j
  /// @li in<0>[i]
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                     potentially multi-dimensional.
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about Op and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  ///
  /// @ingroup skeletonsParamDepsInternal
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void apply_case_of(Coord const& skeleton_size, Coord const& coord,
                     Visitor& visitor, In&& in_flow, tags::up_phase) const
  {
    std::size_t index = tuple_ops::front(coord);
    std::size_t level = stapl::get<1>(coord);

    std::size_t dist = std::size_t(1) << level;

    using stapl::get;
    if (index >= dist) {
      visitor(m_op,
              no_mapper(),
              get<0>(in_flow).consume_from(make_tuple(index - dist, level - 1)),
              get<0>(in_flow).consume_from(make_tuple(index, level - 1)));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief if coord is <i, j, ...> it wraps the @c Op with the
  /// following inputs and sends it to the visitor along with the
  /// @c m_op
  /// assume:
  /// @li dist  = 2 ^ (m - j - 1)
  /// @li mask1 = dist * 2 - 1
  /// @li mask2 = dist - 1
  ///
  /// if ((i & mask1) == mask1) && level == 0
  /// @li in<1>[(i, m-j-1)]
  ///
  /// if ((i & mask1) == mask1) && level  > 0
  /// @li in<0>[i]
  ///
  /// if ((i & mask2) == mask2) && index >= dist
  /// @li in<0>[i-dist]
  /// @li in<1>[i, m-j-2]
  ///
  /// if ((i & mask2) == mask2) && index < dist
  /// @li in<1>[i, m-j-2]
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                     potentially multi-dimensional.
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about Op and input is passed
  ///                     so that later this information can be
  ///                     converted to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  ///
  /// @ingroup skeletonsParamDepsInternal
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void apply_case_of(Coord const& skeleton_size, Coord const& coord,
                     Visitor& visitor, In&& in_flow, tags::down_phase) const
  {
    std::size_t index = tuple_ops::front(coord);
    std::size_t level = stapl::get<1>(coord);
    std::size_t inverted_level = stapl::get<1>(skeleton_size) - level - 1;
    // first if it is the 2^(inverted_level+1) element read from the
    // previous level
    std::size_t dist = std::size_t(1) << inverted_level;
    std::size_t mask1 = (dist << 1) - 1;
    std::size_t mask2 = dist - 1;

    using identity_op_t = typename stapl::identity_selector<Op>::type;
    using stapl::get;
    // read from the previous level only if mask1 passes
    if ((index & mask1) == mask1) {
      if (level == 0) {
        // first level of a single-level down_phase should be handled
        // differently
        if (inverted_level == 0) {
          visitor(
            identity_op_t(),
            no_mapper(),
            get<0>(in_flow).consume_from(make_tuple(index, inverted_level)));

        }
        else {
          visitor(
            identity_op_t(),
            no_mapper(),
            get<1>(in_flow).consume_from(make_tuple(index, inverted_level)));
        }
      }
      else {
        visitor(
          identity_op_t(),
          no_mapper(),
          get<0>(in_flow).consume_from(make_tuple(index, level - 1)));
      }
    }
    // read from the binomial up_phase and previous level with a distance
    else if (((index & mask2) == mask2)) {
      if (index >= dist) {
        visitor(
          m_op,
          no_mapper(),
          get<0>(in_flow).consume_from(make_tuple(index - dist, level - 1)),
          get<1>(in_flow).consume_from(make_tuple(index, inverted_level - 1)));
      }
      else {
        visitor(
          identity_op_t(),
          no_mapper(),
          get<1>(in_flow).consume_from(make_tuple(index, inverted_level - 1)));
      }
    }
  }
public:

  //////////////////////////////////////////////////////////////////////
  /// @brief determines how many of the instances of this parametric
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
    return apply_consumer_count(skeleton_size,
                                producer_coord,
                                flow_index,
                                Tag());
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief This is a consumer_count specialization for binomial tree
  /// up-phase.
  /// @see consumer_count
  ///
  /// @ingroup skeletonsParamDepsInternal
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  std::size_t apply_consumer_count(Size const&  skeleton_size,
                                   Coord const& producer_coord,
                                   FlowIndex,
                                   tags::up_phase) const
  {
    std::size_t producer_index = tuple_ops::front(producer_coord);
    std::size_t producer_level = stapl::get<1>(producer_coord);
    std::size_t num_succs = 0;
    std::size_t mask = (std::size_t(1) << (producer_level+2)) - 1;

    // first, there is definitely a successor for this task in the next level if
    // (producer_index & (2^(producer_level+2) - 1) = 2^(producer_level+2) -1
    if ((producer_index & mask) == mask) {
      num_succs = 1;
    }
    // otherwise there is one successor only if producer_idx + dist < n
    else {
      std::size_t dist = std::size_t(1) << (producer_level+1);
      if (producer_index + dist < tuple_ops::front(skeleton_size)) {
        num_succs = 1;
      }
    }
    return num_succs;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief This is a consumer_count specialization for binomial tree
  /// down-phase for requests that are received from previous levels of
  /// the down-phase.
  ///
  /// @see consumer_count
  ///
  /// @ingroup skeletonsParamDepsInternal
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord>
  std::size_t apply_consumer_count(Size const&  skeleton_size,
                                   Coord const& producer_coord,
                                   std::integral_constant<int, 0>,
                                   tags::down_phase) const
  {
    std::size_t size = tuple_ops::front(skeleton_size);
    std::size_t max_levels =
      size == 1 ? 1 : static_cast<std::size_t>(ceil(log(size) / log(2)));
    std::size_t inverted_level = max_levels - stapl::get<1>(producer_coord) - 2;

    std::size_t dist = std::size_t(1) << inverted_level;
    return tuple_ops::front(producer_coord) + dist < size ? 2 : 1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief This is a consumer_count specialization for binomial tree
  /// down-phase for requests that are received from the up-phase or
  /// the input to the binomial tree based skeleton.
  ///
  /// @see consumer_count
  ///
  /// @ingroup skeletonsParamDepsInternal
  ///////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  std::size_t apply_consumer_count(Size const&,
                                   Coord const& producer_coord,
                                   FlowIndex,
                                   tags::down_phase) const
  {
    std::size_t producer_index = tuple_ops::front(producer_coord);
    std::size_t producer_level = stapl::get<1>(producer_coord);
    std::size_t dist = std::size_t(1) << (producer_level+1);
    std::size_t mask = (dist << 1) - 1;
    return (((producer_index + dist) & mask) == mask) ? 1 : 0;
  }

public:
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
/// @brief Creates a simple reduce parametric dependency given a @c op.
///
/// @copybrief skeletons_impl::binomial_tree_pd
///
/// @ingroup skeletonsParamDepsTree
//////////////////////////////////////////////////////////////////////
template <typename Tag, typename Op>
skeletons_impl::binomial_tree_pd<Op, Tag>
binomial_tree_pd(Op const& op)
{
  return skeletons_impl::binomial_tree_pd<Op, Tag>(op);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_BINOMIAL_TREE_PD_HPP
