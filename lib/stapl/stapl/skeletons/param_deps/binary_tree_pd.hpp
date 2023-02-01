/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_BINARY_TREE_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_BINARY_TREE_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/skeletons/utility/identity_helpers.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This binary tree parametric dependency is one of the several
/// possible parametric dependencies that can be used in a @c binary tree.
/// This parametric dependency is used with @c tree in order to create
/// reduction trees. This parametric dependency can have different
/// layouts which are determined by the given @c Tag.
///
/// @tparam Op  the workfunction to be applied on each pair of elements
/// @tparam Tag determines the layout of the binary tree.
/// @see tree
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Tag>
class binary_tree_pd
  : public param_deps_defaults
{
  Op m_op;

public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = 2;

  using op_type      = Op;

  explicit binary_tree_pd(Op op)
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
  /// @brief If coord is <i, ...> it wraps the @c Op with the following
  /// inputs and sends it to the visitor along with the @c m_op
  /// @li in<0>[i*2]
  /// @li in<0>[i*2+1]
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
                     Visitor& visitor, In&& in_flow, tags::left_skewed) const
  {
    std::size_t level = stapl::get<1>(coord);
    if (stapl::get<0>(skeleton_size) == 1) {
      visitor(typename stapl::identity_selector<Op>::type(),
              no_mapper(),
              stapl::get<0>(in_flow).consume_from(make_tuple(0, level - 1)));
    }
    else {
      std::size_t i = stapl::get<0>(coord);
      visitor(
        m_op,
        no_mapper(),
        stapl::get<0>(in_flow).consume_from(make_tuple(i * 2, level - 1)),
        stapl::get<0>(in_flow).consume_from(make_tuple(i * 2 + 1, level - 1)));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief if coord is <i, h, ...> it wraps the @c Op with the
  /// following inputs and sends it to the visitor along with the
  /// @c m_op
  /// @li in<0>[i + 2<sup>h</sup>]
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
                     Visitor& visitor, In&& in_flow, tags::left_aligned) const
  {
    std::size_t level = stapl::get<1>(coord);
    if (stapl::get<0>(skeleton_size) == 1) {
      visitor(typename stapl::identity_selector<Op>::type(),
              no_mapper(),
              stapl::get<0>(in_flow).consume_from(make_tuple(0, level - 1)));
    }
    else {
      std::size_t i = stapl::get<0>(coord);
      visitor(
        m_op,
        no_mapper(),
        stapl::get<0>(in_flow).consume_from(make_tuple(i, level - 1)),
        stapl::get<0>(in_flow).consume_from(
          make_tuple(i + (std::size_t(1) << level), level - 1)));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief if coord is <i, h, ...> it wraps the @c Op with the
  /// following inputs and sends it to the visitor along with the
  /// @c m_op
  /// @li in<0>[i - 2<sup>h</sup>]
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
                     Visitor& visitor, In&& in_flow, tags::right_aligned) const
  {
    std::size_t level = stapl::get<1>(coord);
    if (stapl::get<0>(skeleton_size) == 1) {
      visitor(typename stapl::identity_selector<Op>::type(),
              no_mapper(),
              stapl::get<0>(in_flow).consume_from(make_tuple(0, level - 1)));
    }
    else {
      std::size_t i = stapl::get<0>(coord);
      visitor(
        m_op,
        no_mapper(),
        stapl::get<0>(in_flow).consume_from(
          make_tuple(i - (std::size_t(1) << level), level - 1)),
        stapl::get<0>(in_flow).consume_from(make_tuple(i, level - 1)));
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
/// @brief Creates a simple binary tree parametric dependency given
/// a @c op.
///
/// @copybrief skeletons_impl::binary_tree_pd
///
/// @ingroup skeletonsParamDepsTree
//////////////////////////////////////////////////////////////////////
template <typename Tag, typename Op>
skeletons_impl::binary_tree_pd<Op, Tag>
binary_tree_pd(Op const& op)
{
  return skeletons_impl::binary_tree_pd<Op, Tag>(op);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_BINARY_TREE_PD_HPP
