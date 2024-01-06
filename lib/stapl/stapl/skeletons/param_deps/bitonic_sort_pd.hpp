/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_BITONIC_SORT_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_BITONIC_SORT_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief The bitonicsort parametric dependency defines the dependencies
/// for each point in the global @c bitonicsort skeleton. At each local
/// point either a minimum or a maximum will be computed depending on where
/// this point is located in the global skeleton. This will be described
/// in the Op
///
/// @tparam Op the workfunction to be applied on each element of the
///            global @c bitonicsort skeleton
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op>
class bitonic_sort_pd
  : public param_deps_defaults
{
  Op m_op;

public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = 2;

  using op_type      = Op;

  explicit bitonic_sort_pd(Op op)
    : m_op(std::move(op))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <column, level> it wraps the @c Op with the
  /// following inputs and sends it to the visitor along with the @c m_op
  /// @li in<0>[column]
  /// @li in<0>[bitonic_column]
  ///
  /// @param skeleton_size <n, log2(n)*(log2(n)+1)/2>
  ///                     where @c n is the problem size
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about Op and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  ///
  /// @see bitonic_column
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord const& skeleton_size, Coord const& coord,
               Visitor& visitor, In&& in_flow) const
  {
    using stapl::get;
    std::size_t column = get<0>(coord);

    // global row idx
    std::size_t level = get<1>(coord) + 1;

    // sub-butterfly idx
    std::size_t sb = get_subbutterfly_index(level);

    // row idx in sub-butterfly
    std::size_t row = get_subbutterfly_row_index(level, sb);

    std::size_t bitonic_column = flip(row, column);

    if (get<0>(skeleton_size) == 1) {
      visitor(typename stapl::identity_selector<Op>::type(),
              no_mapper(),
              get<0>(in_flow).consume_from(make_tuple(0, level-1)));
    }
    else {
      const_cast<Op&>(m_op).set_position(row, column, sb);

      visitor(m_op,
              no_mapper(),
              get<0>(in_flow).consume_from(make_tuple(column, level-1)),
              get<0>(in_flow).consume_from(make_tuple(bitonic_column,level-1)));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of.
  ///
  /// @tparam FlowIndex the flow index to which this request is sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  std::size_t consumer_count(Size const&  skeleton_size,
                             Coord const& /*producer_coord*/,
                             FlowIndex) const
  {
    return tuple_ops::front(skeleton_size) == 1 ? 1 : 2;
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Flips bit @c row in the binary representation of @c column
  ///
  /// @param row    current row index within a sub-butterfly
  /// @param column current column index
  //////////////////////////////////////////////////////////////////////
  std::size_t flip(std::size_t row, std::size_t column) const
  {
    return ((1<<row)^column);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Computes the sub-butterfly index for a given level
  ///
  /// @param level current level in bitonic sort skeleton
  //////////////////////////////////////////////////////////////////////
  std::size_t get_subbutterfly_index(std::size_t level) const
  {
    long int temp = level;
    std::size_t i = 0;
    do {
      i++;
      temp -= i;
    } while (temp > 0);

    return i;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Computes local row index in sub-butterfly from global row idx
  ///
  /// @param level current level in bitonic sort
  /// @param sb    index of the sub-butterfly
  //////////////////////////////////////////////////////////////////////
  std::size_t get_subbutterfly_row_index (std::size_t level,
                                          std::size_t sb) const
  {
   return ((sb * (sb + 1) >> 1) - level);
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
/// @brief Creates a bitonic sort parametric dependency given a @c op.
///
/// @param op the comparison operator to be used in bitonic sort
///
/// @ingroup skeletonsParamDepsAlgorithm
//////////////////////////////////////////////////////////////////////
template <typename Op>
skeletons_impl::bitonic_sort_pd<Op>
bitonic_sort_pd(Op const& op)
{
  return skeletons_impl::bitonic_sort_pd<Op>(op);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_BITONIC_SORT_PD_HPP
