/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_ODD_EVEN_MERGE_SORT_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_ODD_EVEN_MERGE_SORT_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/filters.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>
#include <boost/utility/result_of.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief The oddevenmergesort parametric dependency defines the dependencies
/// for each point in the global @c oddevenmergesort skeleton. At each local
/// point either a minimum or a maximum will be computed depending on where
/// this point is located in the global skeleton. This will be described
/// in the Op
///
/// @tparam Op the workfunction to be applied on each element of the
///            global @c oddevenmergesort skeleton
///
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op>
class odd_even_merge_sort_pd
  : public param_deps_defaults
{
  Op m_op;

public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = 2;

  using consumer_count_filter_type = skeletons::filters::filter<2>;
  using op_type                    = Op;

  explicit odd_even_merge_sort_pd(Op op)
    : m_op(std::move(op))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <column, level> it wraps the @c Op with the
  /// following inputs (local dependencies) and sends it to the visitor
  /// along with the @c m_op
  /// @li in<0>[column]
  /// @li in<0>[oem_column]
  /// If the particular point is just a pass-on-the-value position, it calls
  /// the identity workfunction instead.
  ///
  /// @param skeleton_size <n, log2(n)*(log2(n)+1)/2>
  ///                     where @c n is the problem size
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about Op and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  ///
  /// @see get_oem_column
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord const& skeleton_size, Coord const& coord,
               Visitor& visitor, In&& in_flow) const
  {
    using stapl::get;
    // global column index
    std::size_t column = get<0>(coord);

    // global row index
    std::size_t level = get<1>(coord) + 1;

    // submerge-step index
    std::size_t sm = get_submerge_index(level);

    // row index in a submerge
    std::size_t row = get_submerge_row_index(level, sm);

    bool is_id_point = id_point(row, column, sm);
    bool is_max_point = max_point(row, column, sm, is_id_point);
    std::size_t oem_column = get_oem_column(row,
                                            column,
                                            is_id_point,
                                            is_max_point);

    if (stapl::get<0>(skeleton_size) == 1) {
      visitor(typename stapl::identity_selector<Op>::type(),
              no_mapper(),
              get<0>(in_flow).consume_from(make_tuple(0, level - 1)));
    }
    else {
      if (is_id_point)
      {
        visitor(typename stapl::identity_selector<Op>::type(),
                no_mapper(),
                get<0>(in_flow).consume_from(make_tuple(column, level - 1)));
      }
      else
      {
        const_cast<Op&>(m_op).set_point_property(is_max_point);

        visitor(
          m_op,
          no_mapper(),
          get<0>(in_flow).consume_from(make_tuple(column, level - 1)),
          get<0>(in_flow).consume_from(make_tuple(oem_column, level - 1)));
      }
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
  std::size_t consumer_count(Size const&  skeleton_size, Coord const& coord,
                             FlowIndex) const

  {
    // global column index
    std::size_t column = stapl::get<0>(coord);

    // global row index
    std::size_t level = stapl::get<1>(coord) + 1;

    // submerge-step index
    std::size_t sm = get_submerge_index(level);

    // row index in a submerge
    std::size_t row = get_submerge_row_index(level, sm);

    std::size_t ret = 0;
    if (tuple_ops::front(skeleton_size) == 1)
      ret = 1;
    else
      if ((row>=1) && id_point(row - 1, column, sm))
        ret = 1;
      else ret = 2;

    return ret;
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Determines whether the point specified by coordinates
  /// <row, column, sm> is a pass-on-value-only point, corresponding
  /// to identity.
  ///
  /// @param row      current row index in the submerge step @c sm
  /// @param column   current column index
  /// @param sm       current submerge step
  //////////////////////////////////////////////////////////////////////
  bool id_point(std::size_t row, std::size_t column, std::size_t sm) const
  {
    // column % 2^sm
    std::size_t tmp = (column & ((1 << sm) - 1));
    return (row < sm - 1) &&
           ((tmp < (std::size_t)(1 << row)) ||
            (((std::size_t)(1 << sm) - ((std::size_t)(1 << row))) <= tmp));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determines whether the point specified by coordinates
  /// <row, column, sm> is a point where the maximum of the inputs
  /// is to be computed.
  ///
  /// @param row      current row index in the submerge step @c sm
  /// @param column   current column index
  /// @param sm       current submerge step
  //////////////////////////////////////////////////////////////////////
  bool max_point(std::size_t row, std::size_t column, std::size_t sm,
                 bool is_id_point) const
  {
    // column % 2^(row+1)
    std::size_t tmp = column & ((1 << (row + 1)) - 1);
    return ((!(is_id_point)) && ((row < sm - 1) &&
            (tmp < (std::size_t)(1 << row)))) ||
           ((row == sm - 1) && (tmp >= (std::size_t)(1 << row)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Computes the submerge index for a given level. Submerge indices
  /// span over 1..log2(n). Submerges correspond to the merging of subresults
  /// of increasing size as they unfold in a (balanced) divide-and-conquer
  /// algorithm.
  ///
  /// @param level    current level in the oddevenmerge sort global skeleton
  //////////////////////////////////////////////////////////////////////
  std::size_t get_submerge_index (std::size_t level) const
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
  /// @brief Computes local row index in a submerge
  ///
  /// @param level    current level in the oddevenmerge sort skeleton
  /// @param sm       index of the submerge
  //////////////////////////////////////////////////////////////////////
  std::size_t get_submerge_row_index (std::size_t level, std::size_t sm) const
  {
    return ((sm * (sm + 1) >> 1) - level);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Computes the second local dependency's (producer's) column index.
  /// (The first dependency's column index is @c column itself.)
  ///
  /// @param row              current row index in a submerge step
  /// @param column           current column index
  /// @param is_id_point      is the position a pass-on-value-only point
  /// @param is_max_point     is the position a maximum to be computed point
  //////////////////////////////////////////////////////////////////////
  std::size_t get_oem_column(std::size_t row, std::size_t column,
                             bool is_id_point, bool is_max_point) const
  {
    std::size_t ret = 0;
    if (is_max_point)
      ret = (column - (1 << row));
    else if (!(is_id_point))
      ret = (column + (1 << row));
    return ret;
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
/// @brief Creates an odd-even merge sort parametric dependency given a @c op.
///
/// @copybrief skeletons_impl::odd_even_merge_sort_pd
///
/// @ingroup skeletonsParamDepsAlgorithm
//////////////////////////////////////////////////////////////////////
template <typename Op>
skeletons_impl::odd_even_merge_sort_pd<Op>
odd_even_merge_sort_pd(Op const& op)
{
  return skeletons_impl::odd_even_merge_sort_pd<Op>(op);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_ODD_EVEN_MERGE_SORT_PD_HPP
