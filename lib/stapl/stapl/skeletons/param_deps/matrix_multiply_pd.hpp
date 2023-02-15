/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_MATRIX_PARAM_DEPS_MATRIX_MULTIPLY_PD_HPP
#define STAPL_SKELETONS_MATRIX_PARAM_DEPS_MATRIX_MULTIPLY_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

template <typename Op, typename Tag>
class matrix_multiply_pd;

//////////////////////////////////////////////////////////////////////
/// @brief A SUMMA parametric dependency is used in matrix multiplication.
/// It sends inputs values of matrices A and B to the given SUMMA @c Op.
///
/// @tparam Op the SUMMA workfunction to be applied on each element
/// @see SUMMA
///
/// @ingroup SkeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op>
class matrix_multiply_pd<Op, tags::summa>
  : public param_deps_defaults
{
  Op m_op;

public:
  static constexpr std::size_t in_port_size = 3;
  static constexpr std::size_t op_arity     = 3;

  using op_type = Op;

  explicit matrix_multiply_pd(Op op)
    : m_op(std::move(op))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief  Extracts input index of A, B, and resulting
  /// index of C using coord and the iteration number. Then sends it to the
  /// visitor along with the @c m_op.
  ///
  /// @tparam Coord       The skeleton index <idx, ...>.
  /// @param coord        <i, j, k, ...> where i < m, j < n, k < p, and m,
  ///                     n, and p are the dimensions of the matrices
  ///                     being multiplied.
  /// @param visitor      the information about Op and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord const& /*skeleton_size*/, Coord const& coord,
               Visitor& visitor, In&& in_flow) const
  {
    auto&& idx = tuple_ops::front(coord);

    size_t iter = stapl::get<1>(coord);

    // A index
    auto&& idx1 = make_tuple(stapl::get<0>(idx), iter);

    // B index
    auto&& idx2 = make_tuple(iter, stapl::get<1>(idx));

    // C index
    auto&& idx3 = make_tuple(stapl::get<0>(idx), stapl::get<1>(idx));

    visitor(m_op, no_mapper(),
      stapl::get<0>(in_flow).consume_from(make_tuple(idx1)),
      stapl::get<1>(in_flow).consume_from(make_tuple(idx2)),
      stapl::get<2>(in_flow).consume_from(make_tuple(idx3))
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of
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

  Op get_op(void) const
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
/// @brief Creates a matrix multiply parametric dependency depending on tag.
///
/// @ingroup SkeletonsParamDeps
//////////////////////////////////////////////////////////////////////
template <typename Tag, typename Op>
skeletons_impl::matrix_multiply_pd<Op, Tag>
matrix_multiply_pd(Op const& op)
{
  return skeletons_impl::matrix_multiply_pd<Op, Tag>(op);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_MATRIX_PARAM_DEPS_MATRIX_MULTIPLY_PD_HPP
