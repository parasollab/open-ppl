/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_SHIFTED_FIRST_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_SHIFTED_FIRST_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>
#include <stapl/utility/integer_sequence.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This parametric dependency is used in the cases similar to
/// the @c zip_pd, but whenever the reading from the first input flow
/// is shifted by 1.
///
/// An example of the inputs sent to a spawned element by this
/// skeleton:
/// @li in<0>[index - 1]
/// @li in<1>[index]
/// @li in<2>[index]
/// @li ...
///
/// @tparam Op the workfunction to be applied on each element in the
///            big @c shifted_first skeleton
/// @tparam T  the type of the default value
/// @tparam i  the number of input flows
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename T, int i>
class shifted_first_pd
  : public param_deps_defaults
{
  Op        m_op;
  T         m_neutral_value;

public:
  static constexpr std::size_t in_port_size = i;
  static constexpr std::size_t op_arity     = i;

  using op_type = Op;

  shifted_first_pd(Op op, T nvalue)
    : m_op(std::move(op)),
      m_neutral_value(std::move(nvalue))
  { }

  shifted_first_pd(shifted_first_pd const& other)
    : m_op(other.get_op()),
      m_neutral_value(other.get_init_value())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <idx, ...> it wraps the @c Op with the
  /// following inputs and sends it to the visitor along with the
  /// @c m_op
  /// @li in<0>[idx - 1]
  /// @li in<1>[idx]
  /// @li in<2>[idx]
  /// @li ...
  ///
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about Op and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord const& /*skeleton_size*/, Coord const& coord,
               Visitor&& visitor, In&& in_flow) const
  {
    std::size_t idx = tuple_ops::front(coord);
    apply_case_of(
      idx,
      std::forward<Visitor>(visitor),
      std::forward<In>(in_flow),
      make_index_sequence<i-1>());
  }

  template <typename Visitor, typename In, std::size_t... Indices>
  void apply_case_of(std::size_t id, Visitor&& visitor, In&& in_flow,
                     index_sequence<Indices...>&&) const
  {
    if (id == 0) {
      visitor(m_op,
              no_mapper(),
              constant_input(m_neutral_value),
              stapl::get<Indices+1>(in_flow).consume_from(make_tuple(id))...);
    }
    else {
      visitor(m_op,
              no_mapper(),
              stapl::get<0>(in_flow).consume_from(make_tuple(id - 1)),
              stapl::get<Indices+1>(in_flow).consume_from(make_tuple(id))...);
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
    std::size_t idx = tuple_ops::front(producer_coord);
    return
      (FlowIndex::value == 0) &&
      (idx == (tuple_ops::front(skeleton_size)-1) ||
       (tuple_ops::front(skeleton_size) == 1)) ? 0 : 1;
  }

  Op get_op() const
  {
    return m_op;
  }

  T get_init_value() const
  {
    return m_neutral_value;
  }

  void define_type(typer& t)
  {
    t.member(m_op);
    t.member(m_neutral_value);
  }
};

} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates a shifted first input parametric dependency given a
/// @c op and number of input flows specified by @c i, and the default
/// value specified by @c neutral_value.
///
/// @param op            the operation that will be used for the
///                      spawned element
/// @param neutral_value the value to be used for non-existing indices
///
/// @ingroup skeletonsParamDeps
//////////////////////////////////////////////////////////////////////
template <int i = 2,
          typename Op, typename T>
skeletons_impl::shifted_first_pd<Op, T, i>
shifted_first_pd(Op const& op, T neutral_value)
{
  return skeletons_impl::shifted_first_pd<Op, T, i>(op, neutral_value);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_SHIFTED_FIRST_PD_HPP
