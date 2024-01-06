/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_SKELETONS_PARAM_DEPS_SERIAL_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_SERIAL_PD_HPP

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
/// @brief A serial_pd parametric dependency used in the @c serial
/// skeleton.
///
/// @see serial
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, int i>
class serial_pd
  : public param_deps_defaults
{
  std::size_t const   m_number_of_sets;
  Op                  m_op;
public:
  static constexpr std::size_t in_port_size = i + 1;
  static constexpr std::size_t op_arity     = i;

  using op_type = Op;

  serial_pd(Op op, std::size_t number_of_sets)
   : m_number_of_sets(number_of_sets),
     m_op(std::move(op))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Dispatches the case_of request with the expanded indices.
  //////////////////////////////////////////////////////////////////////
  template <typename... Args>
  void case_of(Args&&... args) const
  {
    apply_case_of(std::forward<Args>(args)..., make_index_sequence<i>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specifies the dependencies for a serial set of task.
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                     potentially multi-dimensional.
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about Op and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In,
            std::size_t... Index>
  void apply_case_of(Coord&& skeleton_size, Coord&& coord,
                     Visitor& visitor, In&& in_flow,
                     index_sequence<Index...>&&) const
  {
    std::size_t cur_index = tuple_ops::front(coord);

    // if it is a simple serial case
    if (m_number_of_sets == 0)
    {
      if (cur_index == 0)
      {
        visitor(
          m_op,
          no_mapper(),
          stapl::get<Index>(in_flow).consume_from(make_tuple(cur_index))...);
      }
      else {
        visitor(
          visitor.notification_list({
            stapl::get<sizeof...(Index)>(in_flow).depend_on(
              make_tuple(cur_index-1))}),
          m_op,
          no_mapper(),
          stapl::get<Index>(in_flow).consume_from(make_tuple(cur_index))...);
      }
    }
    // else if it is a serial-set
    else {
      std::size_t size = tuple_ops::front(skeleton_size);
      std::size_t set_size = size / m_number_of_sets +
                             (size % m_number_of_sets  ? 1 : 0);

      if (cur_index % set_size == 0)
      {
        visitor(
          m_op,
          no_mapper(),
          stapl::get<Index>(in_flow).consume_from(make_tuple(cur_index))...);
      }
      else {
        visitor(
          visitor.notification_list({
            stapl::get<sizeof...(Index)>(in_flow).depend_on(
               make_tuple(cur_index-1))}),
          m_op,
          no_mapper(),
          stapl::get<Index>(in_flow).consume_from(make_tuple(cur_index))...);
      }
    }
  }

  template <typename Size, typename Coord, typename FlowIndex>
  std::size_t consumer_count(Size const&  skeleton_size,
                             Coord const& producer_coord,
                             FlowIndex) const
  {
    std::size_t num_succs = 0;
    if (FlowIndex::value < (in_port_size - 1)) {
      num_succs = 1;
    }
    else {
      std::size_t num_sets = m_number_of_sets == 0 ? 1 : m_number_of_sets;
      std::size_t size = tuple_ops::front(skeleton_size);
      std::size_t idx = stapl::get<0>(producer_coord);
      std::size_t set_size = size / num_sets +
                             (size % num_sets ? 1 : 0);
      if (m_number_of_sets == 0)  {
        num_succs = idx < (size-1) ? 1 : 0;
      }
      else {
        num_succs = ((idx+1) % set_size == 0) or
                    (idx == (size-1)) ? 0 : 1;
      }
    }
    return num_succs;
  }

  std::size_t get_number_of_sets() const
  {
    return m_number_of_sets;
  }

public:
  Op get_op() const
  {
    return m_op;
  }

  void define_type(typer& t)
  {
    t.member(m_number_of_sets);
    t.member(m_op);
  }

};

} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates a serial_pd parametric dependency.
///
/// @tparam i  number of inputs.
/// @param  op operation to be applied at each coordinate.
/// @param  number_of_sets The number of sets to form.
/// @ingroup skeletonsParamDeps
//////////////////////////////////////////////////////////////////////
template <int i, typename Op>
skeletons_impl::serial_pd<Op, i>
serial_pd(Op const& op, std::size_t number_of_sets)
{
  return skeletons_impl::serial_pd<Op, i>(op, number_of_sets);
}


} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_SERIAL_PD_HPP
