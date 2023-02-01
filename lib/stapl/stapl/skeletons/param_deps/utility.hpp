/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_UTILITY_HPP
#define STAPL_SKELETONS_PARAM_DEPS_UTILITY_HPP

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <boost/utility/result_of.hpp>
#include <stapl/skeletons/utility/filters.hpp>
#include <stapl/skeletons/utility/should_flow.hpp>
#include <stapl/skeletons/utility/position.hpp>
#include <stapl/skeletons/utility/sink_traits.hpp>
#include <stapl/utility/tuple/extract_1D.hpp>
#include <stapl/utility/tuple/pad_tuple.hpp>


namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Provides the default implementation of methods
/// required for all parametric dependencies in the framework.
///
/// @ingroup skeletonsParamDepsInternal
/////////////////////////////////////////////////////////////////////
struct param_deps_defaults
{
  template <typename Coord>
  int get_result_id(Coord const&, Coord const&) const
  {
    return -1;
  }

  template <typename Coord, typename Span>
  void configure(Coord&&, Span&&)
  { }
};

} // namespace skeletons_impl

template<typename ...Iter>
void no_op(Iter&&...)
{ }

template <int offset, bool need_config>
struct configure_filter
{
  template <typename Filters, typename Op, typename Coord,
            typename Span, typename std::size_t... Indices>
  static void apply(Filters& filters,
                    Op&& op,
                    Coord&& cur_coord,
                    Span&& span,
                    index_sequence<Indices...>&&)
  {
    no_op(std::get<Indices>(filters).configure_filter(
      (direction)(Indices - offset),
      std::forward<Span>(span),
      std::forward<Op>(op))...);
  }
};


template <int offset>
struct configure_filter<offset, false>
{
  template <typename Filters, typename Op, typename Coord,
            typename Span, typename std::size_t... Indices>
  static void apply(Filters& filters,
                    Op&& op,
                    Coord&& cur_coord,
                    Span&& span,
                    index_sequence<Indices...>&&)
  { }
};


template <int offset, bool need_config>
struct configure_mapper
{
  template <typename Mapper, typename Op, typename Coord,
            typename Span, typename std::size_t... Indices>
  static void apply(Mapper& mappers,
                    Op&& op,
                    Coord&& cur_coord,
                    Span const& span,
                    index_sequence<Indices...>&&)
  {
    constexpr static size_t padded_size = Span::nested_dims_num::value;

    auto&& cur_dims   = span.dimensions();
    auto&& total_dims = span.total_dimension();
    auto&& task_dims  = span.task_dimension();
    auto&& level_dims = span.level_dimensions();

    auto&& padded_coord =
      tuple_ops::extract_1D(tuple_ops::pad_tuple<padded_size>(cur_coord, 0));
    auto&& padded_dim =
      tuple_ops::extract_1D(tuple_ops::pad_tuple<padded_size>(cur_dims, 1));

    no_op(std::get<Indices>(mappers).configure_mapper(
      (direction)(Indices - offset),
      padded_coord, padded_dim, total_dims, task_dims, level_dims,
      op)...);
  }
};


template <int offset>
struct configure_mapper<offset, false>
{
  template <typename Mapper, typename Op, typename Coord,
            typename Span, typename std::size_t... Indices>
  static void apply(Mapper& mappers,
                    Op&& op,
                    Coord&& cur_coord,
                    Span&& span,
                    index_sequence<Indices...>&&)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Computes the output type of a given parametric dependency
/// by passing fine-grain value type of all input flows to the given
/// @c Op.
///
/// @tparam PD          the parametric dependency for which the output
///                     type is requested.
/// @tparam In          a tuple of input flows to be passed to the
///                     parametric dependency.
/// @tparam Indices     an index sequence to traverse over the @c In
///                     in the range of [0..Arity)
//////////////////////////////////////////////////////////////////////
template <typename PD, typename In,
          typename Indices = make_index_sequence<PD::op_arity>>
struct pd_output_type;

template <typename PD, typename In,
          std::size_t... Indices>
struct pd_output_type<PD, In, index_sequence<Indices...>>
{
private:
  using op_type = typename PD::op_type;

public:
  // The op arity being larger than the in_port_size indicates
  // multiple reads from the same in_port.
  // TODO(mani) currently this only happens when in_port_size == 1 and
  // op_arity > 1.
  using type =
    typename boost::result_of<
      op_type(
        flow_value_type<In,
          (PD::op_arity > PD::in_port_size ? 0 : Indices)>...)>::type;
};

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_UTILITY_HPP
