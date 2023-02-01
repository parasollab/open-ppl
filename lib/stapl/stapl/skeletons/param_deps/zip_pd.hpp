/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_ZIP_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_ZIP_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>
#include <stapl/utility/integer_sequence.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief A zip parametric dependency is usually used in @c zip skeleton.
/// It sends input with the same index of each flow to the given @c Op.
///
/// Example - the inputs to a spawned element created by this skeleton
/// would be:
/// @li in<0>[idx]
/// @li in<1>[idx]
/// @li in<2>[idx]
/// @li ...
///
/// @tparam Op the workfunction to be applied on each element
/// @tparam i  the number of input flows
/// @see zip
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, int i, std::size_t dims,
          typename F, typename Mappers, bool SetResult>
class zip_pd
{
  using result_mapper_t =
    typename sink_traits<
      tags::zip<stapl::use_default, i>, dims>::result_mapper_type;

  using mappers_t = typename homogeneous_tuple_type<i, Mappers>::type;
  using filters_t = homogeneous_tuple_type_t<i, F>;

  Op        m_op;
  filters_t m_filters;
  mappers_t m_mappers;
public:
  static constexpr std::size_t in_port_size = i;
  static constexpr std::size_t op_arity     = i;

  using op_type      = Op;

  zip_pd(Op op, F f, Mappers mappers)
    : m_op(std::move(op)),
      m_filters(homogeneous_tuple<i>(std::move(f))),
      m_mappers(homogeneous_tuple<i>(std::move(mappers)))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <idx, ...> it wraps the @c Op with the
  /// following inputs and sends it to the visitor along with the
  /// @c m_op
  /// @li in<0>[idx]
  /// @li in<1>[idx]
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
               Visitor& visitor, In&& in_flow) const
  {
    apply_case_of(
      tuple_ops::front(coord),
      visitor, std::forward<In>(in_flow),
      stapl::make_index_sequence<i>());
  }

private:
  template <typename Index, typename Visitor, typename In,
            std::size_t... Indices>
  void apply_case_of(Index&& index,
                     Visitor& visitor, In&& in_flow,
                     index_sequence<Indices...>&&) const
  {
    visitor.template operator()<SetResult>(
      m_op,
      std::get<0>(m_mappers).get_out_to_out_mapper(),
      stapl::get<Indices>(in_flow).consume_from(
        make_tuple(std::forward<Index>(index)),
        stapl::get<Indices>(m_filters),
        stapl::get<Indices>(m_mappers)
      )...
    );
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief determines how many of the instances of this parametric
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

private:
  template <typename Coord>
  int get_result_id_helper(std::true_type,
                           Coord const& skeleton_size,
                           Coord const& coord) const
  {
    return result_mapper_t(
             tuple_ops::front(skeleton_size))(tuple_ops::front(coord));
  }

  template <typename Coord>
  int get_result_id_helper(std::false_type,
                           Coord const&,
                           Coord const&) const
  {
    return -1;
  }

public:
  template <typename Coord>
  int get_result_id(Coord const& skeleton_size, Coord const& coord) const
  {
    return get_result_id_helper(std::integral_constant<bool, SetResult>(),
                                skeleton_size, coord);
  }

  Op const& get_op() const
  {
    return m_op;
  }

  F const& get_filter() const
  {
    return std::get<0>(m_filters);
  }

  template <typename Coord, typename Span>
  void configure(Coord&& cur_coord, Span&& span)
  {
    constexpr int offset = 1;

    configure_mapper<
      offset,
      !std::is_same<
        homogeneous_tuple_type_t<i, skeletons::no_mapper>,
        mappers_t>::value
      >::apply(m_mappers, m_op, std::forward<Coord>(cur_coord),
        std::forward<Span>(span), make_index_sequence<i>());

    configure_filter<
      offset,
      !std::is_same<
        homogeneous_tuple_type_t<i, skeletons::no_filter>,
        filters_t>::value
      >::apply(m_filters, m_op, std::forward<Coord>(cur_coord),
        std::forward<Span>(span), make_index_sequence<i>());
  }

  void define_type(typer& t)
  {
    t.member(m_op);
    t.member(m_filters);
    t.member(m_mappers);
  }
};

} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates a zip parametric dependency given a @c op. This method
/// is used whenever filtering is needed on the input edge.
///
/// @tparam SetResult whether the skeleton should set the task
///                   results on the pg edge container or not
/// @tparam F         the filter to be used when consuming from a task
/// @copybrief skeletons_impl::zip_pd
///
/// @ingroup skeletonsParamDeps
//////////////////////////////////////////////////////////////////////
template <int i            = 2,
          std::size_t dims = 1,
          bool SetResult   = false,
          typename Op,
          typename F       = skeletons::no_filter,
          typename Mappers = skeletons::no_mapper>
skeletons_impl::zip_pd<Op, i, dims, F, Mappers, SetResult>
zip_pd(Op const& op, F const& f = F(), Mappers const& mappers = Mappers())
{
  return { op, f, mappers };
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_ZIP_PD_HPP
