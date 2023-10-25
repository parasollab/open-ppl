/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_WAVEFRONT_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_WAVEFRONT_PD_HPP


#include <stapl/skeletons/param_deps/wavefront_utils.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>

namespace stapl {
namespace skeletons {
namespace wavefront_utils {


//////////////////////////////////////////////////////////////////////
/// @brief Sets the direction of the filtering function and returns
/// a copy of the filtering function.
///
/// @param filter    filtering function
/// @param direction the direction to be set
///
/// @return a copy of the original filtering function with direction
///         set as the given direction.
//////////////////////////////////////////////////////////////////////
template <typename Filter>
Filter
get_wavefront_filter(Filter f, direction dir)
{
  f.set_direction(dir);
  return f;
}

//////////////////////////////////////////////////////////////////////
/// @brief The special case that no filtering function is defined.
//////////////////////////////////////////////////////////////////////
template<>
inline skeletons::no_filter
get_wavefront_filter(skeletons::no_filter f, direction)
{
  return f;
}


template <bool SetResult = false,
          typename Visitor,
          typename In,
          typename Op,
          typename PGPortMapper,
          typename Coord, std::size_t... Indices, typename... Consumers>
void wavefront_visitor(
       Visitor& visitor, In&& in_flow, Op&& op, PGPortMapper&& pg_port_mapper,
       Coord&& coord, index_sequence<Indices...>&&,
       Consumers&&... consumers)
{
  visitor. template operator()<SetResult>(
    std::forward<Op>(op), std::forward<PGPortMapper>(pg_port_mapper),
    stapl::get<Indices>(in_flow).consume_from(coord)...,
    std::forward<Consumers>(consumers)...
  );
}

} // namespace wavefront_utils

namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief A wavefront parametric dependency represents the common
/// computation pattern used in many applications including dynamic
/// programming problems in which computation starts from one corner
/// of an n-dimensional input and finishes in the farthest corner.
///
/// @tparam num_inputs    number of inputs not coming from the wavefront
///                       data flows.
/// @tparam Op            operation to be applied at each point.
/// @tparam Corners       an array containing the parametric starting
///                       point of this wavefront skeleton
/// @tparam Filter        filtering function to be applied on the producer
///                       side.
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <std::size_t num_inputs, std::size_t dims,
          typename Op, typename Filter, typename Mappers, bool SetResult>
class wavefront_pd;


//////////////////////////////////////////////////////////////////////
/// @brief The specialization for the 2D wavefront.
///
/// @tparam num_inputs    number of inputs not coming from the wavefront
///                       data flows.
/// @tparam Op            operation to be applied at each point.
/// @tparam Filter        filtering function to be applied on the producer
///                       side.
///
/// @note The boundaries are ordered based on the dependence direction:
/// boundary0 is the boundary across the first dimension,
/// boundary1 is the boundary across the second dimension.
///
/// For example, for a wavefront starting from the top-left corner
/// the boundaries are:
/// <code>
///              boundary0
///             ___________
///            |           |
/// boundary1  |           |
///            |___________|
/// </code>
///
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <std::size_t num_inputs,
          typename Op, typename Filter, typename Mappers, bool SetResult>
class wavefront_pd<num_inputs, 2, Op, Filter, Mappers, SetResult>
{
  /// ports [0..num_inputs) provide the input ports
  using input_port_indices   = make_index_sequence<num_inputs>;
  /// the first two ports after the inputs provide the boundary conditions
  using boundary0_port_index = std::integral_constant<int, num_inputs + 0>;
  using boundary1_port_index = std::integral_constant<int, num_inputs + 1>;
  /// the last port provides the loop back to this wavefront
  using self_port_index      = std::integral_constant<int, num_inputs + 2>;

  using corners_type         = std::array<skeletons::position, 2>;

  using skeleton_tag_t       = tags::wavefront<2>;

  using result_mapper_t      =
    typename sink_traits<skeleton_tag_t, 2>::result_mapper_type;

  using mappers_t = homogeneous_tuple_type_t<2, Mappers>;
  using filters_t = homogeneous_tuple_type_t<2, Filter>;
  Op                   m_op;
  corners_type const   m_start_corner;
  filters_t            m_filters;
  mappers_t            m_mappers;
public:
  using op_type = Op;

  //////////////////////////////////////////////////////////////////////
  /// @brief The number of input ports are the inputs + 2 boundary views + 1
  /// loop back from this wavefront
  //////////////////////////////////////////////////////////////////////
  static constexpr std::size_t in_port_size = num_inputs + 3;
  static constexpr std::size_t op_arity     = in_port_size - 1;

  wavefront_pd(Op op, corners_type start_corner,
               Filter filter, Mappers mappers)
    : m_op(std::move(op)),
      m_start_corner(std::move(start_corner)),
      m_filters(homogeneous_tuple<2>(std::move(filter))),
      m_mappers(homogeneous_tuple<2>(std::move(mappers)))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <(i0, i1, ..., ik), ...> it wraps the @c Op
  /// with the following inputs and sends it to the visitor along with
  /// the
  /// @c m_op
  /// @li in<0>[(  i0,   i1, ...,   ik)]
  /// @li in<1>[(i0-1,   i1, ...,   ik)]
  /// @li in<1>[   i0, i1-1, ...,   ik)]
  /// ...
  /// @li in<1>[(  i0,   i1, ..., ik-1)]
  ///
  /// The boundary cases are when any of the coordinates is 0 and should
  /// be handled separately.
  ///
  /// @param coord    <(i0,...,ik), ...> where (i0,...,ik) is an
  ///                 n-dimensional coordinate.
  /// @param visitor  the information about Op and input is passed
  ///                 so that later this information can be converted
  ///                 to a node in the dependence graph.
  /// @param in_flow  a tuple of input flow to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord const& skeleton_size, Coord const& coord,
               Visitor& visitor, In&& in_flow) const
  {
    // Find the (i0, ..., ik)
    std::size_t i0, i1;
    auto&& idxv      = tuple_ops::front(coord);
    auto&& cur_dims  = tuple_ops::front(skeleton_size);
    std::tie(i0, i1) = idxv;

    auto&& idx = make_tuple(idxv);

    using namespace skeletons::wavefront_utils;

    std::size_t start0, start1;
    std::tie(start0, start1) = first_index(m_start_corner,
                                           cur_dims);

    int dir0, dir1;
    std::tie(dir0, dir1) = wavefront_direction(m_start_corner);

    auto&& dep0 = make_tuple(make_tuple(i0+dir0,      i1), -1);
    auto&& dep1 = make_tuple(make_tuple(     i0, i1+dir1), -1);

    using wp = skeletons::position;
    std::array<wp, 2> cur_pos = {{ (i0 == start0 ? wp::first : wp::middle),
                                   (i1 == start1 ? wp::first : wp::middle) }};

    auto&& wf_mappers0 = std::get<0>(m_mappers);
    auto&& wf_mappers1 = std::get<1>(m_mappers);

    auto&& f0 = std::get<0>(m_filters);
    auto&& f1 = std::get<1>(m_filters);

    auto out_to_out_mapper = std::get<0>(m_mappers).get_out_to_out_mapper();

    using stapl::get;
    auto&& boundary0_flow = get<boundary0_port_index::value>(in_flow);
    auto&& boundary1_flow = get<boundary1_port_index::value>(in_flow);
    auto&&      self_flow = get<     self_port_index::value>(in_flow);

    switch(condition_value(cur_pos[0], cur_pos[1]))
    {
      case condition_value(wp::first, wp::first):
        wavefront_utils::wavefront_visitor<SetResult>(
          visitor, std::forward<In>(in_flow),
          m_op, out_to_out_mapper, idx, input_port_indices(),
          boundary0_flow.consume_from(idx, f0, wf_mappers0),
          boundary1_flow.consume_from(idx, f1, wf_mappers1));
        break;

      case condition_value(wp::first, wp::middle):
        wavefront_utils::wavefront_visitor<SetResult>(
          visitor, std::forward<In>(in_flow),
          m_op, out_to_out_mapper, idx, input_port_indices(),
          boundary0_flow.consume_from(idx,  f0, wf_mappers0),
          self_flow.consume_from(     dep1, f1, wf_mappers1));
        break;

      case condition_value(wp::middle, wp::first):
        wavefront_utils::wavefront_visitor<SetResult>(
          visitor, std::forward<In>(in_flow),
          m_op, out_to_out_mapper, idx, input_port_indices(),
          self_flow.consume_from(    dep0, f0, wf_mappers0),
          boundary1_flow.consume_from(idx, f1, wf_mappers1));
        break;

      case condition_value(wp::middle, wp::middle):
      default:
        wavefront_utils::wavefront_visitor<SetResult>(
          visitor, std::forward<In>(in_flow),
          m_op, out_to_out_mapper, idx, input_port_indices(),
          self_flow.consume_from(dep0, f0, wf_mappers0),
          self_flow.consume_from(dep1, f1, wf_mappers1));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of.
  ///
  /// @param skeleton_size   the size of this skeleton
  /// @param producer_coord  the coordinate of the producer element
  ///                        which is providing data to this parametric
  ///                        dependency
  /// @tparam FlowIndex      the flow index on which this request is
  ///                        sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord>
  std::size_t consumer_count(Size const&  skeleton_size,
                             Coord const& producer_coord,
                             self_port_index) const
  {
    // Get the dimensions
    auto&& coord = tuple_ops::front(producer_coord);
    auto&& end_offset = wavefront_utils::last_index(
                          m_start_corner, tuple_ops::front(skeleton_size));

    std::size_t succs = 0;
    succs += (stapl::get<0>(coord) == stapl::get<0>(end_offset) ? 0 : 1);
    succs += (stapl::get<1>(coord) == stapl::get<1>(end_offset) ? 0 : 1);

    return succs;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copybrief consumer_count
  ///
  /// This specialization is used when the request is only sent to the
  /// first input flow of the current skeleton.
  ///
  /// @param producer_coord  the coordinate of the producer element
  ///                        which is providing data to this parametric
  ///                        dependency
  /// @tparam FlowIndex      the flow index on which this request is
  ///                        sent
  ///
  /// @todo This method needs to be specialized for the boundary cases
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, int i>
  typename std::enable_if<(i < num_inputs), std::size_t>::type
  consumer_count(Size const&  /*skeleton_size*/,
                 Coord const& producer_coord,
                 std::integral_constant<int, i>) const
  {
    return 1;
  }

  template <typename Size, typename Coord, int i>
  typename std::enable_if<
    (i >= num_inputs) and (i < self_port_index::value),
    std::size_t
  >::type
  consumer_count(Size const&  skeleton_size,
                 Coord const& producer_coord,
                 std::integral_constant<int, i>) const
  {
    auto&& coord = tuple_ops::front(producer_coord);
    auto&& start_offset = wavefront_utils::first_index(
                            m_start_corner, tuple_ops::front(skeleton_size));
    return (stapl::get<i - num_inputs>(coord) ==
            stapl::get<i - num_inputs>(start_offset) ? 1 : 0);
  }

private:
  template <typename Coord>
  int get_result_id_helper(std::true_type,
                           Coord const& skeleton_size,
                           Coord const& coord) const
  {
    return result_mapper_t(m_start_corner, tuple_ops::front(skeleton_size))
             (tuple_ops::front(coord));
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

  corners_type const& get_start_corner() const
  {
    return m_start_corner;
  }

  Filter const& get_filter() const
  {
    return std::get<0>(m_filters);
  }

  template <typename Coord, typename Span>
  void configure(Coord&& cur_coord, Span&& span)
  {
    configure_mapper<
      0,
      !std::is_same<
        homogeneous_tuple_type_t<2, skeletons::no_mapper>,
        mappers_t>::value
      >::apply(m_mappers, m_op, std::forward<Coord>(cur_coord),
        std::forward<Span>(span), make_index_sequence<2>());

    configure_filter<
      0,
      !std::is_same<
        homogeneous_tuple_type_t<2, skeletons::no_filter>,
        filters_t>::value
      >::apply(m_filters, m_op, std::forward<Coord>(cur_coord),
        std::forward<Span>(span), make_index_sequence<2>());
  }

  void define_type(typer& t)
  {
    t.member(m_op);
    t.member(m_start_corner);
    t.member(m_filters);
    t.member(m_mappers);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief The specialization for the 3D wavefront.
///
/// @tparam num_inputs    number of inputs not coming from the wavefront
///                       data flows.
/// @tparam Op            operation to be applied at each point.
/// @tparam Filter        filtering function to be applied on the producer
///                       side.
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <std::size_t num_inputs,
          typename Op, typename Filter, typename Mappers,bool SetResult>
class wavefront_pd<num_inputs, 3, Op, Filter, Mappers, SetResult>
{
  /// ports [0..num_inputs) provide the input ports
  using input_port_indices   = make_index_sequence<num_inputs>;
  /// the first three ports after the inputs provide the boundary conditions
  using boundary0_port_index = std::integral_constant<int, num_inputs + 0>;
  using boundary1_port_index = std::integral_constant<int, num_inputs + 1>;
  using boundary2_port_index = std::integral_constant<int, num_inputs + 2>;
  /// the last port provides the loop back to this wavefront
  using self_port_index      = std::integral_constant<int, num_inputs + 3>;

  using corners_type         = std::array<skeletons::position, 3>;

  using result_mapper_t      =
    typename sink_traits<tags::wavefront<3>, 3>::result_mapper_type;

  using mappers_t = homogeneous_tuple_type_t<3, Mappers>;
  using filters_t = homogeneous_tuple_type_t<3, Filter>;

  Op                   m_op;
  corners_type const   m_start_corner;
  filters_t            m_filters;
  mappers_t            m_mappers;
public:
  using op_type = Op;

  //////////////////////////////////////////////////////////////////////
  /// @brief The number of input ports are the inputs + 3 boundary views
  /// + 1 loop back from this wavefront
  //////////////////////////////////////////////////////////////////////
  static constexpr std::size_t in_port_size = num_inputs + 4;
  static constexpr std::size_t op_arity     = in_port_size - 1;

  wavefront_pd(Op op, corners_type start_corner,
               Filter filter, Mappers mappers)
    : m_op(std::move(op)),
      m_start_corner(std::move(start_corner)),
      m_filters(homogeneous_tuple<3>(std::move(filter))),
      m_mappers(homogeneous_tuple<3>(std::move(mappers)))
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <(i0, i1, ..., ik), ...> it wraps the @c Op
  /// with the following inputs and sends it to the visitor along with
  /// the
  /// @c m_op
  /// @li in<0>[(  i0,   i1, ...,   ik)]
  /// @li in<1>[(i0-1,   i1, ...,   ik)]
  /// @li in<1>[   i0, i1-1, ...,   ik)]
  /// ...
  /// @li in<1>[(  i0,   i1, ..., ik-1)]
  ///
  /// The boundary cases are when any of the coordinates is 0 and should
  /// be handled separately.
  ///
  /// @param coord    <(i0,...,ik), ...> where (i0,...,ik) is an
  ///                 n-dimensional coordinate.
  /// @param visitor  the information about Op and input is passed
  ///                 so that later this information can be converted
  ///                 to a node in the dependence graph.
  /// @param in_flow  a tuple of input flow to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord const& skeleton_size, Coord const& coord,
               Visitor& visitor, In&& in_flow) const
  {
    // Find the (i0, ..., ik)
    std::size_t i0, i1, i2;
    auto&& idxv  = tuple_ops::front(coord);
    std::tie(i0, i1, i2) = idxv;
    auto&& idx  = make_tuple(idxv);

    using namespace skeletons::wavefront_utils;

    std::size_t start0, start1, start2;
    std::tie(start0, start1, start2) = first_index(
                                         m_start_corner,
                                         tuple_ops::front(skeleton_size));

    int dir0, dir1, dir2;
    std::tie(dir0, dir1, dir2) = wavefront_direction(m_start_corner);

    auto&& dep0 = make_tuple(make_tuple(i0+dir0,      i1,      i2), -1);
    auto&& dep1 = make_tuple(make_tuple(     i0, i1+dir1,      i2), -1);
    auto&& dep2 = make_tuple(make_tuple(     i0,      i1, i2+dir2), -1);

    using wp = skeletons::position;

    std::array<wp, 3> cur_pos = {{ (i0 == start0 ? wp::first : wp::middle),
                                   (i1 == start1 ? wp::first : wp::middle),
                                   (i2 == start2 ? wp::first : wp::middle) }};

    auto&& wf_mappers0 = std::get<0>(m_mappers);
    auto&& wf_mappers1 = std::get<1>(m_mappers);
    auto&& wf_mappers2 = std::get<2>(m_mappers);

    auto&& f0 = std::get<0>(m_filters);
    auto&& f1 = std::get<1>(m_filters);
    auto&& f2 = std::get<2>(m_filters);

    auto out_to_out_mapper = std::get<0>(m_mappers).get_out_to_out_mapper();

    auto&& boundary0_flow = get<boundary0_port_index::value>(in_flow);
    auto&& boundary1_flow = get<boundary1_port_index::value>(in_flow);
    auto&& boundary2_flow = get<boundary2_port_index::value>(in_flow);
    auto&& self_flow      = get<     self_port_index::value>(in_flow);

    switch(condition_value(cur_pos[0], cur_pos[1], cur_pos[2]))
    {
      case condition_value(wp::first, wp::first, wp::first):
        wavefront_utils::wavefront_visitor<SetResult>(
          visitor, std::forward<In>(in_flow),
          m_op, out_to_out_mapper, idx, input_port_indices(),
          boundary0_flow.consume_from(idx, f0, wf_mappers0),
          boundary1_flow.consume_from(idx, f1, wf_mappers1),
          boundary2_flow.consume_from(idx, f2, wf_mappers2));
        break;

      case condition_value(wp::first, wp::first, wp::middle):
        wavefront_utils::wavefront_visitor<SetResult>(
          visitor, std::forward<In>(in_flow),
          m_op, out_to_out_mapper, idx, input_port_indices(),
          boundary0_flow.consume_from(idx,  f0, wf_mappers0),
          boundary1_flow.consume_from(idx,  f1, wf_mappers1),
          self_flow.consume_from(     dep2, f2, wf_mappers2));
        break;

      case condition_value(wp::first, wp::middle, wp::first):
        wavefront_utils::wavefront_visitor<SetResult>(
          visitor, std::forward<In>(in_flow),
          m_op, out_to_out_mapper, idx, input_port_indices(),
          boundary0_flow.consume_from(idx, f0, wf_mappers0),
              self_flow.consume_from(dep1, f1, wf_mappers1),
          boundary2_flow.consume_from(idx, f2, wf_mappers2));
        break;

      case condition_value(wp::first, wp::middle, wp::middle):
        wavefront_utils::wavefront_visitor<SetResult>(
          visitor, std::forward<In>(in_flow),
          m_op, out_to_out_mapper, idx, input_port_indices(),
          boundary0_flow.consume_from(idx,  f0, wf_mappers0),
               self_flow.consume_from(dep1, f1, wf_mappers1),
               self_flow.consume_from(dep2, f2, wf_mappers2));
        break;

      case condition_value(wp::middle, wp::first, wp::first):
        wavefront_utils::wavefront_visitor<SetResult>(
          visitor, std::forward<In>(in_flow),
          m_op, out_to_out_mapper, idx, input_port_indices(),
               self_flow.consume_from(dep0, f0, wf_mappers0),
          boundary1_flow.consume_from(idx,  f1, wf_mappers1),
          boundary2_flow.consume_from(idx,  f2, wf_mappers2));
        break;

      case condition_value(wp::middle, wp::first, wp::middle):
        wavefront_utils::wavefront_visitor<SetResult>(
          visitor, std::forward<In>(in_flow),
          m_op, out_to_out_mapper, idx, input_port_indices(),
               self_flow.consume_from(dep0, f0, wf_mappers0),
          boundary1_flow.consume_from(idx,  f1, wf_mappers1),
               self_flow.consume_from(dep2, f2, wf_mappers2));
        break;

      case condition_value(wp::middle, wp::middle, wp::first):
        wavefront_utils::wavefront_visitor<SetResult>(
          visitor, std::forward<In>(in_flow),
          m_op, out_to_out_mapper, idx, input_port_indices(),
               self_flow.consume_from(dep0, f0, wf_mappers0),
               self_flow.consume_from(dep1, f1, wf_mappers1),
          boundary2_flow.consume_from(idx,  f2, wf_mappers2));
        break;

      case condition_value(wp::middle, wp::middle, wp::middle):
      default:
        wavefront_utils::wavefront_visitor<SetResult>(
          visitor, std::forward<In>(in_flow),
          m_op, out_to_out_mapper, idx, input_port_indices(),
              self_flow.consume_from(dep0, f0, wf_mappers0),
              self_flow.consume_from(dep1, f1, wf_mappers1),
              self_flow.consume_from(dep2, f2, wf_mappers2));
    }
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of.
  ///
  /// @param skeleton_size   the size of this skeleton
  /// @param producer_coord  the coordinate of the producer element
  ///                        which is providing data to this parametric
  ///                        dependency
  /// @tparam FlowIndex      the flow index on which this request is
  ///                        sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord>
  std::size_t consumer_count(Size const&  skeleton_size,
                             Coord const& producer_coord,
                             self_port_index) const
  {
    // Get the dimensions
    auto&& coord = tuple_ops::front(producer_coord);
    auto&& end_offset = wavefront_utils::last_index(
                          m_start_corner, tuple_ops::front(skeleton_size));

    std::size_t succs = 0;
    succs += (stapl::get<0>(coord) == stapl::get<0>(end_offset) ? 0 : 1);
    succs += (stapl::get<1>(coord) == stapl::get<1>(end_offset) ? 0 : 1);
    succs += (stapl::get<2>(coord) == stapl::get<2>(end_offset) ? 0 : 1);

    return succs;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copybrief consumer_count
  ///
  /// This specialization is used when the request is only sent to the
  /// first input flow of the current skeleton.
  ///
  /// @param producer_coord  the coordinate of the producer element
  ///                        which is providing data to this parametric
  ///                        dependency
  /// @tparam FlowIndex      the flow index on which this request is
  ///                        sent
  ///
  /// @todo This method needs to be specialized for the boundary cases
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, int i>
  typename std::enable_if<(i < num_inputs), std::size_t>::type
  consumer_count(Size const&  /*skeleton_size*/,
                 Coord const& producer_coord,
                 std::integral_constant<int, i>) const
  {
    return 1;
  }

  template <typename Size, typename Coord, int i>
  typename std::enable_if<
    (i >= num_inputs) and (i < self_port_index::value),
    std::size_t
  >::type
  consumer_count(Size const&  skeleton_size,
                 Coord const& producer_coord,
                 std::integral_constant<int, i>) const
  {
    auto&& coord = tuple_ops::front(producer_coord);
    auto&& start_offset = wavefront_utils::first_index(
                            m_start_corner, tuple_ops::front(skeleton_size));
    return (stapl::get<i - num_inputs>(coord) ==
            stapl::get<i - num_inputs>(start_offset) ? 1 : 0);
  }

private:
  template <typename Coord>
  int get_result_id_helper(std::true_type,
                       Coord const& skeleton_size,
                       Coord const& coord) const
  {
    return result_mapper_t(m_start_corner, tuple_ops::front(skeleton_size))
             (tuple_ops::front(coord));
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

  corners_type const& get_start_corner() const
  {
    return m_start_corner;
  }

  Filter const& get_filter() const
  {
    return std::get<0>(m_filters);
  }

  template <typename Coord, typename Span>
  void configure(Coord&& cur_coord, Span&& span)
  {
    configure_mapper<
      0,
      !std::is_same<
        homogeneous_tuple_type_t<3, skeletons::no_mapper>,
        mappers_t>::value
      >::apply(m_mappers, m_op, std::forward<Coord>(cur_coord),
        std::forward<Span>(span), make_index_sequence<3>());

    configure_filter<
      0, !std::is_same<
           homogeneous_tuple_type_t<3, skeletons::no_filter>,
           filters_t>::value
      >::apply(m_filters, m_op, std::forward<Coord>(cur_coord),
        std::forward<Span>(span), make_index_sequence<3>());
  }

  void define_type(typer& t)
  {
    t.member(m_op);
    t.member(m_start_corner);
    t.member(m_filters);
    t.member(m_mappers);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief The specialization for an N-D wavefront.
///
/// @tparam num_inputs    number of inputs not coming from the wavefront
///                       data flows.
/// @tparam Op            operation to be applied at each point.
/// @tparam N             number of dimensions of the wavefront.
/// @tparam Filter        filtering function to be applied on the producer
///                       side.
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <std::size_t num_inputs,
          std::size_t N,
          typename Op, typename Filter, typename Mappers, bool SetResult>
class wavefront_pd
{
  //compute an index_sequence for expanding over all dimensions
  using dimensions = make_index_sequence<N>;

  /// ports [0..num_inputs) provide the input_ports
  using input_port_indices  = make_index_sequence<num_inputs>;
  /// the first N ports after the inputs provide the boundary conditions
  template<int M>
  using boundary_port_index = std::integral_constant<int, num_inputs + M>;
  /// the last port provides the loop back to this wavefront
  using self_port_index     = std::integral_constant<int, num_inputs + N>;

  using corners_type         = std::array<skeletons::position, N>;

  using result_mapper_t      =
    typename sink_traits<tags::wavefront<N>, N>::result_mapper_type;

  using mappers_t = homogeneous_tuple_type_t<N, Mappers>;
  using filters_t = homogeneous_tuple_type_t<N, Filter>;

  Op                  m_op;
  corners_type const  m_start_corner;
  filters_t           m_filters;
  mappers_t           m_mappers;

public:
  using op_type = Op;
  //////////////////////////////////////////////////////////////////////
  /// @brief The number of input ports are the inputs + N boundary views
  /// +1 loop back from this wavefront
  //////////////////////////////////////////////////////////////////////
  static constexpr std::size_t in_port_size = num_inputs + N + 1;
  static constexpr std::size_t op_arity     = in_port_size - 1;

  wavefront_pd(Op op, corners_type start_corner, Filter filter, Mappers mappers)
    : m_op(std::move(op)),
      m_start_corner(std::move(start_corner)),
      m_filters(homogeneous_tuple<N>(std::move(filter))),
      m_mappers(homogeneous_tuple<N>(std::move(mappers)))
  { }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the dependence for the ith dimension at a given index.
  ///
  /// A wavefront reads the current index + an offset with a component
  /// in the ith dimension.
  ///
  /// e.g. if I == X, read from (x + d.x, y,       z...)
  ///      if I == Y, read from (x,       y + d.y, z...)
  ///
  /// @param idx  current index, i.e. (x,y,z...)
  /// @param d    tuple of propagation directions
  ///
  /// @return     ith dependence assuming the index is not on the
  ///             ith dimension's boundary.
  //////////////////////////////////////////////////////////////////////
  template<int I, typename Index, typename Direction, std::size_t... Is>
  auto get_dep_elem(Index const& idx, Direction const& d,
                    index_sequence<Is...>) const
  STAPL_AUTO_RETURN((
    make_tuple((get<Is>(idx) + (Is == I ? get<Is>(d) : 0))...)
  ))

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the tuple of @c skeleton::position objects for this
  /// index.
  ///
  /// @param s tuple of starting indices for each dimension
  /// @param i current index
  /// @return  tuple where the ith element is @c position::first if
  ///          at the starting index in the ith dimension, or position::middle
  ///          otherwise.
  //////////////////////////////////////////////////////////////////////
  template<typename Starts, typename Indices, std::size_t... Is>
  auto get_pos(Starts const& s, Indices const& i, index_sequence<Is...>) const
  STAPL_AUTO_RETURN((
    make_tuple((get<Is>(s) == get<Is>(i) ? skeletons::position::first
                                         : skeletons::position::middle)...)
  ))

  //////////////////////////////////////////////////////////////////////
  /// @brief Call the visitor with an index and all of its dependencies.
  ///
  /// This is the base case of visit that just forwards the accumulated
  /// input wrappers to wavefront_visitor.
  ///
  /// @param visitor visitor object
  /// @param in_flow a tuple of input flow to consume from
  /// @param idx     current index of the wavefront
  /// @param p       tuple of @c skeleton::position objects
  /// @param d       tuple of dependencies for each dimension
  /// @param f       tuple of filters for each dimension
  /// @param args    pack of all the input wrappers for each dimension
  ///
  /// @see wavefront_utils::wavefront_visitor
  //////////////////////////////////////////////////////////////////////
  template <int I,
            typename Visitor, typename In,
            typename Index, typename Pos,
            typename Deps, typename Filters, typename... Args>
  void visit(Visitor& visitor, In&& in_flow,
             Index const& idx, Pos const& p, Deps const& d,
             Filters const& f, std::true_type, Args&&... args) const
  {
    wavefront_utils::wavefront_visitor<SetResult>(
      visitor, std::forward<In>(in_flow),
      m_op,  m_mappers, idx, input_port_indices(),
      std::forward<Args>(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the ith input wrapper to pass to the visitor.
  ///
  /// This is the recursive step of visit which computes the input wrapper
  /// for the ith dimension (where the ith dimension should read from,
  /// depending on boundary conditions). It then forwards it to the next
  /// dimension with all previously computed wrappers (if any) or to the
  /// base case if there are no more.
  ///
  /// @param visitor visitor object
  /// @param in_flow a tuple of input flow to consume from
  /// @param idx     current index of the wavefront
  /// @param p       tuple of @c skeleton::position objects
  /// @param d       tuple of dependencies for each dimension
  /// @param f       tuple of filters for each dimension
  /// @param args    pack of of the input wrappers computed so far for
  ///                the previous dimensions, if any
  ///
  /// @see wavefront_utils::wavefront_visitor
  //////////////////////////////////////////////////////////////////////
  template <int I, typename Visitor, typename In, typename Index, typename Pos,
            typename Deps, typename Filters, typename... Args>
  void visit(Visitor& visitor, In&& in_flow,
             Index const& idx, Pos const& p, Deps const& d,
             Filters const& f, std::false_type, Args&&... args) const
  {
    using is_last = std::integral_constant<bool, I == N - 1>;
    //dispatch based on if we're on the first boundary for this dimension
    if (get<I>(p) == skeletons::position::first)
      visit<I+1>(visitor, std::forward<In>(in_flow),
                 idx, p, d, f, is_last(), std::forward<Args>(args)...,
                 stapl::get<boundary_port_index<I>::value>(
                   in_flow).consume_from(idx));
    else
      visit<I+1>(visitor, std::forward<In>(in_flow),
                 idx, p, d, f, is_last(), std::forward<Args>(args)...,
                 stapl::get<self_port_index::value>(in_flow).consume_from(
                   get<I>(d), get<I>(f)));
   }

  //////////////////////////////////////////////////////////////////////
  /// @brief Calls the visitor with the correct dependencies based on
  /// the current coordinate.
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                     potentially multi-dimensional.
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about WF and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In, std::size_t... Is>
  void apply_case_of(Coord const& skeleton_size, Coord const& coord,
                     Visitor& visitor, In&& in_flow,
                     index_sequence<Is...>) const
  {
    using namespace skeletons::wavefront_utils;

    auto&& idxv = tuple_ops::front(coord);
    auto&& idx  = make_tuple(idxv);

    auto starts = first_index(m_start_corner,tuple_ops::front(skeleton_size));

    auto&& dirs = wavefront_direction(m_start_corner);

    auto&& fs   = make_tuple(get_wavefront_filter(get<Is>(m_filters),
      static_cast<direction>(Is))...);

    auto&& deps = make_tuple(
       //create the ith dimension's dependencies
      make_tuple(
        //compute its elements
        get_dep_elem<Is>(idxv, dirs, dimensions()), -1
      )...
    );

    auto&& cur_pos = get_pos(starts, idxv, dimensions());

    static_assert(N != 0, "0-d wavefront");
    visit<0>(visitor, std::forward<In>(in_flow),
             idx, cur_pos, deps, fs, std::false_type());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the number of consumers for a given coordinate.
  ///
  /// This simply checks how many of the coordinates dimensions are not
  /// on the end boundary, i.e the number of times it will be read from.
  ///
  /// @param skeleton_size  size of the skeleton
  /// @param producer_coord coordinate to count consumers for
  /// @return               number of consumers
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, std::size_t... Is>
  std::size_t apply_consumer_count(Size const&  skeleton_size,
                                   Coord const& producer_coord,
                                   index_sequence<Is...>) const
  {
    // Get the dimensions
    auto&& coord = tuple_ops::front(producer_coord);
    auto&& end_offset = wavefront_utils::last_index(
                          m_start_corner, tuple_ops::front(skeleton_size));

    //see how many dimensions are on the boundary
    auto&& on_boundary =
      {(stapl::get<Is>(coord) == stapl::get<Is>(end_offset))...};
    //we have a successor for each dimension if not on boundary
    return std::count(on_boundary.begin(), on_boundary.end(), false);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Calls the visitor with the correct dependencies based on
  /// the current coordinate.
  ///
  /// @param skeleton_size  size of the skeleton
  /// @param coord          current coordinate in the skeleton
  /// @param visitor        visitor object to apply to each element in
  /// the wavefront.
  //////////////////////////////////////////////////////////////////////
  template <typename... Args>
  void case_of(Args&&... args) const
  {
    return apply_case_of(std::forward<Args>(args)..., dimensions());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of.
  ///
  /// @param skeleton_size   the size of this skeleton
  /// @param producer_coord  the coordinate of the producer element
  ///                        which is providing data to this parametric
  ///                        dependency
  /// @tparam FlowIndex      the flow index on which this request is
  ///                        sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord>
  std::size_t consumer_count(Size const&  skeleton_size,
                             Coord const& producer_coord,
                             self_port_index) const
  {
    return apply_consumer_count(skeleton_size, producer_coord, dimensions());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copybrief consumer_count
  ///
  /// This specialization is used when the request is only sent to the
  /// first input flow of the current skeleton.
  ///
  /// @param producer_coord  the coordinate of the producer element
  ///                        which is providing data to this parametric
  ///                        dependency
  /// @tparam FlowIndex      the flow index on which this request is
  ///                        sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord>
  std::size_t consumer_count(Size const&  /*skeleton_size*/,
                             Coord const& producer_coord,
                             std::integral_constant<int, 0>) const
  {
    return 1;
  }

private:
  template <typename Coord>
  int get_result_id_helper(std::true_type,
                       Coord const& skeleton_size,
                       Coord const& coord) const
  {
    return result_mapper_t(m_start_corner, tuple_ops::front(skeleton_size))
             (tuple_ops::front(coord));
  }

  template <typename Coord>
  int get_result_id_helper(std::false_type,
                       Coord const& skeleton_size,
                       Coord const& coord) const
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

  corners_type const& get_start_corner() const
  {
    return m_start_corner;
  }

  Filter const& get_filter() const
  {
    return get<0>(m_filters);
  }

  template <typename Coord, typename Span>
  void configure(Coord&& cur_coord, Span&& span)
  {
    configure_mapper<
      0,
      !std::is_same<
        homogeneous_tuple_type_t<N, skeletons::no_mapper>,
        mappers_t>::value
      >::apply(m_mappers, m_op, std::forward<Coord>(cur_coord),
        std::forward<Span>(span), make_index_sequence<N>());

    configure_filter<
      0, !std::is_same<
           homogeneous_tuple_type_t<N, skeletons::no_filter>,
           filters_t>::value
      >::apply(m_filters, m_op, std::forward<Coord>(cur_coord),
        std::forward<Span>(span), make_index_sequence<N>());
  }

  void define_type(typer& t)
  {
    t.member(m_op);
    t.member(m_start_corner);
    t.member(m_filters);
    t.member(m_mappers);
  }
};


} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates a wavefront parametric dependency given a @c op.
///
/// @tparam num_inputs number of inputs not coming from the wavefront
///                    data flows.
/// @param op          operation to be applied at each point.
/// @param corners     the starting corner of the wavefront.
/// @param filter      filtering function to be applied on the producer
///                    side.
/// @param mappers     the mappers to be used for the paragraph ports
///
/// @ingroup skeletonsParamDeps
//////////////////////////////////////////////////////////////////////
template <int num_inputs,
          bool SetResult = false,
          typename Op,
          std::size_t dims,
          typename Filter,
          typename Mappers>
skeletons_impl::wavefront_pd<num_inputs, dims, Op, Filter, Mappers, SetResult>
wavefront_pd(Op const& op,
             std::array<skeletons::position, dims> const& corners,
             Filter const& filter,
             Mappers const& mappers)
{
  return skeletons_impl::wavefront_pd<
           num_inputs, dims, Op, Filter, Mappers, SetResult
           >(op, corners, filter, mappers);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_WAVEFRONT_PD_HPP
