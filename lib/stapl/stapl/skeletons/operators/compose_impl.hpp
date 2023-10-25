/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_OPERATORS_COMPOSE_IMPL_HPP
#define STAPL_SKELETONS_OPERATORS_COMPOSE_IMPL_HPP

#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/flows/compose_flows.hpp>
#include <initializer_list>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

template <typename Spawner, typename... Views>
struct set_dimensions_helper
{

  using views_tuple_t = std::tuple<Views...>;

  Spawner       const& m_spawner;
  views_tuple_t m_view_sizes;

  explicit set_dimensions_helper(Spawner const& spawner, Views... views)
    : m_spawner(spawner),
      m_view_sizes(std::make_tuple(views...))
  { }

  template <typename S, std::size_t... Indices>
  void operator_helper(S& skeleton, index_sequence<Indices...>&&) const
  {
    skeleton.set_dimensions(m_spawner, std::get<Indices>(m_view_sizes)...);
  }

  template <typename S>
  void operator()(S& skeleton) const
  {
    operator_helper(
      skeleton,
      make_index_sequence<std::tuple_size<views_tuple_t>::value>() );
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compose operator groups the given skeletons together and
/// performs the spawning process in the order of composition
/// (P0, P1, ... Pn). The spawning process starts from the first
/// skeleton (P0) and continues until the last skeleton(Pn)
/// is spawned.
///
/// By default skeletons in a @c compose are connected to their successors,
/// (@ref flows::compose_flows::piped) which is known as a pipeline in other
/// contexts. This behavior can be modified by using custom @c Flows.
///
/// @b Example - A simple @c compose with default @c piped flow:
/// @dot
/// digraph piped {
///     rankdir = LR;
///     node [shape=record];
///     in [label="In", color="white"];
///     out [label="Out", color="white"];
///     subgraph cluster_piped {
///       a0 [label=<S<sub>0</sub>>];
///       a1 [label=<S<sub>1</sub>>];
///       a2 [label="...", color="white"];
///       a3 [label=<S<sub>n</sub>>];
///       label=<Compose<sub>piped</sub>>;
///     }
///     a0 -> a1;
///     a1 -> a2;
///     a2 -> a3;
///     in -> a0;
///     a3 -> out;
/// }
/// @enddot
///
/// @see flows::compose_flows
///
/// @ingroup skeletonsOperatorsInternal
//////////////////////////////////////////////////////////////////////
template <typename Skeletons, typename Flows>
class compose
{
public:
  using dims_type         = std::size_t;
  using index_type        = typename stapl::tuple_element<
                                       0, Skeletons>::type::index_type;
  using skeletons_size    = std::integral_constant<
                              int, stapl::tuple_size<Skeletons>::value>;
  using skeletons_type    = Skeletons;
  using ports_t           = typename Flows::template port_types<compose>;
  using in_port_type      = typename ports_t::in_port_type;
  using skeleton_tag_type = tags::unnamed_skeleton;
  using flows_t           = Flows;

  static constexpr std::size_t in_port_size = ports_t::in_port_size;

  template <typename In>
  struct out_port_type
  {
    using type = typename ports_t::template out_port_type<In>::type;
  };

private:
  Skeletons  m_skeletons;
public:

  explicit compose(Skeletons skeletons)
    : m_skeletons(std::move(skeletons))
  { }

  compose() = delete;
  compose(compose const& other) = default;
  compose(compose&& other) = default;

  dims_type dimensions() const
  {
    return skeletons_size::value;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief sets the underlying skeleton sizes based on the given tuple
  /// of view sizes, and considers its own size as two (@c LeftP and
  /// @c RightP)
  ///
  /// @param spawner      the spawner which is spawning this skeleton.
  /// @param view_domains a tuple of view domains.
  //////////////////////////////////////////////////////////////////////
  template <typename Spawner, typename... Views>
  void set_dimensions(Spawner const& spawner, Views const&... views)
  {
    tuple_ops::for_each(m_skeletons,
                        skeletons_impl::set_dimensions_helper<
                          Spawner, Views...
                        >(spawner, views...));
  }

private:
  template <std::size_t... Index>
  std::size_t last_id(index_sequence<Index...>&&) const
  {
    std::initializer_list<std::size_t> a = {get_skeleton<Index>().last_id()...};
    return std::accumulate(a.begin(), a.end(), (std::size_t) 0);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief computes the id offset of a skeleton with the index @c up_to.
  ///
  /// The id offset is used in may places, such as various compose flows.
  ///
  /// @see flows::compose_flows
  //////////////////////////////////////////////////////////////////////
  template <std::size_t up_to>
  std::size_t lid_offset(std::size_t compose_lid_offset) const
  {
    return compose_lid_offset + last_id(make_index_sequence<up_to>());
  }

  std::size_t last_id() const
  {
    return last_id(make_index_sequence<skeletons_size::value>());
  }

  template <std::size_t N>
  auto get_skeleton() const
  STAPL_AUTO_RETURN((
    stapl::get<N>(m_skeletons)
  ))

  Skeletons
  get_skeletons() const
  {
    return m_skeletons;
  }

  in_port_type
  in_port(std::size_t id_offset) const
  {
    return ports_t(*this).in_port(id_offset);
  }

  template <typename In>
  typename out_port_type<In>::type
  out_port(In const& in, std::size_t id_offset) const
  {
    return ports_t(*this).out_port(in, id_offset);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the input port of the skeleton at index @c i and
  /// gives it an offset of @c id_offset
  //////////////////////////////////////////////////////////////////////
  template <int i>
  typename stapl::tuple_element<i, Skeletons>::type::in_port_type
  get_in_port(std::size_t id_offset) const
  {
    return this->template get_skeleton<i>().in_port(
             this->template lid_offset<i>(id_offset));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the output port of the skeleton at index @c i and
  /// gives it an offset of @c id_offset
  //////////////////////////////////////////////////////////////////////
  template <typename In, int i>
  typename stapl::tuple_element<i, Skeletons>::type::
    template out_port_type<In>::type
  get_out_port(In const& in, std::size_t id_offset) const
  {
    return this->template get_skeleton<i>().out_port(
             in, this->template lid_offset<i>(id_offset));
  }

  template <int i, typename State,
            typename Spawner, typename Coord,
            typename In, typename Out>
  State spawn_skeleton(State const& state,
                       Spawner& spawner,
                       std::size_t compose_lid_offset,
                       Coord const& skeleton_size, Coord const& coord,
                       In const& in, Out const& out)
  {
    bool is_complete;
    std::size_t lid_offset;
    std::size_t cur_stage;

    std::tie(is_complete, lid_offset, cur_stage) = state;
    auto&& sk = stapl::get<i>(m_skeletons);

    // if the spawning is left incomplete from the previous
    // stage do not continue
    if (is_complete) {
      if (i == cur_stage) {
        is_complete &= spawner.spawn(
                         sk,
                         lid_offset,
                         skeleton_size, coord,
                         ports_t(*this).in_flow(
                           in, compose_lid_offset,
                           std::integral_constant<int, i>(),
                           std::integral_constant<
                             bool,
                             (skeletons_size::value - 1 == i)
                           >()
                         ),
                         ports_t(*this).out_flow(
                           out, compose_lid_offset,
                           std::integral_constant<int, i>(),
                           std::integral_constant<
                             bool,
                             (skeletons_size::value - 1  == i)
                           >()
                         ));
        cur_stage++;
      }
    }
    return stapl::make_tuple(is_complete, lid_offset + sk.last_id(),
                             cur_stage);
  }

  template <typename State, typename... Args>
  State
  spawn_skeletons(std::integral_constant<int, 0>,
                  State const& state,
                  Args&&... args)
  {
    return spawn_skeleton<0>(state, std::forward<Args>(args)...);
  }

  template <int i, typename State, typename... Args>
  State
  spawn_skeletons(std::integral_constant<int, i>,
                  State const& state,
                  Args&&... args)
  {
    return
      spawn_skeleton<i>(
        spawn_skeletons(
          std::integral_constant<int, i-1>(), state,
          std::forward<Args>(args)...),
        std::forward<Args>(args)...
      );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief A @c compose skeleton starts the spawning process by spawning
  /// first the @c LeftP and then followed by spawning the @c RightP. If
  /// spawning of the @c LeftP is not finished (returned @c false). The
  /// spawning of the @c RightP would not start immediately.
  ///
  /// @param spawner      is used for spawning the enclosed skeleton
  ///                     for the given number of times by @c SizeF
  /// @param lid_offset   the safe id to start the spawning from
  /// @param skeleton_size the size of skeletons enclosing this skeleton
  /// @param coord        the coordinate of this skeleton in the
  ///                     evaluated skeletons coordinate space
  /// @param in           the input flow passed to this skeleton
  /// @param out          the output flow that this skeleton would
  ///                     produce its results for
  /// @param cur_stage    current state of the @c compose skeleton
  /// @return true        if spawning of last iteration is finished
  ///
  /// @see repeat.hpp
  /// @see spawner.hpp
  //////////////////////////////////////////////////////////////////////
  template <typename Spawner,
            typename Coord,
            typename In, typename Out>
  bool spawn(Spawner& spawner,
             std::size_t lid_offset,
             Coord const& skeleton_size, Coord const& coord,
             In const& in, Out const& out,
             std::size_t cur_stage)
  {
    auto new_coord = stapl::tuple_cat(make_tuple((std::size_t)0), coord);
    auto new_skeleton_size = stapl::tuple_cat(
                               make_tuple(dimensions()), skeleton_size);
    auto state = stapl::make_tuple(true, lid_offset, cur_stage);

    bool is_complete;
    std::size_t cur_lid_offset;

    std::tie(is_complete, cur_lid_offset, cur_stage) =
      spawn_skeletons(
        std::integral_constant<int, skeletons_size::value-1>(),
        state,
        spawner, lid_offset, new_skeleton_size, new_coord, in, out);

    if (is_complete)
      return true;
    else {
      spawner.record_state(*this,
                           lid_offset,
                           skeleton_size, coord,
                           in, out,
                           cur_stage);
      return false;
    }

  }

  void define_type(typer& t)
  {
    t.member(m_skeletons);
  }
};

} // namespace skeletons_impl
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_OPERATORS_COMPOSE_IMPL_HPP
