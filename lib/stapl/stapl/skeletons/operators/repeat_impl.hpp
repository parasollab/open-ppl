/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_OPERATORS_REPEAT_IMPL_HPP
#define STAPL_SKELETONS_OPERATORS_REPEAT_IMPL_HPP

#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/skeletons/flows/repeat_flows.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

template<typename S, bool b = has_metadata_type<S>::value>
struct skeleton_dimensions_metadata_type
{
  using type = typename S::dims_type;
};

template<typename S>
struct skeleton_dimensions_metadata_type<S, true>
{
  using type = typename S::metadata_type;
};

//////////////////////////////////////////////////////////////////////
/// @brief A repeat skeleton spawns a single skeleton for the number of
/// times specified by @c SizeF.
///
/// @tparam S     the enclosed skeleton that will be spawned for @c SizeF
///               times
/// @tparam SizeF a size functor used to determine number of iterations
/// @tparam Flows specifies the high-level flow dependencies for this
///               @c repeat skeleton
///
/// @b Example -  A simple @c repeat skeleton with @c piped flow:
///
/// @copydetails flows::repeat_flows::piped
///
/// @ingroup skeletonsOperatorsInternal
//////////////////////////////////////////////////////////////////////
template <typename S, typename SizeF, typename Flows>
class repeat
{
public:
  using dims_type         = std::size_t;
  using nested_p_type     = S;
  using index_type        = typename S::index_type;
  using ports_t           = typename Flows::template port_types<repeat>;
  using in_port_type      = typename ports_t::in_port_type;
  using skeleton_tag_type = tags::unnamed_skeleton;

  static constexpr std::size_t in_port_size = ports_t::in_port_size;

  template <typename In>
  struct out_port_type
  {
    using type = typename ports_t::template out_port_type<In>::type;
  };

private:
  dims_type m_dims;
  S         m_skeleton;
  SizeF     m_size_functor;
public:
  repeat(S const& skeleton, SizeF const& size_functor)
    : m_dims(dims_type()),
      m_skeleton(skeleton),
      m_size_functor(size_functor)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief returns underlying skeleton to be repeated
  ///
  //////////////////////////////////////////////////////////////////////
  nested_p_type nested_skeleton() const
  {
    return m_skeleton;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief maps the input port of repeated skeleton
  ///
  /// @param id_offset  offset of repeat skeleton
  //////////////////////////////////////////////////////////////////////
  in_port_type
  in_port(std::size_t id_offset) const
  {
    return ports_t(*this).in_port(id_offset);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief maps the output port of repeated skeleton
  ///
  /// @tparam In         in flow type
  /// @param  id_offset  offset of repeat skeleton
  //////////////////////////////////////////////////////////////////////
  template <typename In>
  typename out_port_type<In>::type
  out_port(In const& in, std::size_t id_offset) const
  {
    return ports_t(*this).out_port(in, id_offset);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief calculate the offset of underlying skeleton based on repeat
  ///        offset and current iteration number
  ///
  /// @param repeat_lid_offset  offset of repeat skeleton
  /// @param iter_num           current iteration number
  //////////////////////////////////////////////////////////////////////
  std::size_t lid_offset(std::size_t repeat_lid_offset,
                         std::size_t iter_num) const
  {
    return repeat_lid_offset + nested_skeleton().last_id() * iter_num;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief maps the in port of current underlying skeleton based on the
  ///        the offset of repeat and iteration number
  ///
  /// @param repeat_lid_offset  offset of repeat skeleton
  /// @param iter_num           current iteration number
  //////////////////////////////////////////////////////////////////////
  typename nested_p_type::in_port_type
  nested_in_port(std::size_t repeat_lid_offset, std::size_t iter_num) const
  {
    return nested_skeleton().in_port(lid_offset(repeat_lid_offset, iter_num));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief maps the out port of current underlying skeleton based on the
  ///        the offset of repeat and iteration number
  ///
  /// @tparam In                 in flow type
  /// @param  repeat_lid_offset  offset of repeat skeleton
  /// @param  iter_num           current iteration number
  //////////////////////////////////////////////////////////////////////
  template <typename In>
  typename nested_p_type::template out_port_type<In>::type
  nested_out_port(In const& in,
                  std::size_t repeat_lid_offset,
                  std::size_t iter_num) const
  {
    return nested_skeleton().out_port(
             in, lid_offset(repeat_lid_offset, iter_num));
  }

private:
  typename skeleton_dimensions_metadata_type<S>::type
  apply_dimensions_metadata(std::true_type) const
  {
    return m_skeleton.dimensions_metadata();
  }

  typename skeleton_dimensions_metadata_type<S>::type
  apply_dimensions_metadata(std::false_type) const
  {
    return m_skeleton.dimensions();
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief sets the underlying skeleton sizes based on the given tuple
  /// of view sizes. It computes its own size by passing the underlying
  /// skeleton's size to the @c SizeF
  ///
  /// @param spawner      the spawner which is spawning this skeleton.
  /// @param views        the views which are passed to the skeleton.
  //////////////////////////////////////////////////////////////////////
  template <typename Spawner, typename... Views>
  void set_dimensions(Spawner const& spawner, Views const&... views)
  {
    m_skeleton.S::set_dimensions(spawner, views...);

    m_dims = m_size_functor(apply_dimensions_metadata(
               std::integral_constant<bool,
               has_metadata_type<S>::value>()));
  }

  dims_type dimensions() const
  {
    return m_dims;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief @c last_id is the maximum task id that this skeleton will
  /// create
  //////////////////////////////////////////////////////////////////////
  std::size_t last_id() const
  {
    return m_skeleton.last_id() * dimensions();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief A @c repeat skeleton starts the spawning process by spawning
  /// one iteration at a time and with the help of the flows given
  /// it redirects input and outputs in between the iterations. It
  /// continues the spawning process for the number of times given by
  /// @c SizeF.
  ///
  /// Similar to the other composite skeletons, a repeat skeleton can pause
  /// the spawning process and continue later on.
  ///
  /// @param spawner      is used for spawning the enclosed skeleton for
  ///                     the given number of times by @c SizeF
  /// @param lid_offset   the safe id to start the spawning from
  /// @param skeleton_size the size of skeletons enclosing this skeleton
  /// @param coord        the coordinate of this skeleton in the evaluated
  ///                     skeletons coordinate space
  /// @param in           the input flow passed to this skeleton
  /// @param out          the output flow that this skeleton would produce
  ///                     its results for
  /// @param cur_stage    current iteration number of the repetition
  /// @return true        if spawning of last iteration is finished
  ///
  /// @see repeat.hpp
  /// @see spawner.hpp
  //////////////////////////////////////////////////////////////////////
  template <typename Spawner,
            typename Coord,
            typename In, typename Out>
  bool spawn(Spawner& spawner, std::size_t lid_offset,
             Coord const& skeleton_size, Coord const& coord,
             In const& in, Out const& out,
             std::size_t cur_stage = 0)
  {
    bool is_complete = true;
    auto cur_coord = stapl::tuple_cat(make_tuple((std::size_t)0), coord);
    auto cur_p_size = stapl::tuple_cat(make_tuple(dimensions()), skeleton_size);
    ports_t ports(*this);
    std::size_t repeat_lid_offset = lid_offset;
    if (dimensions() > 1) {
      if (cur_stage == 0) {
        is_complete &= spawner.spawn(m_skeleton, lid_offset,
                                     cur_p_size, cur_coord,
                                     ports.in_flow(0, in,
                                                   repeat_lid_offset,
                                                   tags::repeat_first_iter()),
                                     ports.out_flow(0, out,
                                                    repeat_lid_offset,
                                                    tags::repeat_first_iter()));
        ++cur_stage;
      }

      //if the dimension is bigger than 2 and if we have not just come from the
      //previous if with (is_complete == false)
      if (dimensions() > 2 && cur_stage >= 1 && is_complete) {
        for (; cur_stage < dimensions() - 1; ++cur_stage) {
          tuple_ops::front(cur_coord) = cur_stage;
          lid_offset += m_skeleton.last_id();
          is_complete &= spawner.spawn(m_skeleton, lid_offset,
                                       cur_p_size, cur_coord,
                                       ports.in_flow(
                                               cur_stage, in,
                                               repeat_lid_offset,
                                               tags::repeat_iter()),
                                       ports.out_flow(
                                                cur_stage, out,
                                                repeat_lid_offset,
                                                tags::repeat_iter()));
          if (!is_complete)
            break;
        }
      }

      //if it is the last iteration, and we have not just come from the
      //previous if
      if (cur_stage == dimensions() - 1 && is_complete) {
        tuple_ops::front(cur_coord) = cur_stage;
        lid_offset += m_skeleton.last_id();
        is_complete &= spawner.spawn(m_skeleton, lid_offset,
                                     cur_p_size, cur_coord,
                                     ports.in_flow(cur_stage, in,
                                                   repeat_lid_offset,
                                                   tags::repeat_last_iter()),
                                     ports.out_flow(cur_stage, out,
                                                    repeat_lid_offset,
                                                    tags::repeat_last_iter()));
        //remember there should be a store point here, what if the underneath
        //is not finished we should wait for it to be finished, and then we can
        //tell the parent hey we are done
      }
    }
    else {
      // if there is only one repetition, then the output and inputs are fed
      // to this sp by its parent
      is_complete &=
        spawner.spawn(m_skeleton, lid_offset, cur_p_size, cur_coord,
                      ports.in_flow(0, in, repeat_lid_offset,
                                    tags::repeat_single_iter()),
                      ports.out_flow(0, out, repeat_lid_offset,
                                     tags::repeat_single_iter()));
    }

    if (is_complete)
      return true;
    else {
      //store your state in the memento and return false
      spawner.record_state(*this, lid_offset, skeleton_size, coord,
                           in, out,
                           cur_stage);
      return false;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_dims);
    t.member(m_skeleton);
    t.member(m_size_functor);
  }
};

} // namespace skeletons_impl
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_OPERATORS_REPEAT_IMPL_HPP
