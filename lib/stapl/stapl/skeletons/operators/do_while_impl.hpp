/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_OPERATORS_DO_WHILE_IMPL_HPP
#define STAPL_SKELETONS_OPERATORS_DO_WHILE_IMPL_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/flows/dontcare_flow.hpp>
#include <stapl/skeletons/flows/do_while_flows.hpp>
#include <tuple>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Enum representing the different nested component skeletons
/// in a @c do_while skeleton.
///
/// @see lid_offset
/// @see get_nested_skeleton
/// @see nested_in_port
/// @see nested_out_port
//////////////////////////////////////////////////////////////////////
enum class do_while_component : std::size_t {
 body = 0, reduction = 1, continuation = 2
};

//////////////////////////////////////////////////////////////////////
/// @brief Do-while is a composite skeleton that repeats a skeleton for
/// a number of iterations. This number is determined by evaluating
/// the @c ContinuationP after every iteration.
///
/// @b Example - A do-while skeleton with default @c piped flow:
///
/// @copydetails flows::do_while_flows::piped
///
/// @ingroup skeletonsOperatorsInternal
//////////////////////////////////////////////////////////////////////
template <typename BodyP, typename RedP, typename ContinuationP,
          typename Flows>
class do_while
{
public:
  using dims_type           = std::size_t;
  using body_p_type         = BodyP;
  using reduction_p_type    = RedP;
  using continuation_p_type = ContinuationP;
  using index_type          = typename BodyP::index_type;

  using in_port_type        = typename Flows::template port_types<
                                do_while>::in_port_type;
  using skeleton_tag_type   = tags::unnamed_skeleton;

  static constexpr std::size_t in_port_size =
    Flows::template port_types<do_while>::in_port_size;

  template <typename In>
  struct out_port_type
  {
    using type = typename Flows::template port_types<
                   do_while>::template out_port_type<In>::type;
  };

private:
  using coord_t = long long;

  dims_type     m_dims;
  BodyP         m_body_p;
  RedP          m_red_p;
  ContinuationP m_continuation_p;
  bool          m_has_hold;
public:
  do_while(BodyP const& body_p, RedP const& red_p,
           const ContinuationP& continuation_p)
    : m_dims(dims_type()),
      m_body_p(body_p),
      m_red_p(red_p),
      m_continuation_p(continuation_p),
      m_has_hold(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief sets the underlying skeleton sizes based on the given tuple
  /// of view sizes. In addition, it sets the dimension of the do_while
  /// itself to 0.
  ///
  /// @param spawner      the spawner which is spawning this skeleton.
  /// @param views        set of views passed to the skeleton.
  //////////////////////////////////////////////////////////////////////
  template <typename Spawner, typename... Views>
  void set_dimensions(Spawner const& spawner, Views const&... views)
  {
    m_body_p.set_dimensions(spawner, views...);
    m_red_p.set_dimensions(spawner,  views...);
    m_continuation_p.set_dimensions(spawner, views...);
    m_dims = 0;
  }

  dims_type dimensions() const
  {
    return m_dims;
  }

  body_p_type body_skeleton() const
  {
    return m_body_p;
  }

  reduction_p_type reduction_skeleton() const
  {
    return m_red_p;
  }

  continuation_p_type continuation_skeleton() const
  {
    return m_continuation_p;
  }

  std::size_t current_iteration() const
  {
    return dimensions() - 1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief @c last_id is the maximum task id that this skeleton will
  /// create
  //////////////////////////////////////////////////////////////////////
  std::size_t last_id() const
  {
    return (m_body_p.last_id() +
            m_red_p.last_id() +
            m_continuation_p.last_id()) * dimensions();
  }

  in_port_type
  in_port(std::size_t lid_offset) const
  {
    return Flows().in_port(*this, lid_offset);
  }

  template <typename In>
  typename out_port_type<In>::type
  out_port(In const& in, std::size_t lid_offset) const
  {
    return Flows().out_port(in, *this, lid_offset);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief calculate the offset of underlying skeleton based on do-while
  ///        offset and current iteration number
  ///
  /// @param repeat_lid_offset  offset of repeat skeleton
  /// @param iter_num           current iteration number
  /// @param c                  which component to get the offset for
  ///
  /// @see do_while_component
  //////////////////////////////////////////////////////////////////////
  std::size_t lid_offset(std::size_t do_while_lid_offset,
                         std::size_t iter_num,
                         do_while_component c = do_while_component::body) const
  {
    std::size_t iteration_size = m_body_p.last_id() +
                                 m_red_p.last_id() +
                                 m_continuation_p.last_id();

    size_t offset = do_while_lid_offset + iteration_size * iter_num;

    switch(c){
      case do_while_component::continuation: offset += m_red_p.last_id();
      case do_while_component::reduction:    offset += m_body_p.last_id();
      case do_while_component::body:
        break;
    }
    return offset;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the specified nested component skeleton.
  ///
  /// @see do_while_component
  //////////////////////////////////////////////////////////////////////
  template<do_while_component Comp>
  auto get_nested_skeleton() const
    STAPL_AUTO_RETURN((
      std::get<static_cast<std::size_t>(Comp)>(std::tie(
        m_body_p, m_red_p, m_continuation_p))
    ));

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the output port of one of the nested do_while
  /// skeletons (@a body, @a reduction, or @a continuation) for the
  /// given iteration.
  ///
  /// @param lid_offset      the lid offset for the @c do_while skeleton
  /// @param iter            the iteration to get the component skeleton
  ///                        for
  ///
  /// @return                the output port of the specified component
  ///                        skeleton
  ///
  /// @see do_while.hpp
  /// @see do_while_flows.hpp
  /// @see do_while_component
  /// @see current_iteration
  //////////////////////////////////////////////////////////////////////
  template<typename In, do_while_component Comp>
  auto nested_out_port(In const& in,
                       std::size_t lid_offset,
                       std::size_t iter) const
    STAPL_AUTO_RETURN((
      this->get_nested_skeleton<Comp>().out_port(
        in, this->lid_offset(lid_offset, iter, Comp))
    ));

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the input port of one of the internal do_while
  /// skeletons (@a body, @a reduction, or @a continuation) for the
  /// given iteration.
  ///
  /// @param lid_offset      the lid offset for the @c do_while skeleton
  /// @param iter            the iteration to get the component skeleton
  ///                        for
  ///
  /// @return                the input port of the specified component
  ///                        skeleton
  ///
  /// @see do_while.hpp
  /// @see do_while_flows.hpp
  /// @see do_while_component
  /// @see current_iteration
  //////////////////////////////////////////////////////////////////////
  template<do_while_component Comp>
  auto nested_in_port(std::size_t lid_offset, std::size_t iter) const
    STAPL_AUTO_RETURN((
      this->get_nested_skeleton<Comp>().in_port(
        this->lid_offset(lid_offset, iter, Comp))
    ));


  //////////////////////////////////////////////////////////////////////
  /// @brief Spawning of a do-while is more complicated than the other
  /// composite skeletons. A do-while starts by spawning the first
  /// iteration (@a body, @a reduction, and @a continuation) without
  /// knowing if it is one of the intermediate iterations or if it is
  /// the last iteration. Therefore, it creates three follow-up stubs:
  ///
  /// @li stub for making this iteration as the last iteration
  /// @li stub for making this iteration as one of the intermediate
  ///     iterations
  /// @li stub for the next iteration
  ///
  /// It is then the job of @c do_while_pd to invoke the appropriate stub
  /// based on the continuation condition. If the condition is satisfied
  /// (meaning that we have to continue with the next iteration), stubs
  /// (2) and (3) would be called, otherwise stub (1) would only be
  /// called.
  ///
  /// Like every other composite skeleton, do_while manages the offset
  /// for the skeletons so the spawning of the underlying skeletons would
  /// not overlap.
  ///
  /// @param spawner         is used for spawning the enclosed skeletons
  /// @param orig_lid_offset the safe id to start the spawning from
  /// @param skeleton_size    the size of skeletons enclosing this skeleton
  /// @param coord           the coordinate of this skeleton in the
  ///                        evaluated skeletons coordinate space
  /// @param in              the input flow passed to this skeleton
  /// @param out             the output flow that this skeleton would
  ///                        produce its results for
  /// @param cur_stage       current iteration number of the @c do_while
  ///                        loop
  /// @return true           if spawning of last iteration is finished
  ///
  /// @see do_while.hpp
  /// @see memento.hpp
  /// @see spawner.hpp
  //////////////////////////////////////////////////////////////////////
  template <typename Spawner,
            typename Coord,
            typename In, typename Out>
  bool spawn(Spawner& spawner,
             std::size_t orig_lid_offset,
             Coord const& skeleton_size, Coord const& coord,
             In const& in, Out const& out,
             std::size_t cur_stage)
  {
    auto cur_coord = stapl::tuple_cat(make_tuple(cur_stage), coord);
    auto cur_skeleton_size = stapl::tuple_cat(make_tuple(cur_stage),
                                            skeleton_size);
    Flows flows;
    //if there is a hold, then it means that this element was spawned before
    //otherwise we have to create a set_hold_spawner and spawn everything with
    //that
    if (m_has_hold) {
      //remove the hold at once
      m_has_hold = false;
      spawner.set_spawn_mode(skeletons::SET_NUM_SUCCS);
      //if cur_stage is set to max int then it means that this is the last
      //iteration so set the outputs accordingly

      if (cur_stage == boost::integer_traits<std::size_t>::const_max) {
        spawner.spawn(m_body_p,
                      lid_offset(orig_lid_offset, dimensions()-1),
                      cur_skeleton_size, cur_coord,
                      flows.in_flow(*this, in, orig_lid_offset,
                                    tags::while_iteration()),
                      flows.out_flow(*this, out, orig_lid_offset,
                                     tags::while_last_iter()));
      }
      //else compute all the num_succs as it was one of the intermediate
      //iterations
      else {
        spawner.spawn(m_body_p,
                      lid_offset(orig_lid_offset, dimensions()-1),
                      cur_skeleton_size, cur_coord,
                      flows.in_flow(*this, in, orig_lid_offset,
                                    tags::while_iteration()),
                      flows.out_flow(*this, out, orig_lid_offset,
                                     tags::while_iteration()));
      }
    }
    else {
      m_dims = cur_stage+1;
      spawner.set_spawn_mode(skeletons::SET_HOLD);

      if (cur_stage == 0) {
      spawner.spawn(m_body_p,
                    lid_offset(orig_lid_offset, 0),
                    cur_skeleton_size, cur_coord,
                    flows.in_flow(*this, in, orig_lid_offset,
                                  tags::while_first_iter()),
                    stapl::make_tuple(flows::dontcare_flow()));
      }
      else {
        spawner.spawn(m_body_p,
                      lid_offset(orig_lid_offset, cur_stage),
                      cur_skeleton_size, cur_coord,
                      flows.in_flow(*this, in, orig_lid_offset,
                                    tags::while_iteration()),
                      stapl::make_tuple(flows::dontcare_flow()));
      }
      spawner.set_spawn_mode(skeletons::DEFAULT);
      //TODO: for now the reduction part also does not need a hold
      size_t red_lid_offset = lid_offset(orig_lid_offset, cur_stage,
                                         do_while_component::reduction);
      size_t cont_lid_offset = lid_offset(orig_lid_offset, cur_stage,
                                         do_while_component::continuation);
      spawner.spawn(m_red_p,
                    red_lid_offset,
                    cur_skeleton_size, cur_coord,
                    flows.in_flow(*this, in, orig_lid_offset,
                                  tags::while_red()),
                    flows.out_flow(*this, out,orig_lid_offset,
                                   tags::while_red()));

      //the guard does not need an update for its successor count it is always
      //zero
      spawner.spawn(m_continuation_p,
                    cont_lid_offset,
                    cur_skeleton_size, cur_coord,
                    flows.in_flow(*this, in, orig_lid_offset,
                                  tags::while_continuation()),
                    stapl::make_tuple(flows::dontcare_flow()));

      //we have to store it in the front anyway
      m_has_hold = true;

      // |____________________| <--- Top of Memento
      // |cur_iter{last_iter} |
      // |____________________|
      // |   cur_iter{iter}   |
      // |____________________|
      // |   next_iter_while  |
      // |____________________|
      // |        ...         |
      // @see do_while.hpp

      //stub for the next iteration
      spawner.record_state(*this, orig_lid_offset, skeleton_size, coord,
                           in, out,
                           cur_stage+1,
                           true, true);
      //stub for making this an intermediate iteration, set_num_succs
      spawner.record_state(*this, orig_lid_offset, skeleton_size, coord,
                           in, out,
                           cur_stage,
                           true, true);
      //stub for making this iteration the last iteration, set_num_succs
      spawner.record_state(*this, orig_lid_offset, skeleton_size, coord,
                           in, out,
                           boost::integer_traits<std::size_t>::const_max,
                           true, true);
    }
    return false;
  }
};

} // namespace skeletons_impl
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_OPERATORS_DO_WHILE_IMPL_HPP
