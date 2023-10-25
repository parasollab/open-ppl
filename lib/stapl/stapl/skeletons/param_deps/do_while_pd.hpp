/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PARAM_DEPS_DO_WHILE_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_DO_WHILE_PD_HPP

#include <type_traits>
#include <stapl/views/localize_element.hpp>
#include <stapl/skeletons/flows/producer_info.hpp>
#include <stapl/skeletons/executors/memento.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {


//////////////////////////////////////////////////////////////////////
/// @brief There are three elements pushed on the top of the memento
/// when this method is invoked. Two of them stand for the current
/// iteration and one is for the one that comes after this iteration.
///
/// @dot
/// digraph do_while_piped_graph {
///     rankdir = LR;
///     node [shape=record];
///     a0 [label="<b0> ...|cur-iter\[is_last\]|cur-iter\[\!is_last\]|next-iter|..."];
///     a1 [label="Top of Memento", color=white];
///     a1 -> a0:b0;
/// }
/// @enddot
///
/// As you can see the top element is the one which is used to set the
/// num of successors of the elements spawned in the current iteration,
/// as if the current iteration was one of the intermediate iterations.
/// The second from the top is for the same iteration as well but as if
/// the current iteration was the last iteration.
///
/// @tparam ContCond a continuation functor used in the do-while
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename ContCond>
struct do_while_stubs
{
  using result_type = void;
private:
  ContCond m_cont_cond;

public:
  explicit do_while_stubs(ContCond const& guard_cond) :
    m_cont_cond(std::move(guard_cond))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Given the value that it can pass to the condition functor,
  /// it restores the state using the skeleton manager
  ///
  /// @param element         the element that is sent to the continuation
  ///                        condition in order to find out if do-while
  ///                        should continue or finish
  /// @param memento_stack   the current skeleton manager which is
  ///                        responsible for continuation of the spawning
  ///                        process
  /// @see skeletons_impl::do_while_pd
  //////////////////////////////////////////////////////////////////////
  template <typename Element>
  void operator()(Element element, memento memento_stack) const
  {
    if (m_cont_cond(element)) {
      /// the already-run iteration is going to be one of the intermediate
      /// iterations, and we would like to continue with the next iteration.
      /// So run the stubs that make this one of the already run iteration as
      /// one of the intermediates, then run the stub that would create the
      /// next iteration

      // set num succs as intermediate iteration
      memento_stack.pop();
      memento_stack.resume(true);
      // spawn next iteration
      memento_stack.resume(true);
    }
    else {
      /// the already-run iteration is going to be last iteration of the
      /// do-while run the stub to make it as the last iteration and discard
      /// the other stubs

      // set num succs as last iteration
      memento_stack.resume(true);
      memento_stack.pop();
      // skip next iteration
      memento_stack.pop();
    }
  }

  void define_type(typer &t)
  {
    t.member(m_cont_cond);
  }

};


//////////////////////////////////////////////////////////////////////
/// @brief A @c do_while parametric dependency represents an element in the
/// dependence graph that controls if the next iteration of a do-while
/// loop should be spawned or not. It does that by wrapping user's
/// @c ContCond by @c do_while_stub with and sending @c memento_stack
/// to it.
///
/// @tparam ContCond a user provided continuation functor
///
/// @see do_while
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename ContCond>
class do_while_pd
  : public param_deps_defaults
{
  skeletons_impl::do_while_stubs<ContCond> m_op;

public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = 2;

  using op_type     = do_while_stubs<ContCond>;

  explicit do_while_pd(ContCond op)
    : m_op(std::move(op))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <i, h, ...> it wraps the @c WF with the
  /// following inputs and sends it to the visitor along with the @c m_op
  /// @li in<0>[i]
  /// @li current @c memento_stack
  ///
  /// @param skeleton_size <n, m, p, ...> where each element is
  ///                     potentially multi-dimensional.
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about WF and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord const& skeleton_size, Coord const& coord,
               Visitor& visitor, In&& in_flow) const
  {
    visitor(m_op,
            no_mapper(),
            stapl::get<0>(in_flow).consume_from(
              make_tuple(tuple_ops::front(coord), stapl::get<1>(coord) - 1)),
            reflexive_input(visitor.get_memento_stack()));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of.
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

  void define_type(typer& t)
  {
    t.member(m_op);
  }
};

} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates a do while parametric dependency given a
/// @c continuation_cond functor
///
/// @copybrief skeletons_impl::do_while_pd
///
/// @ingroup skeletonsParamDeps
//////////////////////////////////////////////////////////////////////
template <typename ContCond>
skeletons_impl::do_while_pd<ContCond>
do_while_pd(ContCond const& continuation_cond)
{
  return skeletons_impl::do_while_pd<ContCond>(continuation_cond);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PARAM_DEPS_DO_WHILE_PD_HPP
