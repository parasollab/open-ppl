/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_DO_WHILE_FLOWS_HPP
#define STAPL_SKELETONS_DO_WHILE_FLOWS_HPP

#include <stapl/skeletons/flows/flow_helpers.hpp>
#include <stapl/skeletons/operators/do_while_impl.hpp>

namespace stapl {
namespace skeletons {
namespace flows {
namespace do_while_flows {

using stapl::skeletons::skeletons_impl::do_while_component;

//////////////////////////////////////////////////////////////////////
/// @brief A @c piped flow for a @c do_while skeleton, connects the out
/// flow of each iteration to the input of the next iteration, and
/// connects the input of the @c do_while skeleton to the first
/// iteration of the while loop. It also connects the output of the
/// @c do_while skeleton to the output of the last iteration.
///
/// @dot
/// digraph do_while_piped_graph {
///     rankdir = LR;
///     node [shape=record];
///     in [label="In", color="white"];
///     out [label="Out", color="white"];
///     subgraph cluster_while {
///       a0 [label="{<b0> Body|Red.|Cont.}"];
///       a1 [label="{<b0> Body|Red.|Cont.}"];
///       a2 [label="...", color="white"];
///       label=<Do-While<sub>piped</sub>>;
///       a0:b0 -> a1:b0;
///     }
///     a0 -> a1;
///     a1 -> a2;
///     in -> a0;
///     a2 -> out;
/// }
/// @enddot
///
/// @ingroup skeletonsFlowsDoWhile
//////////////////////////////////////////////////////////////////////
struct piped
{
  template <typename While>
  struct port_types
  {
    static constexpr std::size_t in_port_size =
      While::body_p_type::in_port_size;
    using in_port_type = typename While::body_p_type::in_port_type;

    template <typename In>
    struct inner_ports_types
    {
      using body_in_flow_type  = In;
      using body_out_port_type = typename While::body_p_type::
                                   template out_port_type<
                                     body_in_flow_type>::type;
      static body_in_flow_type
      body_in_flow(While const& do_while, In const& in, size_t lid_offset)
      {
        return in;
      }

      static body_out_port_type
      body_out_port(While const& do_while, In const& in, size_t lid_offset)
      {
        auto&& inner_flow = body_in_flow(do_while, in, lid_offset);
        return do_while.body_skeleton().out_port(inner_flow, lid_offset);
      }

      using red_in_flow_type   = body_out_port_type;
      using red_out_port_type  = typename While::reduction_p_type::
                                   template out_port_type<
                                     red_in_flow_type>::type;

      static red_in_flow_type
      red_in_flow(While const& do_while, In const& in, size_t lid_offset)
      {
        return body_out_port(do_while, in, lid_offset);
      }

      static body_out_port_type
      red_out_port(While const& do_while, In const& in, size_t lid_offset)
      {
        auto&& inner_flow = red_in_flow(do_while, in, lid_offset);
        return do_while.reduction_skeleton().out_port(inner_flow, lid_offset);
      }

      using cont_in_flow_type  = red_out_port_type;
      using cont_out_port_type = typename While::continuation_p_type::
                                   template out_port_type<
                                     cont_in_flow_type>::type;

      static cont_in_flow_type
      cont_in_flow(While const& do_while, In const& in, size_t lid_offset)
      {
        return red_out_port(do_while, in, lid_offset);
      }

      static body_out_port_type
      cont_out_port(While const& do_while, In const& in, size_t lid_offset)
      {
        auto&& inner_flow = red_in_flow(do_while, in, lid_offset);
        return do_while.continuation_skeleton().out_port(inner_flow,
                                                         lid_offset);
      }

    };


    template <typename In>
    struct out_port_type
    {
      using type = typename While::body_p_type::
                     template out_port_type<In>::type;
    };
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief body<b>[first-iter]</b><sub>in-flow</sub> =
  ///        do-while<sub>in-flow</sub>
  //////////////////////////////////////////////////////////////////////
  template <typename While, typename In>
  In
  in_flow(While const&, In const& in, std::size_t,
          tags::while_first_iter)
  {
    return in;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief body<b>[iter]</b><sub>in-flow</sub> =
  ///        body<b>[iter-1]</b><sub>out-flow</sub>
  //////////////////////////////////////////////////////////////////////
  template <typename While, typename In>
  typename port_types<While>::template inner_ports_types<In>::body_out_port_type
  in_flow(While const& while_p, In const& in, std::size_t lid_offset,
          tags::while_iteration)
  {
    return while_p.template nested_out_port<In, do_while_component::body>(
      in, lid_offset, while_p.current_iteration() - 1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief reduction<b>[iter]</b><sub>in-flow</sub> =
  ///        body<b>[iter]</b><sub>out-flow</sub>
  //////////////////////////////////////////////////////////////////////
  template <typename While, typename In>
  typename port_types<While>::template inner_ports_types<In>::red_in_flow_type
  in_flow(While const& while_p, In const& in, std::size_t lid_offset,
          tags::while_red)
  {
    //connect to the current iteration's body
    return while_p.template nested_out_port<In, do_while_component::body>(
      in, lid_offset, while_p.current_iteration());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief continuation<b>[iter]</b><sub>in-flow</sub> =
  ///        reduction<b>[iter]</b><sub>out-flow</sub>
  //////////////////////////////////////////////////////////////////////
  template <typename While, typename In>
  typename port_types<While>::template inner_ports_types<In>::cont_in_flow_type
  in_flow(While const& while_p, In const& in, std::size_t lid_offset,
          tags::while_continuation)
  {
    using in_flow_t      = typename port_types<While>::
                             template inner_ports_types<In>::red_in_flow_type;
    in_flow_t inner_flow =
      port_types<While>::
        template inner_ports_types<In>::red_in_flow(while_p, in, lid_offset);

    return while_p .template nested_out_port<
             in_flow_t, do_while_component::reduction
           >(inner_flow, lid_offset, while_p.current_iteration());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief body<b>[iter]</b><sub>out-flow</sub> =
  ///        (reduction<b>[iter]</b><sub>in-flow</sub>,
  ///         body<b>[iter+1]</b><sub>in-flow</sub>)
  //////////////////////////////////////////////////////////////////////
  template <typename While, typename Out, typename Tag>
  flows::result_of::concat<
             typename While::reduction_p_type::in_port_type,
             typename While::body_p_type::in_port_type>
  out_flow(While const& while_p, Out const&, std::size_t lid_offset,
           Tag)
  {
    auto reduction_port =
      while_p.template nested_in_port<do_while_component::reduction>(
        lid_offset, while_p.current_iteration());
    auto body_port =
      while_p.template nested_in_port<do_while_component::body>(
        lid_offset, while_p.current_iteration() + 1);

    return flows::concat(reduction_port, body_port);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief body<b>[last-iter]</b><sub>out-flow</sub> =
  ///        (reduction<b>[last-iter]</b><sub>in-flow</sub>,
  ///         do-while<sub>out-flow</sub>)
  //////////////////////////////////////////////////////////////////////
  template <typename While, typename Out>
  flows::result_of::concat<
             typename While::reduction_p_type::in_port_type, Out>
  out_flow(While const& while_p, Out const& out,
           std::size_t lid_offset,
           tags::while_last_iter)
  {
    auto reduction_port =
      while_p.template nested_in_port<do_while_component::reduction>(
        lid_offset, while_p.current_iteration());
    return flows::concat(reduction_port, out);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief reduction<b>[iter]</b><sub>out-flow</sub> =
  ///        continuation<b>[iter]</b><sub>in-flow</sub>
  //////////////////////////////////////////////////////////////////////
  template <typename While, typename Out>
  typename While::continuation_p_type::in_port_type
  out_flow(While const& while_p, Out const&, std::size_t lid_offset,
           tags::while_red)
  {
    return while_p.template nested_in_port<do_while_component::continuation>(
      lid_offset, while_p.current_iteration());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief do-while<sub>in-flow</sub> =
  ///        body<b>[first-iter]</b><sub>in-flow</sub>
  //////////////////////////////////////////////////////////////////////
  template <typename While>
  typename port_types<While>::in_port_type
  in_port(While const& while_p, std::size_t lid_offset)
  {
    return while_p.body_skeleton().in_port(lid_offset);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief do-while<sub>out-flow</sub> =
  ///        body<b>[last-iter]</b><sub>out-flow</sub>
  //////////////////////////////////////////////////////////////////////
  template <typename In, typename While>
  typename port_types<While>::template out_port_type<In>::type
  out_port(In const& in, While const& while_p, std::size_t lid_offset)
  {
    //when the output flow of while is requested, the spawning of the while
    //should be done, otherwise we do not know where to connect to
    return while_p.template nested_out_port<In, do_while_component::body>(
      in, lid_offset, while_p.dimensions() - 1);
  }
};

} // namespace do_while_flows
} // namespace flows
} // namespace skeleton
} // namespace stapl

#endif // STAPL_SKELETONS_DO_WHILE_FLOWS_HPP
