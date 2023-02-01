/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.
// All rights reserved.
// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FLOWS_TRANSPOSE_2D_HPP
#define STAPL_SKELETONS_FLOWS_TRANSPOSE_2D_HPP

#include <utility>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/skeletons/flows/flow_helpers.hpp>

namespace stapl {
namespace skeletons {
namespace flows {

//////////////////////////////////////////////////////////////////////
/// @brief AUTOMATICALLY GENERATED DOCUMENTATION
///
/// @dot
///  digraph transpose_2d {
///    rankdir = LR;
///    node [shape=record];
///    in [label="Inflow", color="white"];
///    out [label="Outflow", color="white"];
///    subgraph cluster_transpose_2d {
///    a1[label=<P<sub>1</sub>>]
///    a0[label=<P<sub>0</sub>>]
///    a2[label=<P<sub>2</sub>>]
///    label=<Compose<sub>[transpose_2d]</sub>>;
///    }
///    a2 -> out;
///    a0 -> a1;
///    in -> a0[label="get<0>(in)"];
///    a1 -> a2;
///    in -> a2[label="get<1>(in)"];
///  }
/// @enddot
//////////////////////////////////////////////////////////////////////
struct transpose_2d
{
  template <typename Compose>
  struct port_types
  {
  private:
    using skeletons_t = typename Compose::skeletons_type;

    Compose const& m_compose;

  public:
    static constexpr std::size_t in_port_size = 2;
    using in_port_type = flows::result_of::concat<
      stapl::tuple<typename stapl::tuple_element <
        0,
        typename stapl::tuple_element<0, skeletons_t>::type::in_port_type>::type
      >,
      stapl::tuple<typename stapl::tuple_element <
        1,
        typename stapl::tuple_element<2, skeletons_t>::type::in_port_type>::type
      >
    >;

    template <typename In>
    struct inner_ports_types
    {
    public:
      using in_flow_0_type =
        stapl::tuple<typename stapl::tuple_element<0, In>::type>;

      using out_port_0_type =
        typename stapl::tuple_element<
          0, skeletons_t>::type::template out_port_type<in_flow_0_type>::type;

      static in_flow_0_type
      in_flow_0(Compose const& compose, In const& in, size_t lid_offset)
      {
        return stapl::make_tuple(tuple_ops::front(in));
      }

      static out_port_0_type
      out_port_0(Compose const& compose, In const& in, size_t lid_offset)
      {
        auto&& inner_flow = in_flow_0(compose, in, lid_offset);
        return compose.template get_skeleton<0>().out_port(inner_flow,
                                                           lid_offset);
      }

      using in_flow_1_type = out_port_0_type;

      using out_port_1_type =
        typename stapl::tuple_element<
          1, skeletons_t>::type::template out_port_type<in_flow_1_type>::type;

      static in_flow_1_type
      in_flow_1(Compose const& compose, In const& in, size_t lid_offset)
      {
        return out_port_0(compose, in, lid_offset);
      }

      static out_port_1_type
      out_port_1(Compose const& compose, In const& in, size_t lid_offset)
      {
        auto&& inner_flow = in_flow_1(compose, in, lid_offset);
        return compose.template get_skeleton<1>().out_port(inner_flow,
                                                           lid_offset);
      }

      using in_flow_2_type = flows::result_of::concat<
        out_port_1_type,
        stapl::tuple<typename stapl::tuple_element<1, In>::type> >;

      using out_port_2_type = typename
        stapl::tuple_element<2, skeletons_t>::type::
          template out_port_type<in_flow_2_type>::type;

      static in_flow_2_type
      in_flow_2(Compose const& compose, In const& in, size_t lid_offset)
      {
        return flows::concat(out_port_1(compose, in, lid_offset),
                             make_tuple(get<1>(in)));
      }

      static out_port_2_type
      out_port_2(Compose const& compose, In const& in, size_t lid_offset)
      {
        auto&& inner_flow = in_flow_2(compose, in, lid_offset);
        return compose.template get_skeleton<2>().out_port(inner_flow,
                                                           lid_offset);
      }
    };

    template <typename In>
    struct out_port_type
    {
    public:
      using type = typename inner_ports_types<In>::out_port_2_type;
    };

    port_types(Compose const& compose)
      : m_compose(compose)
    { }

    template <typename In>
    typename inner_ports_types<In>::out_port_0_type
    in_flow(In const& in, std::size_t lid_offset,
            std::integral_constant<int, 1>,
            std::integral_constant<bool, false> /*!is_last*/)
    {
      using inner_flow_t = typename inner_ports_types<In>::in_flow_0_type;
      inner_flow_t inner_flow =
        inner_ports_types<In>::in_flow_0(m_compose, in, lid_offset);

      return m_compose.template get_out_port<inner_flow_t , 0
             >(inner_flow, lid_offset);
    }

    template <typename Out>
    stapl::tuple<typename stapl::tuple_element <
      0,
      typename stapl::tuple_element<2, skeletons_t>::type::in_port_type>::type
    >
    out_flow(Out const& out, std::size_t lid_offset,
             std::integral_constant<int, 1>,
             std::integral_constant<bool, false> /*!is_last*/)
    {
      return stapl::make_tuple(
        std::get<0>(m_compose.template get_in_port<2>(lid_offset)));
    }

    template <typename In>
    stapl::tuple<typename stapl::tuple_element <
      0,
      In>::type
    >
    in_flow(In const& in, std::size_t lid_offset,
            std::integral_constant<int, 0>,
            std::integral_constant<bool, false> /*!is_last*/)
    {
      return stapl::make_tuple(
        std::get<0>(in));
    }

    template <typename Out>
    stapl::tuple<typename stapl::tuple_element <
      0,
      typename stapl::tuple_element<1, skeletons_t>::type::in_port_type>::type
    >
    out_flow(Out const& out, std::size_t lid_offset,
             std::integral_constant<int, 0>,
             std::integral_constant<bool, false> /*!is_last*/)
    {
      return stapl::make_tuple(
        std::get<0>(m_compose.template get_in_port<1>(lid_offset)));
    }

    template <typename In>
    flows::result_of::concat<
      typename inner_ports_types<In>::out_port_1_type,
      stapl::tuple<typename stapl::tuple_element <
        1,
        In>::type
      >
    >
    in_flow(In const& in, std::size_t lid_offset,
            std::integral_constant<int, 2>,
            std::integral_constant<bool, true> /*!is_last*/)
    {
      using inner_flow_t = typename inner_ports_types<In>::in_flow_1_type;
      inner_flow_t inner_flow =
        inner_ports_types<In>::in_flow_1(m_compose, in, lid_offset);

      return concat(m_compose.template get_out_port<inner_flow_t , 1>(
                      inner_flow, lid_offset),
                    stapl::make_tuple(get<1>(in)));
    }

    template <typename Out>
    stapl::tuple<typename stapl::tuple_element <
      0,
      Out>::type
    >
    out_flow(Out const& out, std::size_t lid_offset,
             std::integral_constant<int, 2>,
             std::integral_constant<bool, true> /*!is_last*/)
    {
      return stapl::make_tuple(
        std::get<0>(out));
    }

    typename port_types<Compose>::in_port_type
    in_port(std::size_t lid_offset)
    {
      return concat(
        stapl::make_tuple(
          std::get<0>(m_compose.template get_in_port<0>(lid_offset))),
        stapl::make_tuple(
          std::get<1>(m_compose.template get_in_port<2>(lid_offset)))
      );
    }

    template <typename In>
    typename out_port_type<In>::type
    out_port(In const& in, std::size_t lid_offset)
    {
      using inner_flow_t      = typename inner_ports_types<In>::in_flow_2_type;
      inner_flow_t inner_flow =
        inner_ports_types<In>::in_flow_2(m_compose, in, lid_offset);

      return m_compose.template get_out_port<inner_flow_t , 2>(inner_flow,
                                                               lid_offset);
    }

  };
};

} // namespace flows
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FLOWS_TRANSPOSE_2D_HPP
