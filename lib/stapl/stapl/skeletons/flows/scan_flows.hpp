/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SCAN_FLOWS_HPP
#define STAPL_SKELETONS_SCAN_FLOWS_HPP

#include <type_traits>
#include <stapl/skeletons/flows/flow_helpers.hpp>
#include <stapl/skeletons/flows/repeat_flows.hpp>
#include <stapl/skeletons/utility/tags.hpp>

namespace stapl {
namespace skeletons {
namespace flows {
namespace compose_flows {

namespace scan_inner_ports {

template <typename Compose, typename In, int i>
struct inner_ports_types
{
  using in_flow_type  = typename inner_ports_types<Compose, In, i - 1>
                          ::out_port_type;
  using out_port_type = typename tuple_element<
                          i, typename Compose::skeletons_type
                        >::type::template out_port_type<in_flow_type>::type;
};

template <typename Compose, typename In>
struct inner_ports_types<Compose, In, 0>
{
  using in_flow_type  = In;
  using out_port_type = typename tuple_element<
                          0, typename Compose::skeletons_type
                        >::type::template out_port_type<in_flow_type>::type;
};

}

//////////////////////////////////////////////////////////////////////
/// @brief A simplified compose flow for scans. The same flow can be written
/// as compose<input_to_last>(compose(a, b), c). This is done to reduce
/// compilation time while maintaining expressivity of this simple case.
///
/// @dot
/// digraph scan {
///     rankdir = LR;
///     node [shape=record];
///     in [label="In", color="white"];
///     out [label="Out", color="white"];
///     subgraph cluster_coarse_scan_flow {
///       a0 [label=<P<sub>0</sub>>]
///       a1 [label=<P<sub>1</sub>>]
///       a2 [label=<P<sub>2</sub>>]
///       label=<Compose<sub>scan_compose</sub>>;
///     }
///     a0 -> a1;
///     a1 -> a2;
///     a2 -> out;
///     in -> a0;
///     in -> a2;
/// }
/// @enddot
///
/// @ingroup skeletonsFlowsCompose
//////////////////////////////////////////////////////////////////////
struct scan
{
  template <typename Compose>
  struct port_types
  {
  private:
    using skeletons_t = typename Compose::skeletons_type;
    Compose const& m_compose;
  public:
    static constexpr std::size_t in_port_size = 1;

    using in_port_type = tuple<flows::result_of::concat<
      typename tuple_element<0, skeletons_t>::type::in_port_type,
      stapl::tuple<typename tuple_ops::result_of::back<
        typename tuple_element<2, skeletons_t>::type::in_port_type>::type>>>;

    template <typename In>
    struct inner_ports_types
    {
      using in_flow_0_type  = In;
      using out_port_0_type = typename tuple_element<0, skeletons_t>::type::
                                template out_port_type<in_flow_0_type>::type;

      static in_flow_0_type
      in_flow_0(Compose const& compose, In const& in, size_t lid_offset)
      {
        return in;
      }

      static out_port_0_type
      out_port_0(Compose const& compose, In const& in, size_t lid_offset)
      {
        auto&& inner_flow = in_flow_0(compose, in, lid_offset);
        return compose.template get_skeleton<0>().out_port(inner_flow,
                                                           lid_offset);
      }

      using in_flow_1_type  = out_port_0_type;
      using out_port_1_type = typename tuple_element<1, skeletons_t>::type::
                                template out_port_type<in_flow_1_type>::type;

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

      using  in_flow_2_type = flows::result_of::concat<out_port_1_type, In>;
      using out_port_2_type = typename tuple_element<2, skeletons_t>::type::
                                template out_port_type<in_flow_2_type>::type;

      static in_flow_2_type
      in_flow_2(Compose const& compose, In const& in, size_t lid_offset)
      {
        return flows::concat(out_port_1(compose, in, lid_offset), in);
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
      using type = typename inner_ports_types<In>::out_port_2_type;
    };


    explicit port_types(Compose const& compose)
      : m_compose(compose)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[0]<sub>in-flow</sub> =
    ///        compose<sub>in-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    In
    in_flow(In const& in, std::size_t, std::integral_constant<int, 0>,
            std::integral_constant<bool, false> /*!is_last*/)
    {
      return in;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[1]<sub>in-flow</sub> =
    ///        P[0]<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    typename inner_ports_types<In>::in_flow_1_type
    in_flow(In const& in, std::size_t lid_offset,
            std::integral_constant<int, 1>,
            std::integral_constant<bool, false> /*!is_last*/)
    {
      return m_compose.template get_skeleton<0>().
               template out_port<In>(in, lid_offset);
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[2]<sub>in-flow</sub> =
    ///        [P[1]<sub>out-flow</sub>,
    ///         compose<sub>in-flow</sub>]
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    typename inner_ports_types<In>::in_flow_2_type
    in_flow(In const& in, std::size_t lid_offset,
            std::integral_constant<int, 2>,
            std::integral_constant<bool, true> /*is_last*/)
    {
      auto&& inner_flow =
        inner_ports_types<In>::in_flow_1(m_compose, in, lid_offset);

      return flows::concat(
               m_compose.template get_skeleton<1>().
                out_port(
                    inner_flow,
                    m_compose.template lid_offset<1>(lid_offset)
                  ),
               in);
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[0]<sub>out-flow</sub> =
    ///        P[1]<sub>in-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out>
    typename tuple_element<1, skeletons_t>::type::in_port_type
    out_flow(Out const&, std::size_t lid_offset,
             std::integral_constant<int, 0>,
             std::integral_constant<bool, false> /*!is_last*/)
    {
      return m_compose.template get_skeleton<1>().in_port(
                 m_compose.template lid_offset<1>(lid_offset));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[1]<sub>out-flow</sub> =
    ///        pop_back(P[2]<sub>in-flow</sub>)
    //////////////////////////////////////////////////////////////////////
    template <typename Out>
    typename tuple_ops::result_of::pop_back<
      typename tuple_element<2, skeletons_t>::type::in_port_type
    >::type
    out_flow(Out const&, std::size_t lid_offset,
             std::integral_constant<int, 1>,
             std::integral_constant<bool, false> /*!is_last*/)
    {
      return tuple_ops::pop_back(
               m_compose.template get_skeleton<2>().in_port(
                 m_compose.template lid_offset<2>(lid_offset)));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[last]<sub>out-flow</sub> =
    ///        compose<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out>
    Out
    out_flow(Out const& out, std::size_t /*lid_offset*/,
             std::integral_constant<int, 2>,
             std::integral_constant<bool, true> /*is_last*/)
    {
      return out;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief compose<sub>in-flow</sub> =
    ///        [P[0]<sub>in-flow</sub>,
    ///         back(P[1]<sub>in-flow</sub>),
    ///         ...,
    ///         back(P[n-1]<sub>in-flow</sub>)]
    //////////////////////////////////////////////////////////////////////
    typename port_types<Compose>::in_port_type
    in_port(std::size_t lid_offset)
    {
      return typename port_types<Compose>::in_port_type(flows::concat(
        m_compose.template get_in_port<0>(lid_offset),
        stapl::make_tuple(
          tuple_ops::back(m_compose.template get_in_port<2>(lid_offset)))));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief compose<sub>out-flow</sub> =
    ///        P[n-1]<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    typename out_port_type<In>::type
    out_port(In const& in, std::size_t lid_offset)
    {
      auto&& inner_flow =
        inner_ports_types<In>::in_flow_2(m_compose, in, lid_offset);

      return m_compose.template get_skeleton<2>().
               out_port(
                 inner_flow,
                 m_compose.template lid_offset<2>(lid_offset));
    }
  };
};

} // namespace compose_flows

namespace repeat_flows {

//////////////////////////////////////////////////////////////////////
/// @brief A @c scan_broadcast flow is a customized flow for a
/// @c repeat skeleton. It connects the out flow of each iteration to
/// the input of the next iteration, and in addition connects the
/// first flow from the input flow to the @c repeat skeleton to the
/// input flow of every iteration. For the last iteration it connects
/// the last flow (@c back) of the input flow instead of the first
/// flow.
///
/// @dot
/// digraph scan_broadcast {
///     rankdir = LR;
///     node [shape=record];
///     in [label="In", color="white"];
///     out [label="Out", color="white"];
///     subgraph cluster_scan_broadcast {
///       a0 [label=<Iter<sub>0</sub>>];
///       a1 [label=<Iter<sub>1</sub>>];
///       a2 [label="...", color="white"];
///       a3 [label=<Iter<sub>n</sub>>];
///       label="Repeat<sub>scan_broadcast</sub>";
///     }
///     a0 -> a1;
///     a1 -> a2;
///     a2 -> a3;
///     in -> a0 [label=<<i>front(in)</i>>];
///     in -> a1 [label=<<i>front(in)</i>>];
///     in -> a3 [label=<<i>back(in)</i>>];
///     a3 -> out;
/// }
/// @enddot
///
/// @ingroup skeletonsFlowsRepeat
//////////////////////////////////////////////////////////////////////
struct scan_broadcast
{
  //////////////////////////////////////////////////////////////////////
  /// @brief This is a customized flow over a an existing flow, that
  /// only converts the @c consumers_count request to 2D requests.
  /// This is used for @c scan skeleton.
  ///
  /// @tparam Flow the underlying flow
  //////////////////////////////////////////////////////////////////////
  template <typename Flow>
  class custom_flow
    : private Flow
  {
  public:
    explicit custom_flow(Flow const& flow)
      : Flow(flow)
    { }

    template <typename F = skeletons::no_filter>
    using producer_type = typename Flow::template producer_type<F>;

    template <typename Index, typename F = skeletons::no_filter>
    producer_type<F>
    consume_from(Index const& index, F const& filter = F()) const
    {
      return Flow::template consume_from(index, filter);
    }

    template <typename Coord>
    std::size_t consumer_count(Coord const& producer_coord) const
    {
      return Flow::consumer_count(
               stapl::make_tuple(tuple_ops::front(producer_coord), -1));
    }
  };

  template <typename Repeat>
  struct port_types
  {
  private:
    using nested_p_type = typename Repeat::nested_p_type;
    Repeat const& m_repeat;
  public:
    static constexpr std::size_t in_port_size  =
      Repeat::nested_p_type::in_port_size - 1;
    using in_port_type  = typename tuple_ops::result_of::pop_front<
                            typename Repeat::nested_p_type::in_port_type>::type;


    template <typename In>
    struct inner_port_types
    {
      using in_flow_t    = typename tuple_ops::result_of::front<In>::type;

      using in_flow_type = stapl::tuple<in_flow_t, in_flow_t>;

      static in_flow_type
      in_flow(Repeat const& /*compose*/, In const& in, size_t lid_offset)
      {
        auto&& front_flow = tuple_ops::front(in);
        return stapl::make_tuple(front_flow, front_flow);
      }
    };

    template <typename In>
    struct out_port_type
    {
      /// remember that repeat skeleton is assumed to be homogeneous
      /// so we can simply use the input for the first level, in order
      /// to find the out-port of this flow
      using type =
        typename repeat_flows::piped::port_types<Repeat>::
          template out_port_type<
            typename inner_port_types<In>::in_flow_type
          >::type;
    };


    explicit port_types(Repeat const& repeat)
      : m_repeat(repeat)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @brief P<b>[single-iter]</b><sub>in-flow</sub> =
    ///        repeat<sub>in-flow</sub>
    ///
    /// If there is only one iteration, we should read from the input not
    /// from the partial results of the reduce tree.
    /// The dont-care-flow is used in here to make the requests by
    /// @c scan_broadcast_pd regular.
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    In
    in_flow(std::size_t, In const& in, std::size_t, tags::repeat_single_iter)
    {
      return in;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P<b>[first-iter]</b><sub>in-flow</sub> =
    ///        [dont-care-flow,
    ///         front(repeat<sub>in-flow</sub>]
    ///
    /// The dont-care-flow is used in here to make the requests by
    /// @c scan_broadcast_pd regular.
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    typename inner_port_types<In>::in_flow_type
    in_flow(std::size_t, In const& in, std::size_t, tags::repeat_first_iter)
    {
      return stapl::make_tuple(tuple_ops::front(in), tuple_ops::front(in));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P<b>[iter]</b><sub>in-flow</sub> =
    ///        [P<b>[iter -1]</b><sub>out-flow</sub>,
    ///         front(repeat<sub>in-flow</sub>]
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    flows::result_of::concat<
      typename nested_p_type::template out_port_type<
        typename inner_port_types<In>::in_flow_type
      >::type,
      stapl::tuple<typename tuple_ops::result_of::front<In>::type>>
    in_flow(std::size_t iter_num, In const& in, std::size_t lid_offset,
            tags::repeat_iter)
    {
      auto&& inner_flow =
        inner_port_types<In>::in_flow(m_repeat, in, lid_offset);

      return flows::concat(
               m_repeat.nested_out_port(
                 inner_flow, lid_offset, iter_num -1),
               stapl::make_tuple(tuple_ops::front(in)));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P<b>[last-iter]</b><sub>in-flow</sub> =
    ///        [P<b>[last-iter -1]</b><sub>out-flow</sub>,
    ///         back(repeat<sub>in-flow</sub>]
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    flows::result_of::concat<
      typename nested_p_type::template out_port_type<
        typename inner_port_types<In>::in_flow_type
      >::type,
      tuple<
        custom_flow<typename std::decay<
                      typename tuple_ops::result_of::back<In>::type>::type>>>
    in_flow(std::size_t iter_num, In const& in,
            std::size_t lid_offset,
            tags::repeat_last_iter)
    {
      auto&& inner_flow =
        inner_port_types<In>::in_flow(m_repeat, in, lid_offset);

      using custom_flow_t = custom_flow<typename std::decay<
                              typename tuple_ops::result_of::back<In>::type
                            >::type>;

      return flows::concat(
               m_repeat.nested_out_port(
                 inner_flow, lid_offset, iter_num - 1),
               stapl::make_tuple(custom_flow_t(tuple_ops::back(in))));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P<b>[single-iter]</b><sub>out-flow</sub> =
    ///        out
    //////////////////////////////////////////////////////////////////////
    template <typename Out>
    Out
    out_flow(std::size_t, Out const& out, std::size_t,
             tags::repeat_single_iter)
    {
      return out;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P<b>[iter]</b><sub>out-flow</sub> =
    ///        front(P<b>[iter+1]</b><sub>in-flow</sub>)
    //////////////////////////////////////////////////////////////////////
    template <typename Out, typename Tag>
    stapl::tuple<
      typename tuple_ops::result_of::front<
        typename nested_p_type::in_port_type>::type>
    out_flow(std::size_t iter_num, Out const&, std::size_t lid_offset, Tag)
    {
      return stapl::make_tuple(
               tuple_ops::front(
                 m_repeat.nested_in_port(lid_offset, iter_num+1)));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P<b>[last-iter]</b><sub>out-flow</sub> =
    ///        repeat<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out>
    Out
    out_flow(std::size_t, Out const& out, std::size_t, tags::repeat_last_iter)
    {
      return out;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief repeat<sub>in-flow</sub> =
    ///        pop_front(P<b>[first_iter]</b><sub>in-flow</sub>)
    //////////////////////////////////////////////////////////////////////
    typename port_types<Repeat>::in_port_type
    in_port(std::size_t lid_offset)
    {
      return tuple_ops::pop_front(
               m_repeat.nested_skeleton().in_port(lid_offset));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief repeat<sub>out-flow</sub> =
    ///        P<b>[last-iter]</b><sub>out-flow</sub>)
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    typename out_port_type<In>::type
    out_port(In const& in, std::size_t lid_offset)
    {
      auto&& inner_flow =
        inner_port_types<In>::in_flow(m_repeat, in, lid_offset);

      std::size_t iter_num = m_repeat.dimensions();
      return m_repeat.nested_out_port(
               inner_flow, lid_offset, iter_num-1);
    }

  };
};

}

} // namespace flows
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_SCAN_FLOWS_HPP
