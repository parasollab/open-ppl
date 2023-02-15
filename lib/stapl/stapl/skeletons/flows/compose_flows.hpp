/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_COMPOSE_FLOWS_HPP
#define STAPL_SKELETONS_COMPOSE_FLOWS_HPP

#include <type_traits>
#include <stapl/skeletons/flows/flow_helpers.hpp>
#include <stapl/skeletons/utility/tags.hpp>

namespace stapl {
namespace skeletons {
namespace flows {
namespace compose_flows {
namespace piped_inner_ports {

template <typename Compose, typename In, int i>
struct inner_ports_types
{
  using in_flow_type  = typename inner_ports_types<
                        Compose, In, i - 1>::out_port_type;
  using out_port_type = typename tuple_element<
                          i, typename Compose::skeletons_type
                        >::type::template out_port_type<in_flow_type>::type;

  static in_flow_type
  in_flow(Compose const& compose, In const& in, size_t lid_offset)
  {
    return inner_ports_types<
             Compose, In, i - 1>::out_port(compose, in, lid_offset);
  }

  static out_port_type
  out_port(Compose const& compose, In const& in, size_t lid_offset)
  {
    auto&& inner_flow = in_flow(compose, in, lid_offset);
    return compose.template get_skeleton<i>().out_port(inner_flow, lid_offset);
  }
};

template <typename Compose, typename In>
struct inner_ports_types<Compose, In, 0>
{
  using in_flow_type  = In;
  using out_port_type = typename tuple_element<
                          0, typename Compose::skeletons_type
                        >::type::template out_port_type<in_flow_type>::type;

  static in_flow_type
  in_flow(Compose const& /*compose*/, In const& in, size_t lid_offset)
  {
    return in;
  }

  static out_port_type
  out_port(Compose const& compose_sk, In const& in, size_t lid_offset)
  {
    auto&& inner_flow = in_flow(compose_sk, in, lid_offset);
    return compose_sk.template get_skeleton<0>().out_port(inner_flow,
                                                          lid_offset);
  }
};

} // piped_inner_ports

//////////////////////////////////////////////////////////////////////
/// @brief A @c piped flow for a @c compose, pipes the input flow of the
/// @c compose to the in flow of the left skeleton. The output flow of the
/// left skeleton to the input flow of right skeleton. Finally, it pipes
/// the output flow of the right skeleton to the output flow of the
/// @c compose
///
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
/// @ingroup skeletonsFlowsCompose
//////////////////////////////////////////////////////////////////////
struct piped
{
  template <typename Compose>
  struct port_types
  {
  private:
    using skeletons_t    = typename Compose::skeletons_type;
    using skeletons_size = typename Compose::skeletons_size;
  protected:
    Compose const& m_compose;
  public:
    static constexpr std::size_t in_port_size =
      tuple_element_t<0, skeletons_t>::in_port_size;
    using in_port_type   =
      typename tuple_element_t<0, skeletons_t>::in_port_type;

    template <typename In, int i>
    struct inner_ports_types
    {
      using in_flow_type  = typename piped_inner_ports::inner_ports_types<
                              Compose, In, i>::in_flow_type;
      using out_port_type = typename piped_inner_ports::inner_ports_types<
                              Compose, In, i>::out_port_type;

      static in_flow_type
      in_flow(Compose const& compose, In const& in, size_t lid_offset)
      {
        return piped_inner_ports::inner_ports_types<Compose, In, i>::in_flow(
          compose, in, lid_offset);
      }
    };

    template <typename In>
    struct out_port_type
    {
      using type = typename inner_ports_types<
                     In, (skeletons_size::value - 1)>::out_port_type;
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
    in_flow(In const& in, std::size_t,
            std::integral_constant<int, 0>,
            std::false_type /*!is_last*/)
    {
      return in;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[i]<sub>in-flow</sub> =
    ///        P[i-1]<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In, int i, bool B>
    typename std::enable_if<
      (i > 0), inner_ports_types<In, i-1>
    >::type::out_port_type
    in_flow(In const& in, std::size_t lid_offset,
            std::integral_constant<int, i>,
            std::integral_constant<bool, B> /*is_last*/)
    {
      auto&& inner_flow =
         inner_ports_types<In, i-1>::in_flow(m_compose, in, lid_offset);

      return m_compose.template get_skeleton<i-1>().
               out_port(
                 inner_flow,
                 m_compose.template lid_offset<i-1>(lid_offset));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[i]<sub>out-flow</sub> =
    ///        P[i+1]<sub>in-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out, int i>
    typename std::enable_if<
      (i < skeletons_size::value-1), tuple_element<i+1, skeletons_t>
    >::type::type::in_port_type
    out_flow(Out&& /*out*/, std::size_t lid_offset,
             std::integral_constant<int, i>,
             std::false_type /*!is_last*/)
    {
      return m_compose.template get_skeleton<i+1>().in_port(
               m_compose.template lid_offset<i+1>(lid_offset));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[last]<sub>out-flow</sub> =
    ///        compose<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out, int i>
    Out
    out_flow(Out&& out, std::size_t /*lid_offset*/,
             std::integral_constant<int, i>,
             std::true_type /*is_last*/)
    {
      return out;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief compose<sub>in-flow</sub> =
    ///        P[0]<sub>in-flow</sub>
    //////////////////////////////////////////////////////////////////////
    typename port_types<Compose>::in_port_type
    in_port(std::size_t lid_offset)
    {
      return m_compose.template get_skeleton<0>().in_port(lid_offset);
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief compose<sub>out-flow</sub> =
    ///        P[last]<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    typename out_port_type<In>::type
    out_port(In const& in, std::size_t lid_offset)
    {
      lid_offset = m_compose.template lid_offset<
                     skeletons_size::value - 1>(lid_offset);

      auto&& inner_flow  =
        inner_ports_types<In, skeletons_size::value - 1>::in_flow(
          m_compose, in, lid_offset);

      return m_compose.template get_skeleton<skeletons_size::value - 1>().
               out_port(inner_flow, lid_offset);
    }
  };

};

namespace input_to_all_inner_ports {

template <typename Compose, typename In, int i>
struct inner_ports_types
{
  using in_flow_type  = result_of::concat<
                          typename inner_ports_types<
                            Compose, In, i-1>::out_port_type,
                          In>;
  using out_port_type = typename tuple_element<
                          i, typename Compose::skeletons_type
                        >::type::template out_port_type<in_flow_type>::type;

  static in_flow_type
  in_flow(Compose const& compose, In const& in, size_t lid_offset)
  {
    return flows::concat(
      inner_ports_types<Compose, In, i - 1>::out_port(compose, in, lid_offset),
      in);
  }

  static out_port_type
  out_port(Compose const& compose, In const& in, size_t lid_offset)
  {
    auto&& inner_flow = in_flow(compose, in, lid_offset);
    return compose.template get_skeleton<i>().out_port(inner_flow, lid_offset);
  }
};

template <typename Compose, typename In>
struct inner_ports_types<Compose, In, 0>
{
  using in_flow_type  = In;
  using out_port_type = typename tuple_element<
                          0, typename Compose::skeletons_type
                        >::type::template out_port_type<in_flow_type>::type;

  static in_flow_type
  in_flow(Compose const& compose, In const& in, size_t lid_offset)
  {
    return in;
  }

  static out_port_type
  out_port(Compose const& compose, In const& in, size_t lid_offset)
  {
    auto&& inner_flow = in_flow(compose, in, lid_offset);
    return compose.template get_skeleton<0>().out_port(inner_flow, lid_offset);
  }
};

}

//////////////////////////////////////////////////////////////////////
/// @brief An @c input_to_all flow for a @c compose is similar to
/// @c compose_flows::piped, but in addition it pipes the input flow
/// of the @c compose to the @c rightP as well.
///
/// @dot
/// digraph input_to_all {
///     rankdir = LR;
///     node [shape=record];
///     in [label="In", color="white"];
///     out [label="Out", color="white"];
///     subgraph cluster_input_to_all {
///       a0 [label="Left-Skeleton"];
///       a1 [label="Right-Skeleton"];
///       label=<Compose<sub>input_to_all</sub>>;
///     }
///     a0 -> a1;
///     a1 -> out;
///     in -> a0;
///     in -> a1;
/// }
/// @enddot
///
/// @ingroup skeletonsFlowsCompose
//////////////////////////////////////////////////////////////////////
struct input_to_all
{
  template <typename Compose>
  struct port_types
  {
  private:
    using skeletons_t    = typename Compose::skeletons_type;
    using skeletons_size = typename Compose::skeletons_size;
  protected:
    Compose const& m_compose;
  public:
    static constexpr std::size_t in_port_size = skeletons_size::value;
    using in_port_type   = flows::result_of::concat<
                             typename tuple_element_t<
                               0, skeletons_t>::in_port_type,
                             typename tuple_ops::result_of::pop_front<
                               flows::result_of::in_flows<
                                 Compose, flows::get_back_modifier>>::type>;

    template <typename In, int i>
    struct inner_ports_types
    {
      using in_flow_type  = typename input_to_all_inner_ports::
                              inner_ports_types<Compose, In, i>::in_flow_type;
      using out_port_type = typename input_to_all_inner_ports::
                              inner_ports_types<Compose, In, i>::out_port_type;

      static in_flow_type
      in_flow(Compose const& compose, In const& in, size_t lid_offset)
      {
        return input_to_all_inner_ports::inner_ports_types<
          Compose, In, i>::in_flow(compose, in, lid_offset);
      }
    };

    template <typename In>
    struct out_port_type
    {
      using type = typename inner_ports_types<
                     In, (skeletons_size::value - 1)>::out_port_type;
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
            std::false_type /*is_last*/)
    {
      return in;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[i]<sub>in-flow</sub> =
    ///        P[i-1]<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In, int i, bool B>
    result_of::concat<
      typename std::enable_if<
        (i > 0), inner_ports_types<In, i-1>
      >::type::out_port_type,
      In
     >
    in_flow(In const& in, std::size_t lid_offset,
            std::integral_constant<int, i>,
            std::integral_constant<bool, B> /*is_last*/)
    {
      auto&& inner_flow =
        inner_ports_types<In, i-1>::in_flow(m_compose, in, lid_offset);

      return flows::concat(
               m_compose.template get_skeleton<i-1>().
                 out_port(
                  inner_flow, m_compose.template lid_offset<i-1>(lid_offset)),
               in
            );
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[i]<sub>out-flow</sub> =
    ///        P[i+1]<sub>in-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out, int i>
    tuple<
      typename tuple_ops::result_of::front<
        typename std::enable_if<
          (i < skeletons_size::value-1), tuple_element<i+1, skeletons_t>
        >::type::type::in_port_type>::type>
    out_flow(Out const&, std::size_t lid_offset, std::integral_constant<int, i>,
             std::false_type /*is_last*/)
    {
      return stapl::make_tuple(
               tuple_ops::front(
                 m_compose.template get_skeleton<i+1>().in_port(
                   m_compose.template lid_offset<i+1>(lid_offset))));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[last]<sub>out-flow</sub> =
    ///        compose<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out, int i>
    Out
    out_flow(Out const& out, std::size_t /*lid_offset*/,
             std::integral_constant<int, i>,
             std::true_type /*is_last*/)
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
      return flows::concat(
               m_compose.template get_skeleton<0>().in_port(lid_offset),
               tuple_ops::pop_front(
                  flows::in_flows<flows::get_back_modifier>(
                    m_compose, lid_offset)
               ));
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
        inner_ports_types<In, skeletons_size::value - 1>::in_flow(m_compose, in,
                                                                  lid_offset);
      return m_compose.template get_skeleton<skeletons_size::value - 1>().
               out_port(
                 inner_flow,
                 m_compose.template lid_offset<skeletons_size::value - 1>(
                   lid_offset)
               );
    }
  };
};


namespace last_input_to_all_inner_ports {

template <typename Compose, typename In, int i>
struct inner_ports_types
{
  using in_flow_type = result_of::concat<
    typename inner_ports_types<Compose, In, i - 1>::out_port_type,
    std::tuple<typename tuple_ops::result_of::back<In>::type>>;

  using out_port_type = typename tuple_element<
                          i, typename Compose::skeletons_type
                        >::type::template out_port_type<in_flow_type>::type;

  static in_flow_type
  in_flow(Compose const& compose, In const& in, size_t lid_offset)
  {
    return flows::concat(
             inner_ports_types<
               Compose, In, i - 1>::out_port(compose, in, lid_offset),
             tuple_ops::back(in)
           );
  }

  static out_port_type
  out_port(Compose const& compose, In const& in, size_t lid_offset)
  {
    auto&& inner_flow = in_flow(compose, in, lid_offset);
    return compose.template get_skeleton<i>().out_port(inner_flow, lid_offset);
  }
};

template <typename Compose, typename In>
struct inner_ports_types<Compose, In, 0>
{
  using in_flow_type  = typename tuple_ops::result_of::pop_back<In>::type;
  using out_port_type = typename tuple_element<
                          0, typename Compose::skeletons_type
                        >::type::template out_port_type<in_flow_type>::type;

  static in_flow_type
  in_flow(Compose const& compose, In const& in, size_t lid_offset)
  {
    return tuple_ops::pop_back(in);
  }

  static out_port_type
  out_port(Compose const& compose, In const& in, size_t lid_offset)
  {
    auto&& inner_flow = in_flow(compose, in, lid_offset);
    return compose.template get_skeleton<0>().out_port(inner_flow, lid_offset);
  }
};

} // namespace last_input_to_all_inner_ports


//////////////////////////////////////////////////////////////////////
/// @brief An @c last_input_to_all flow for a @c compose takes the first n-1
/// flows in the given flow tuple given as an input to the @c compose
/// and connects it to the input flow of the first skeleton in the
/// @c compose. It then pipes the output of each skeleton to its next
/// skeleton, and in addition passes the last flow given to the @c compose
/// to every enclosed skeleton.
///
/// @dot
/// digraph last_input_to_all {
///     rankdir = LR;
///     node [shape=record];
///     in [label="In", color="white"];
///     out [label="Out", color="white"];
///     subgraph cluster_last_input_to_all {
///       a0 [label=<S<sub>0</sub>>]
///       a1 [label=<S<sub>1</sub>>]
///       a2 [label="...", color="white"];
///       a3 [label=<S<sub>n</sub>>]
///       label=<Compose<sub>last_input_to_all</sub>>;
///     }
///     a0 -> a1;
///     a1 -> a2;
///     a2 -> a3;
///     a3 -> out;
///     in -> a0 [label="pop_back(in)"];
///     in -> a1 [label="back(in)"];
///     in -> a3 [label="back(in)"]
/// }
/// @enddot
///
/// @ingroup skeletonsFlowsCompose
//////////////////////////////////////////////////////////////////////
struct last_input_to_all
{
  template <typename Compose>
  struct port_types
    : public input_to_all::port_types<Compose>
  {
  private:
    using skeletons_t    = typename Compose::skeletons_type;
    using skeletons_size = typename Compose::skeletons_size;
    using base_t         = input_to_all::port_types<Compose>;
  public:
    static constexpr std::size_t in_port_size = skeletons_size::value;
    using in_port_type   =
      typename flows::result_of::concat<
        typename tuple_ops::result_of::pop_back<
          typename tuple_element_t<0, skeletons_t>::in_port_type
        >::type,
        typename tuple_ops::result_of::pop_front<
          flows::result_of::in_flows<Compose, flows::get_back_modifier>
        >::type
      >;

    template <typename In, int i>
    struct inner_ports_types
    {
      using in_flow_type  = typename last_input_to_all_inner_ports::
                              inner_ports_types<Compose, In, i>::in_flow_type;
      using out_port_type = typename last_input_to_all_inner_ports::
                              inner_ports_types<Compose, In, i>::out_port_type;

      static in_flow_type
      in_flow(Compose const& compose, In const& in, size_t lid_offset)
      {
        return last_input_to_all_inner_ports::inner_ports_types<
          Compose, In, i>::in_flow(compose, in, lid_offset);
      }
    };

    template <typename In>
    struct out_port_type {
      using type = typename inner_ports_types<
                     In,(skeletons_size::value - 1)>::out_port_type;
    };

    explicit port_types(Compose const& compose)
      : base_t(compose)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[0]<sub>in-flow</sub> =
    ///        compose<sub>in-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    typename tuple_ops::result_of::pop_back<In>::type
    in_flow(In const& in, std::size_t, std::integral_constant<int, 0>,
            std::false_type /*is_last*/)
    {
      return tuple_ops::pop_back(in);
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[i]<sub>in-flow</sub> =
    ///        P[i-1]<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In, int i, bool B>
    result_of::concat<
      typename std::enable_if<
        (i > 0), inner_ports_types<In, i-1>
      >::type::out_port_type,
      stapl::tuple<typename tuple_ops::result_of::back<In>::type>
    >
    in_flow(In const& in, std::size_t lid_offset,
            std::integral_constant<int, i>,
            std::integral_constant<bool, B> /*is_last*/)
    {
      auto&& inner_flow =
        inner_ports_types<In, i - 1>::in_flow(this->m_compose, in, lid_offset);

      return flows::concat(
               this->m_compose.template get_skeleton<i-1>().
                 out_port(
                   inner_flow,
                   this->m_compose.template lid_offset<i-1>(lid_offset)),
               stapl::make_tuple(tuple_ops::back(in))
            );
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
      return flows::concat(
               tuple_ops::pop_back(
                 this->m_compose.template get<0>().in_port(lid_offset)),
               tuple_ops::pop_front(
                  flows::template in_flows<flows::get_back_modifier>(
                    this->m_compose, lid_offset)
               ));
    }
  };
};


namespace input_to_last_inner_ports {

template <typename Compose, typename In, int i,
          bool is_last =
           (Compose::skeletons_size::value - 1 == i)>
struct inner_ports_types;

template <typename Compose, typename In>
struct inner_ports_types<Compose, In, 0, false>
{
  using in_flow_type  = In;
  using out_port_type = typename tuple_element<
                          0, typename Compose::skeletons_type
                        >::type::template out_port_type<in_flow_type>::type;

  static in_flow_type
  in_flow(Compose const& compose, In const& in, size_t lid_offset)
  {
    return in;
  }

  static out_port_type
  out_port(Compose const& compose, In const& in, size_t lid_offset)
  {
    auto&& inner_flow = in_flow(compose, in, lid_offset);
    return compose.template get_skeleton<0>().out_port(inner_flow, lid_offset);
  }
};

template <typename Compose, typename In, int i>
struct inner_ports_types<Compose, In, i, false>
{
  using in_flow_type  = typename inner_ports_types<
                          Compose, In, i - 1>::out_port_type;
  using out_port_type = typename tuple_element<
                          i, typename Compose::skeletons_type
                        >::type::template out_port_type<in_flow_type>::type;

  static in_flow_type
  in_flow(Compose const& compose, In const& in, size_t lid_offset)
  {
    return inner_ports_types<Compose, In, i - 1>::out_port(compose,
                                                           in,
                                                           lid_offset);
  }

  static out_port_type
  out_port(Compose const& compose, In const& in, size_t lid_offset)
  {
    auto&& inner_flow = in_flow(compose, in, lid_offset);
    return compose.template get_skeleton<i>().out_port(inner_flow, lid_offset);
  }
};

template <typename Compose, typename In, int i>
struct inner_ports_types<Compose, In, i, true>
{
  using in_flow_type  = flows::result_of::concat<
                          typename inner_ports_types<
                            Compose, In, i - 1>::out_port_type,
                          In>;
  using out_port_type = typename tuple_element<
                          0, typename Compose::skeletons_type
                        >::type::template out_port_type<in_flow_type>::type;

  static in_flow_type
  in_flow(Compose const& compose, In const& in, size_t lid_offset)
  {
    return flows::concat(
             inner_ports_types<Compose, In, i - 1>::out_port(
               compose, in, lid_offset),
             in);
  }

  static out_port_type
  out_port(Compose const& compose, In const& in, size_t lid_offset)
  {
    auto&& inner_flow = in_flow(compose, in, lid_offset);
    return compose.template get_skeleton<0>().out_port(inner_flow, lid_offset);
  }
};

}

//////////////////////////////////////////////////////////////////////
/// @brief A very common flow used in many algorithms is when the last
/// input is forked and passed to the last skeleton in the sequence.
/// Examples of this flow can be seen in @c scan and @c sink.
///
/// @dot
/// digraph input_to_last {
///     rankdir = LR;
///     node [shape=record];
///     in [label="In", color="white"];
///     out [label="Out", color="white"];
///     subgraph cluster_coarse_scan_flow {
///       a0 [label=<P<sub>0</sub>>]
///       a1 [label=<P<sub>1</sub>>]
///       a2 [label="...", color="white"];
///       a3 [label=<P<sub>n</sub>>]
///       label=<Compose<sub>last_input_to_last</sub>>;
///     }
///     a0 -> a1;
///     a1 -> a2;
///     a2 -> a3;
///     a3 -> out;
///     in -> a0;
///     in -> a3 [label="back(in)"];
/// }
/// @enddot
///
/// @ingroup skeletonsFlowsCompose
//////////////////////////////////////////////////////////////////////
struct input_to_last
{
  template <typename Compose>
  struct port_types
  {
  private:
    using skeletons_t    = typename Compose::skeletons_type;
    using skeletons_size = typename Compose::skeletons_size;
  protected:
    Compose const& m_compose;
  public:
    static constexpr std::size_t in_port_size = skeletons_size::value;
    using in_port_type   = flows::result_of::concat<
                             typename tuple_element<0, skeletons_t>::type
                               ::in_port_type,
                             stapl::tuple<
                               typename tuple_ops::result_of::back<
                                 typename tuple_element<
                                   skeletons_size::value - 1, skeletons_t
                                 >::type::in_port_type>::type>>;

    template <typename In, int i>
    struct inner_ports_types
    {
      using in_flow_type  = typename input_to_last_inner_ports::
                              inner_ports_types<Compose, In, i>::in_flow_type;
      using out_port_type = typename input_to_last_inner_ports::
                              inner_ports_types<Compose, In, i>::out_port_type;

      static in_flow_type
      in_flow(Compose const& compose, In const& in, size_t lid_offset)
      {
        return input_to_last_inner_ports::inner_ports_types<
          Compose, In, i>::in_flow(compose, in, lid_offset);
      }
    };

    template <typename In>
    struct out_port_type
    {
      using type = typename inner_ports_types<
                     In, (skeletons_size::value - 1)>::out_port_type;
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
            std::false_type /*!is_last*/)
    {
      return in;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[i]<sub>in-flow</sub> =
    ///        P[i-1]<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In, int i>
    typename std::enable_if<
      (i > 0), inner_ports_types<In, i-1>
    >::type::out_port_type
    in_flow(In const& in, std::size_t lid_offset,
            std::integral_constant<int, i>,
            std::false_type /*!is_last*/)
    {
      auto&& inner_flow =
        inner_ports_types<In, i-1>::in_flow(m_compose, in, lid_offset);

      return m_compose.template get_skeleton<i-1>().
               out_port(
                 inner_flow,
                 m_compose.template lid_offset<i-1>(lid_offset));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[n-1]<sub>in-flow</sub> =
    ///        [P[n-2]<sub>out-flow</sub>,
    ///         compose<sub>in-flow</sub>]
    //////////////////////////////////////////////////////////////////////
    template <typename In, int i>
    result_of::concat<
      typename std::enable_if<
        (i > 0), inner_ports_types<In, i-1>
      >::type::out_port_type,
      In
    >
    in_flow(In const& in, std::size_t lid_offset,
            std::integral_constant<int, i>,
            std::true_type /*is_last*/)
    {
      auto&& inner_flow =
        inner_ports_types<In, i-1>::in_flow(m_compose, in, lid_offset);

      return flows::concat(
               m_compose.template get_skeleton<i-1>().
                 out_port(
                   inner_flow,
                   m_compose.template lid_offset<i-1>(lid_offset)),
               in);
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[i|i==n-2]<sub>out-flow</sub> =
    ///        front(P[i+1]<sub>in-flow</sub>)
    //////////////////////////////////////////////////////////////////////
    template <typename Out, int i>
    stapl::tuple<
      typename tuple_ops::result_of::front<
        typename std::enable_if<
          (i == skeletons_size::value - 2), tuple_element<i+1, skeletons_t>
        >::type::type::in_port_type
      >::type>
    out_flow(Out const&, std::size_t lid_offset,
             std::integral_constant<int, i>,
             std::false_type /*!is_last*/)
    {
        return stapl::make_tuple(
                 tuple_ops::front(
                   m_compose.template get_skeleton<i+1>().in_port(
                     m_compose.template lid_offset<i+1>(lid_offset))));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[i]<sub>out-flow</sub> =
    ///        P[i+1]<sub>in-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out, int i>
    typename std::enable_if<
      (i < skeletons_size::value - 2), tuple_element<i+1, skeletons_t>
    >::type::type::in_port_type
    out_flow(Out const&, std::size_t lid_offset,
             std::integral_constant<int, i>,
             std::false_type /*!is_last*/)
    {
      return m_compose.template get_skeleton<i+1>().in_port(
               m_compose.template lid_offset<i+1>(lid_offset));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief P[last]<sub>out-flow</sub> =
    ///        compose<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out, int i>
    Out
    out_flow(Out const& out, std::size_t /*lid_offset*/,
             std::integral_constant<int, i>,
             std::true_type /*is_last*/)
    {
      return out;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief compose<sub>in-flow</sub> =
    ///        [P[0]<sub>in-flow</sub>,
    ///         back(P[n-1]<sub>in-flow</sub>)]
    //////////////////////////////////////////////////////////////////////
    typename port_types<Compose>::in_port_type
    in_port(Compose const& m_compose, std::size_t lid_offset)
    {
      return flows::concat(
               m_compose.template get_skeleton<0>().in_port(lid_offset),
               stapl::make_tuple(tuple_ops::back(
                  m_compose.template get_skeleton<
                    (skeletons_size::value - 1)>().in_port(
                    m_compose.template lid_offset<
                      (skeletons_size::value - 1)>(lid_offset))))
             );
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief compose<sub>out-flow</sub> =
    ///        P[n-1]<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    typename out_port_type<In>::type
    out_port(In const& in, std::size_t lid_offset)
    {
      return m_compose.template get_skeleton<skeletons_size::value - 1>().
               out_port(
                 in,
                 m_compose.template lid_offset<skeletons_size::value - 1>(
                   lid_offset)
               );
    }
  };
};

} // namespace compose_flows
} // namespace flows
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_COMPOSE_FLOWS_HPP
