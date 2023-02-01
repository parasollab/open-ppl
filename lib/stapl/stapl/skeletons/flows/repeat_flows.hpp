/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_REPEAT_FLOWS_HPP
#define STAPL_SKELETONS_REPEAT_FLOWS_HPP

#include <stapl/skeletons/flows/flow_helpers.hpp>
#include <stapl/skeletons/operators/consumer_count.hpp>

namespace stapl {
namespace skeletons {
namespace flows {
namespace repeat_flows {

//////////////////////////////////////////////////////////////////////
/// @brief A @c piped flow for a @c repeat skeleton, connects the out
/// flow of each iteration to the input of the next iteration, and
/// connects the input of the @c repeat skeleton to the first
/// iteration of the while loop. It also connects the output of the
/// @c repeat skeleton to the output of the last iteration.
///
/// @dot
/// digraph piped {
///     rankdir = LR;
///     node [shape=record];
///     in [label="In", color="white"];
///     out [label="Out", color="white"];
///     subgraph cluster_piped {
///       a0 [label=<Iter<sub>0</sub>>];
///       a1 [label=<Iter<sub>1</sub>>];
///       a2 [label="...", color="white"];
///       a3 [label=<Iter<sub>n</sub>>];
///       label=<Repeat<sub>piped</sub>>;
///     }
///     a0 -> a1;
///     a1 -> a2;
///     a2 -> a3;
///     in -> a0;
///     a3 -> out;
/// }
/// @enddot
///
/// @ingroup skeletonsFlowsRepeat
//////////////////////////////////////////////////////////////////////
struct piped
{
  template <typename Repeat>
  struct port_types
  {
  private:
    using nested_p_type = typename Repeat::nested_p_type;
  protected:
    Repeat const& m_repeat;
  public:
    static constexpr std::size_t in_port_size  = nested_p_type::in_port_size;
    using in_port_type  = typename nested_p_type::in_port_type;

    template <typename In>
    struct out_port_type
    {
      using type = typename nested_p_type::template out_port_type<In>::type;
    };


    explicit port_types(Repeat const& repeat)
      : m_repeat(repeat)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @brief S<b>[single-iter]</b><sub>in-flow</sub> =
    ///        repeat<sub>in-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    In
    in_flow(std::size_t, In const& in, std::size_t, tags::repeat_single_iter)
    {
      return in;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief S<b>[first-iter]</b><sub>in-flow</sub> =
    ///        repeat<sub>in-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    In
    in_flow(std::size_t, In const& in, std::size_t, tags::repeat_first_iter)
    {
      return in;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief S<b>[iter]</b><sub>in-flow</sub> =
    ///        S<b>[iter-1]</b><sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In, typename Tag>
    typename nested_p_type::template out_port_type<In>::type
    in_flow(std::size_t iter_num, In const& in, std::size_t lid_offset, Tag)
    {
      return m_repeat.nested_out_port(in, lid_offset, iter_num-1);
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief S<b>[single-iter]</b><sub>out-flow</sub> =
    ///        repeat<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out>
    Out
    out_flow(std::size_t, Out const& out, std::size_t, tags::repeat_single_iter)
    {
      return out;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief S<b>[iter]</b><sub>out-flow</sub> =
    ///        S<b>[iter+1]</b><sub>in-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out, typename Tag>
    typename nested_p_type::in_port_type
    out_flow(std::size_t iter_num, Out const&, std::size_t lid_offset, Tag)
    {
      return m_repeat.nested_in_port(lid_offset, iter_num+1);
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief S<b>[last-iter]</b><sub>out-flow</sub> =
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
    ///        S<b>[first-iter]</b><sub>in-flow</sub>
    //////////////////////////////////////////////////////////////////////
    typename port_types<Repeat>::in_port_type
    in_port(std::size_t lid_offset)
    {
      return m_repeat.nested_skeleton().in_port(lid_offset);
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief repeat<sub>out-flow</sub> =
    ///        S<b>[last-iter]</b><sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    typename out_port_type<In>::type
    out_port(In const& in, std::size_t lid_offset)
    {
      std::size_t iter_num = m_repeat.dimensions();
      return m_repeat.nested_out_port(in, lid_offset, iter_num-1);
    }
  };
};


//////////////////////////////////////////////////////////////////////
/// @brief The @c output_from_all flow provides the output of each
/// iteration not only to its next iteration but to the skeleton that
/// comes after the repeat. In other words, all the tasks will provide
/// data to the out flow, and the skeleton that comes after the current
/// skeleton will be able to request reads from every element of this
/// skeleton.
///
/// @dot
/// digraph output_from_all {
///     rankdir = LR;
///     node [shape=record];
///     in [label="In", color="white"];
///     out [label="Out", color="white"];
///     subgraph cluster_output_from_all {
///       a0 [label=<Iter<sub>0</sub>>];
///       a1 [label=<Iter<sub>1</sub>>];
///       a2 [label="...", color="white"];
///       a3 [label=<Iter<sub>n</sub>>];
///       label=<Repeat<sub>output_from_all</sub>>;
///     }
///     a0 -> a1;
///     a1 -> a2;
///     a2 -> a3;
///     in -> a0;
///     a0 -> out;
///     a1 -> out;
///     a3 -> out;
/// }
/// @enddot
///
/// @ingroup skeletonsFlowsRepeat
//////////////////////////////////////////////////////////////////////
struct output_from_all
{
  //////////////////////////////////////////////////////////////////////
  /// @brief This is a customized flow over a @c repeat skeleton that
  /// accepts only 2D queries.
  ///
  /// @tparam S the underlying repeat skeleton
  //////////////////////////////////////////////////////////////////////
  template <typename In, typename S>
  class custom_flow
  {
  public:
    using domain_type = typename tuple_element<0, In>::type::domain_type;

  private:
    S           m_skeleton;
    In          m_in;
    std::size_t m_lid_offset;
    domain_type m_domain;
  public:
    using port_t =
      typename
        tuple_element<0, typename S::template out_port_type<In>::type>::type;

    using flow_value_type = typename port_t::flow_value_type;

    custom_flow(S const& skeleton, In const& in, std::size_t lid_offset)
      : m_skeleton(skeleton),
        m_in(in),
        m_lid_offset(lid_offset),
        m_domain(stapl::get<0>(m_in).domain())
    { }

    template <typename F = skeletons::no_filter>
    using producer_type = typename port_t::template producer_type<F>;

    template <typename Index, typename F = skeletons::no_filter>
    producer_type<F>
    consume_from(Index const& index, F const& filter = F()) const
    {
      std::size_t offset = m_lid_offset +
                           (m_skeleton.last_id() * stapl::get<1>(index));
      return stapl::get<0>(m_skeleton.out_port(m_in, offset)).
               consume_from(index, filter);
    }

    domain_type domain(void) const
    {
      return m_domain;
    }
  };


  template <typename Repeat>
  struct port_types
    : public piped::port_types<Repeat>
  {
  private:
    using nested_p_type =  typename Repeat::nested_p_type;
    using base_t        =  piped::port_types<Repeat>;
  public:
    static constexpr std::size_t in_port_size =
      piped::port_types<Repeat>::in_port_size;
    using in_port_type = typename piped::port_types<Repeat>::in_port_type;

    template <typename In>
    struct out_port_type
    {
      using type = tuple<custom_flow<In, nested_p_type>>;
    };

    explicit port_types(Repeat const& repeat)
      : base_t(repeat)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @brief S<b>[last-iter]</b><sub>out-flow</sub> =
    ///        repeat<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out>
    Out
    out_flow(std::size_t, Out const& out, std::size_t, tags::repeat_single_iter)
    {
      return out;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief S<b>[last-iter]</b><sub>out-flow</sub> =
    ///        repeat<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out>
    Out
    out_flow(std::size_t, Out const& out, std::size_t, tags::repeat_last_iter)
    {
      return out;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief S<b>[iter]</b><sub>out-flow</sub> =
    ///        S<b>[iter+1]</b><sub>in-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out, typename Tag>
    flows::result_of::concat<
                typename nested_p_type::in_port_type, Out>
    out_flow(std::size_t iter_num, Out const& out, std::size_t lid_offset, Tag)
    {
      return flows::concat(
               this->m_repeat.nested_in_port(lid_offset, iter_num+1),
               out);
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief repeat<sub>out-flow</sub> =
    ///        cloned-flow[custom_flow<sub>S</sub>,
    ///                    S<sub>flow-size</sub>]
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    typename out_port_type<In>::type
    out_port(In const& in, std::size_t lid_offset)
    {
      return stapl::make_tuple(
               custom_flow<In, nested_p_type>(
                 this->m_repeat.nested_skeleton(), in, lid_offset
               ));
    }
  };
};

namespace flows_impl {

  //////////////////////////////////////////////////////////////////////
  /// @brief This is a customized flow over a an existing flow, that
  /// only converts the @c consumers_count request to 2D requests.
  ///
  /// This custom flow replaces the height coordinate with -1 to
  /// indicate that the request is coming from an input outside of
  /// scan skeleton. This custom flow is used for various scan skeletons
  /// including Hillis-Steele and Blelloch scan skeleton.
  ///
  /// @tparam Flow the underlying flow
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  class custom_flow
    : public F
  {
  public:
    explicit custom_flow(F const& flow)
      : F(flow)
    { }

    template <typename Coord>
    std::size_t consumer_count(Coord const& producer_coord) const
    {
      return skeletons::consumer_count(
        static_cast<F const&>(*this),
        stapl::make_tuple(tuple_ops::front(producer_coord), -1));
    }
  };

} // namespace flows_impl

//////////////////////////////////////////////////////////////////////
/// @brief An @c input_wrapper flow is used in various scan skeletons
/// to convert a 1D input coordinate to a 2D coordinate by appending
/// -1 to the coordinate it receives.
///
/// The value -1 is used later on in scan skeletons to indicate that
/// a @c consumer_count request is coming from the input not within
/// the levels of a scan tree.
///
/// @ingroup skeletonsFlows
//////////////////////////////////////////////////////////////////////
template <typename Flows>
struct input_wrapper
{
  template <typename Skeleton>
  struct port_types
    : public Flows::template port_types<Skeleton>
  {
    struct make_custom_flow
    {
      template <typename Flow>
      flows_impl::custom_flow<Flow>
      operator()(Flow const& flow) const
      {
        return flows_impl::custom_flow<Flow>(flow);
      }
    };

    using base_flow_t   = typename Flows::template port_types<Skeleton>;
    static constexpr std::size_t in_port_size  = base_flow_t::in_port_size;
    using custom_flow_t =
      flows_impl::custom_flow<typename base_flow_t::in_port_type>;
    using in_port_type  = tuple<custom_flow_t>;

    template <typename In>
    struct out_port_type
    {
      using type = typename base_flow_t::template out_port_type<In>::type;
    };

    explicit port_types(Skeleton const& skeleton)
      : base_flow_t(skeleton)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @brief compose<sub>in-flow</sub> =
    ///        [custom_flow<Flow>]
    //////////////////////////////////////////////////////////////////////
    in_port_type
    in_port(std::size_t lid_offset)
    {
      return in_port_type{custom_flow_t{base_flow_t::in_port(lid_offset)}};
    }
  };
};

} // namespace repeat_flows
} // namespace flows
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_REPEAT_FLOWS_HPP
