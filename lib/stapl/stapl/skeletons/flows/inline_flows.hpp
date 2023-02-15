/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_INLINE_FLOWS_HPP
#define STAPL_SKELETONS_INLINE_FLOWS_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/flows/dontcare_flow.hpp>
#include <stapl/skeletons/flows/flow_helpers.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>

//////////////////////////////////////////////////////////////////////
/// @brief Declare n placeholders for skeleton outputs with a given
///  prefix.
///
/// For example:
/// @code
///    DECLARE_INLINE_PLACEHOLDERS(3, x)
/// @endcode
/// will declare variables x0, x1, and x2.
//////////////////////////////////////////////////////////////////////
#define DECLARE_INLINE_PLACEHOLDERS(n, prefix) \
  BOOST_PP_REPEAT(n, DECLARE_INLINE_PLACEHOLDER, prefix)

//////////////////////////////////////////////////////////////////////
/// @brief Declare n input placeholders with a given prefix.
///
/// For example:
/// @code
///    DECLARE_INLINE_INPUT_PLACEHOLDERS(3, in)
/// @endcode
/// will declare variables in0, in1, and in2.
//////////////////////////////////////////////////////////////////////
#define DECLARE_INLINE_INPUT_PLACEHOLDERS(n, prefix) \
  BOOST_PP_REPEAT(n, DECLARE_INLINE_INPUT_PLACEHOLDER, prefix)

#define DECLARE_INLINE_PLACEHOLDER(z, n, prefix) \
  stapl::skeletons::flows::inline_flows::placeholders::x<n> prefix ## n;

#define DECLARE_INLINE_INPUT_PLACEHOLDER(z, n, prefix) \
  stapl::skeletons::flows::inline_flows::placeholders::input<n> prefix ## n;


namespace stapl {
namespace skeletons {
namespace flows {

namespace inline_flows {

namespace placeholders{
//////////////////////////////////////////////////////////////////////
/// @brief A placeholder for the output of a composed skeleton.
///
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<size_t N> struct x;


//////////////////////////////////////////////////////////////////////
/// @brief A placeholder for the inputs to a compose.
///
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<size_t N> struct input;


} // namespace placeholders


using namespace placeholders;

//////////////////////////////////////////////////////////////////////
/// @brief Holds the placeholder names for the output of a skeleton
///  and its inputs as well as a skeleton value.
///
/// @see ph_list
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<typename Skeleton, typename OutPh, typename InPhs>
struct flow_tuple;


//////////////////////////////////////////////////////////////////////
/// @brief A list of placeholders that can be appended to with the
/// comma operator.
///
/// This exists to simplify specifying multiple inputs.
///
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<typename... Phs>
struct ph_list
{
  template<typename T>
  constexpr ph_list<Phs..., T>
  operator,(T) const
  {
    return { };
  }
};

template<typename Phs>
struct ph_list_to_tuple;

template<typename... Phs>
struct ph_list_to_tuple<ph_list<Phs...>>
{
  using type = tuple<Phs...>;
};


template<typename Skeleton, typename OutPh, typename InPhs>
struct flow_tuple
{
  using output_t   = OutPh;
  using input_t    = InPhs;
  using skeleton_t = typename std::decay<Skeleton>::type;

private:
  Skeleton const& m_skeleton;

public:
  flow_tuple(Skeleton const& s)
    : m_skeleton(s)
  { }

  Skeleton const& get_skeleton() const
  {
    return m_skeleton;
  }

};

template<class Skeleton, class OutPh, class... InPhs>
flow_tuple<Skeleton, OutPh, ph_list<InPhs...>>
operator|(flow_tuple<Skeleton, OutPh, ph_list<>>&& f, ph_list<InPhs...>) {
  return flow_tuple<Skeleton, OutPh, ph_list<InPhs...>> { f.get_skeleton() };
}

template<class Skeleton, class OutPh, class InPh>
flow_tuple<Skeleton, OutPh, ph_list<InPh>>
operator|(flow_tuple<Skeleton, OutPh, ph_list<>>&& f, InPh) {
  return flow_tuple<Skeleton, OutPh, ph_list<InPh>>{ f.get_skeleton() };
}


//////////////////////////////////////////////////////////////////////
/// @brief Holds an output placeholder and a list of its inputs.
///
/// @see inline_flow
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<typename OutPh, typename InPhs>
struct ph_flow
{
  using output_t = OutPh;
  using input_t = InPhs;
};


//////////////////////////////////////////////////////////////////////
/// @brief Converts a tuple of @c flow_tuple types to a tuple of
/// @c ph_flow types.
///
/// @see ph_flow
/// @see flow_tuple
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<typename... FlowTuples>
using ph_flows_tuple_of =
    tuple<ph_flow<typename FlowTuples::output_t,
      typename ph_list_to_tuple<typename FlowTuples::input_t>::type>...>;


namespace placeholders {

template<size_t N>
struct x
{
  static constexpr size_t value = N;

  template<typename Skeleton>
  flow_tuple<Skeleton, x, ph_list<>>
  operator<<(Skeleton s) const
  {
    static_assert(
      is_skeleton<typename std::decay<Skeleton>::type>::value,
     "Flows can only be specified on skeletons or flow tuples");
    return { std::move(s) };
  }

  template<typename Placeholder>
  constexpr ph_list<x,typename std::decay<Placeholder>::type>
  operator,(Placeholder const&) const {
    return {};
  }
};


template<size_t N>
struct input
{
  static constexpr size_t value = N;

  template<typename Placeholder>
  constexpr ph_list<input,typename std::decay<Placeholder>::type>
  operator,(Placeholder const&) const {
    return {};
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A trait to detect if a given type is an input placeholder.
///
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<typename T>
struct is_input
  : std::false_type
{ };


template<size_t N>
struct is_input<input<N>>
  : std::true_type
{ };


} // namespace placeholders

//////////////////////////////////////////////////////////////////////
/// @brief Gets the list of inputs for a given output in a tuple of @c
/// ph_flow.
///
/// @see ph_list
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<typename Ph, typename PhFlows>
struct get_inputs_of
{ };


template<typename Ph, typename InPhs, typename... PhFlows>
struct get_inputs_of<Ph, stapl::tuple<ph_flow<Ph, InPhs>, PhFlows...>>
{
  using type = InPhs;

  static type call(
    stapl::tuple<ph_flow<Ph, InPhs>, PhFlows...> const& phflows)
  {
    return get<0>(phflows).get_input();
  }
};


template<typename Ph, typename PhFlow, typename... PhFlows>
struct get_inputs_of<Ph, stapl::tuple<PhFlow, PhFlows...>>
 : get_inputs_of<Ph, stapl::tuple<PhFlows...>>
{ };


template<typename Ph, typename PhFlows>
using get_inputs_of_t = typename get_inputs_of<Ph, PhFlows>::type;

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to get_index_of_ph that checks if the given
/// type uses Ph as its output placeholder.
//////////////////////////////////////////////////////////////////////
template<typename PhFlow, typename Ph>
using writes_to_ph = std::is_same<typename PhFlow::output_t, Ph>;

//////////////////////////////////////////////////////////////////////
/// @brief Returns the index of a given output in a tuple of @c
/// ph_flow.
///
/// This maps a placeholder to a its skeleton in a compose, for example.
///
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<typename Ph, typename PhFlows, size_t N = 0>
using get_index_of_ph =
  stapl::find_first_index<PhFlows, writes_to_ph, Ph>;

//////////////////////////////////////////////////////////////////////
/// @brief Changes the type of a tuple at a given index.
///
///
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<size_t N, typename Tuple, typename T,
         typename Indices = make_index_sequence<
                              stapl::tuple_size<Tuple>::value>>
struct change_tuple_element
{ };

template<typename T, typename U>
struct change_tuple_element<0, T, stapl::tuple<U>,
                            index_sequence<0>>
{
  using type = stapl::tuple<T>;
};


template<size_t N, typename... Ts, typename T, std::size_t... Is>
struct change_tuple_element<N, stapl::tuple<Ts...>, T,
                            stapl::index_sequence<Is...>>
{
  using type = stapl::tuple<typename std::conditional<Is == N, T, Ts>::type...>;
};


template<size_t N, typename Tuple, typename T>
using change_tuple_element_t = typename change_tuple_element<N,Tuple,T>::type;


namespace detail{
template<typename Key, typename Map, int I>
struct index_of_key_impl
 : std::integral_constant<int, -1>
{ };


template<typename Key, typename Value, typename... Entries, int I>
struct index_of_key_impl<Key, tuple<std::pair<Key,Value>, Entries...>, I>
  : std::integral_constant<int, I>
{ };

template<typename Key, typename Entry, typename... Entries, int I>
struct index_of_key_impl<Key, tuple<Entry, Entries...>, I>
  : index_of_key_impl<Key, tuple<Entries...>, I+1>
{ };

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Computes the index of a key type in a map (a tuple of pairs)
/// or -1 if not found.
///
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Map>
using index_of_key = detail::index_of_key_impl<Key, Map, 0>;

//////////////////////////////////////////////////////////////////////
/// @brief Computes the value of a key type in a map (a tuple of pairs).
///
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Map>
struct value_of_key
{ };


template<typename Key, typename Value, typename... Entries>
struct value_of_key<Key, tuple<std::pair<Key,Value>, Entries...>>
{
  using type  = Value;
};


template<typename Key, typename Entry, typename... Entries>
struct value_of_key<Key, tuple<Entry, Entries...>>
  : value_of_key<Key, tuple<Entries...>>
{ };


template<typename Key, typename Map>
using value_of_key_t = typename value_of_key<Key, Map>::type;

//////////////////////////////////////////////////////////////////////
/// @brief Appends a value into a map (a tuple of pairs) at the given
/// index or inserts a new key-value pair if index == -1.
///
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<int I, typename Value, typename Key, typename Map>
struct insert_into_multimap_at
{
  using old_entry = typename tuple_element<I,Map>::type;
  using new_entry = std::pair<Key,
    typename stapl::result_of::tuple_cat<typename old_entry::second_type,
    tuple<Value>>::type>;
  using type = change_tuple_element_t<I, Map, new_entry>;
};


template<typename Value, typename Key, typename Map>
struct insert_into_multimap_at<-1,Value,Key,Map>
{
  using type = typename stapl::result_of::
    tuple_cat<Map, tuple<std::pair<Key, tuple<Value>>> >::type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Inserts a key-value pair or appends to the value if the key
/// already exists in the map.
///
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Value, typename Map>
struct insert_into_multimap
{
  using type = typename insert_into_multimap_at<index_of_key<Key, Map>::value,
                                                Value, Key, Map>::type;
};

namespace detail{
//////////////////////////////////////////////////////////////////////
/// @brief Creates a map of placeholders to the list (tuple) of the
///  skeleton placeholders that read from them and at which index.
///
/// The user specifies only what flows each skeleton takes at which
/// index, but where the output of a skeleton and at what indices is
/// also needed for a flows class, which is computed here.
///
/// For exmaple, if we have x2 reads from [x0, x1], x0 will have an
/// entry (x2,0) and x1 will have an entry (x2, 1).
///
/// The following is a pseudocode explanation of the process.
/// @code
/// outputs_map = [];
///
/// for (OutPh, InPhs...) in PhFlows
///   int index = 0;
///   for InPh in InPhs...
///     outputs_map[InPh].push_back( (OutPh, index) );
///     ++index;
///   done
/// done
///
/// return outputs_map;
/// @endcode
//////////////////////////////////////////////////////////////////////
template<typename Accum, typename PhFlows, int I = 0>
struct compute_outputs_impl
{
  using type = Accum;
};


//end of inner for loop, reset index = 0 and iterate over next pair
template<typename Accum, typename OutPh, typename... PhFlows, int I>
struct compute_outputs_impl<Accum,
                            tuple<ph_flow<OutPh,tuple<>>, PhFlows...>, I>
  : compute_outputs_impl<Accum, tuple<PhFlows...>, 0>
{ };

//iteration of inner for loop, insert element and increment index
template<typename Accum, typename OutPh, typename InPh, typename... InPhs,
         typename... PhFlows, int I>
struct compute_outputs_impl<Accum,
        tuple<ph_flow<OutPh, tuple<InPh, InPhs...>>, PhFlows...>, I>
  : compute_outputs_impl<typename insert_into_multimap<InPh,
      std::pair<OutPh, std::integral_constant<int, I>>, Accum>::type,
      tuple<ph_flow<OutPh, tuple<InPhs...>>, PhFlows...>, I + 1>
{ };


} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief compute_outputs_impl
/// @see compute_outputs_impl
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<typename PhFlows>
using compute_outputs_t =
  typename detail::compute_outputs_impl<tuple<>,PhFlows>::type;

//////////////////////////////////////////////////////////////////////
/// @brief Computes a tuple of the input placeholders that are used
/// from the result of @c compute_outputs_t.
///
/// @see compute_outputs_t
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<typename Map, typename Accum = tuple<>>
struct compute_used_inputs
{
  using type = Accum;
};


template<size_t N, typename OutPhs, typename... Entries, typename Accum>
struct compute_used_inputs<stapl::tuple<std::pair<input<N>, OutPhs>,
                                        Entries...>, Accum>
 : compute_used_inputs<tuple<Entries...>,
 typename stapl::result_of::tuple_cat<Accum, tuple<input<N>>>::type>
{ };


template<typename Entry, typename... Entries, typename Accum>
struct compute_used_inputs<stapl::tuple<Entry, Entries...>, Accum>
  : compute_used_inputs<stapl::tuple<Entries...>, Accum>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief A flows class that computes in and out flows based on the
/// tuple of ph_flows parameter. The last skeleton is used as the
/// output.
///
/// @note members beginning with get are type-aliases or metafunctions
/// to differentiate them from helper functions used by the methods
/// out_flow, in_flow, in_port, and out_port.
/// @ingroup skeletonsFlowsInline
//////////////////////////////////////////////////////////////////////
template<typename PhFlows>
struct inline_flow
{
public:

  using ph_flows = PhFlows;

  using outputs_map = compute_outputs_t<PhFlows>;
  using inputs_used = typename compute_used_inputs<outputs_map>::type;

  template<typename Ph>
  using get_outputs_of_t = value_of_key_t<Ph, outputs_map>;

  template<typename Ph>
  using get_ph_index = get_index_of_ph<Ph, PhFlows>;

  template<size_t N>
  using get_ph_of_index_t =
    typename tuple_element<N, PhFlows>::type::output_t;

  template<typename Compose>
  struct port_types
  {
  private:
    using skeletons_t = typename Compose::skeletons_type;
    static constexpr size_t skeleton_size =
      stapl::tuple_size<skeletons_t>::value;

    template<size_t N>
    using get_skeleton_t = typename stapl::tuple_element<N,skeletons_t>::type;
    template<typename Ph>
    using get_skeleton_of_ph_t = get_skeleton_t<get_ph_index<Ph>::value>;

    Compose const& m_compose;

  public:

    template<typename In>
    struct inner_port_types
    {
    public:
      template<typename Placeholder>
      struct get_out_flow_of;

    private:
      template<typename InPhs>
      struct get_inflow_from_inputs
      { };

      template<typename... InPhs>
      struct get_inflow_from_inputs<tuple<InPhs...>>
      {
        using type = flows::result_of::concat<
          typename get_out_flow_of<InPhs>::type...>;

        static type
        call(Compose const& compose, In const& in, size_t lid_offset)
        {
          return flows::concat(
                   get_out_flow_of<InPhs>::call(compose, in, lid_offset)...);
        }
      };

    public:
      template<typename Ph>
      struct get_in_flow_of
      {
        using type =
          typename get_inflow_from_inputs<get_inputs_of_t<Ph, PhFlows>>::type;

        static type
        call(Compose const& compose, In const& in, size_t lid_offset)
        {
          return get_inflow_from_inputs<get_inputs_of_t<Ph, PhFlows>>::call(
                   compose, in, lid_offset);
        }

      };

      template<typename Ph>
      struct get_out_flow_of
      {

        static constexpr size_t idx = get_ph_index<Ph>::value;

        using type =
          typename tuple_element<idx, skeletons_t>::type
            ::template out_port_type<typename get_in_flow_of<Ph>::type>::type;

        static type
        call(Compose const& compose, In const& in, size_t lid_offset)
        {
          auto&& inner_flow = get_in_flow_of<Ph>::call(compose, in, lid_offset);

          return compose.template get_skeleton<idx>().out_port(
                   inner_flow, lid_offset);
        }
      };

      template<size_t N>
      struct get_out_flow_of<input<N>>
      {
        using type = stapl::tuple<typename tuple_element<N, In>::type>;

        static type
        call(Compose const& compose, In const& in, size_t lid_offset)
        {
          return stapl::make_tuple(get<N>(in)) ;
        }
      };
    }; //inner_port_types


    //assume last skeleton is output
    template<typename In>
    struct out_port_type
    : inner_port_types<In>::template get_out_flow_of<
      typename tuple_element<skeleton_size - 1, PhFlows>::type::output_t>
    { };


    //in_port_type needed
    port_types(Compose const& compose)
      : m_compose(compose)
    { }

  private:
    //Tells if anything reads from a placeholder internally
    template<typename Ph>
    using ph_has_out_flow =
      std::integral_constant<bool, index_of_key<Ph, outputs_map>::value != -1>;

    template<size_t N>
    using index_has_out_flow = ph_has_out_flow<get_ph_of_index_t<N>>;

    template<typename IndexedOutputs>
    struct get_out_flow_from_indexed_outputs
    { };


    //Gets the output bundle by grabbing single flows from inflows it goes to
    template<typename... IndexedOutputs>
    struct get_out_flow_from_indexed_outputs<tuple<IndexedOutputs...>>
    {
      using type = stapl::tuple<
                     typename stapl::tuple_element<
                      IndexedOutputs::second_type::value,
                       typename get_skeleton_of_ph_t<
                         typename IndexedOutputs::first_type>::in_port_type
                     >::type...>;
    };

  private:
    template<typename Ph>
    using get_out_flow_of_ph_t =
      typename get_out_flow_from_indexed_outputs<get_outputs_of_t<Ph>>::type;

    template<size_t N>
    using get_out_flow_of_t = get_out_flow_of_ph_t<get_ph_of_index_t<N>>;

    //Rebundles individual flows from other bundles
    template<int I, typename... IndexedOutputs>
    get_out_flow_of_t<static_cast<size_t>(I)>
    indexed_outputs_to_out_flow(size_t lid_offset, tuple<IndexedOutputs...>)
    {
      return stapl::make_tuple(stapl::get<IndexedOutputs::second_type::value>(
                 m_compose.template get_in_port<
                   get_ph_index<typename IndexedOutputs::first_type>::value
                 >(lid_offset))...);
    };

     //ReturnType parameter needed for Intel when computing in_port_type
    template<typename ReturnType, typename... IndexedOutputs>
    ReturnType
    indexed_outputs_to_out_flow(size_t lid_offset, tuple<IndexedOutputs...>)
    {
      return ReturnType{ stapl::get<IndexedOutputs::second_type::value>(
                  m_compose.template get_in_port<
                    get_ph_index<typename IndexedOutputs::first_type>::value
               >(lid_offset))...};
    };

    template<typename Phs>
    struct get_combined_out_flow_of_ph
    { };


    template<typename... Phs>
    struct get_combined_out_flow_of_ph<tuple<Phs...>>
    {
      using type = flows::result_of::concat<get_out_flow_of_ph_t<Phs>...>;
    };


    template<typename In, typename Ph>
    using get_in_flow_of_ph_t =
      typename inner_port_types<In>::template get_in_flow_of<Ph>::type;

    template<typename In, size_t N>
    using get_in_flow_of_t = get_in_flow_of_ph_t<In, get_ph_of_index_t<N>>;

    //get an out flow in the flow bundle to rebundle in in_flow_for_inputs
    template<typename In, size_t N>
    stapl::tuple<typename tuple_element<static_cast<int>(N), In>::type>
    single_out_flow(In const& in, size_t lid_offset, input<N>, std::true_type)
    {
      return
      stapl::tuple<typename tuple_element<static_cast<int>(N), In>::type>
      { std::get<static_cast<int>(N)>(in) };
    }

    template<typename In, typename Ph>
    typename inner_port_types<In>::template get_out_flow_of<Ph>::type
    single_out_flow(In const& in, size_t lid_offset, Ph, std::false_type)
    {
      get_in_flow_of_ph_t<In, Ph> inner_flow =
        inner_port_types<In>::template get_in_flow_of<Ph>::call(
          m_compose, in, lid_offset);

      return m_compose.template get_out_port<
        get_in_flow_of_ph_t<In, Ph>,
        get_ph_index<Ph>::value
      >(inner_flow, lid_offset);
    };

    //Gets the input bundle from the list of input placeholders
    template<typename In, int I, typename... InPhs>
    get_in_flow_of_t<In, static_cast<size_t>(I)>
    in_flow_from_inputs(In const& in, size_t lid_offset, tuple<InPhs...>)
    {
      return flows::concat(
               this->single_out_flow(in, lid_offset, InPhs(),
                 is_input<InPhs>())...
      );
    }

    //Returns the out flow of a skeleton or a dontcare_flow if not used
    template<typename Out, int I, typename Last>
    get_out_flow_of_t<I>
    out_flow_if_used(Out const& out, std::size_t lid_offset,
                    std::integral_constant<int, I>, Last, std::true_type)
    {
      return indexed_outputs_to_out_flow<I>(lid_offset,
        get_outputs_of_t<get_ph_of_index_t<I>>());
    }

    template<typename Out, typename Id>
    stapl::tuple<flows::dontcare_flow>
    out_flow_if_used(Out const&, size_t, Id, std::false_type, std::false_type)
    {
      return { };
    }

  public:
    using in_port_type =
      typename get_combined_out_flow_of_ph<inputs_used>::type;

    static constexpr std::size_t in_port_size = tuple_size<in_port_type>::value;

    template<typename Out, typename Id>
    Out
    out_flow(Out const& out, size_t, Id, std::true_type)
    {
      return out;
    }

    template<typename Out, int I, typename Last>
    auto
    out_flow(Out const& out, std::size_t lid_offset,
      std::integral_constant<int, I> ic, Last last)
    STAPL_AUTO_RETURN((
      this->out_flow_if_used(out, lid_offset, ic, last, index_has_out_flow<I>())
    ));

    template<typename In, int I, typename Last>
    get_in_flow_of_t<In, I>
    in_flow(In const& out, std::size_t lid_offset,
      std::integral_constant<int, I>, Last)
    {
      return this->in_flow_from_inputs<In, I>(
        out, lid_offset, typename tuple_element<I, PhFlows>::type::input_t()
      );
    }

    template<typename In>
    typename out_port_type<In>::type
    out_port(In const& in, std::size_t lid_offset)
    {
      constexpr size_t last_index = skeleton_size - 1;

      lid_offset = m_compose.template lid_offset<last_index>(lid_offset);

      auto&& inner_flow =
        inner_port_types<In>::template get_in_flow_of<
          typename tuple_element<last_index, PhFlows>::type::output_t
        >::call(m_compose, in, lid_offset);

      return m_compose.template get_skeleton<last_index>().out_port(inner_flow,
                                                                    lid_offset);
    }

  private:

    template<typename... InPhs>
    in_port_type
    in_port_from_used(std::size_t lid, stapl::tuple<InPhs...>)
    {
      //get the (OutPh, index) pairs for all inputs
      using outputs =
        typename stapl::result_of::tuple_cat<get_outputs_of_t<InPhs>...>::type;
      return indexed_outputs_to_out_flow<in_port_type>(lid, outputs());
    }

  public:
    in_port_type
    in_port(std::size_t lid_offset)
    {
      return in_port_from_used(lid_offset, inputs_used());
    }


  }; //port_types

};


} // namespace inline_flows
} // namespace flows
} // namespace skeletons
} // namespace stapl

#endif
