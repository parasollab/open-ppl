#ifndef STAPL_SKELETONS_OPERATORS_DEFINE_DAG_HPP
#define STAPL_SKELETONS_OPERATORS_DEFINE_DAG_HPP

#include <utility>
#include <stapl/utility/utility.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/push_back.hpp>
#include <stapl/utility/tuple/fold_type.hpp>
#include <stapl/skeletons/flows/inline_flows.hpp>

#include <stapl/utility/tuple/homogeneous_tuple.hpp>
#include <stapl/utility/pack_ops.hpp>
#include <stapl/utility/tuple/fold.hpp>
#include <stapl/skeletons/operators/compose_impl.hpp>
#include <stapl/utility/tuple/filter_type.hpp>
#include <stapl/utility/tuple/safe_tuple_element.hpp>
#include <stapl/utility/not_same.hpp>


namespace stapl {
namespace skeletons {

using flows::inline_flows::inline_flow;
using flows::inline_flows::placeholders::input;
using flows::inline_flows::placeholders::x;

//////////////////////////////////////////////////////////////////////
/// @brief Base type for all define_dag nodes.
//////////////////////////////////////////////////////////////////////
struct define_dag_base
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Get the number of outputs of a skeleton or define_dag node.
///
/// @return 1 if C is a skeleton or C::out_port_size if a define_dag node.
//////////////////////////////////////////////////////////////////////
template<class C>
constexpr typename std::enable_if<
  std::is_base_of<define_dag_base, C>::value, size_t>::type
get_out_port_size() {
  return C::out_port_size;
}

template<class C>
constexpr typename std::enable_if<
  not std::is_base_of<define_dag_base, C>::value, size_t>::type
get_out_port_size() {
  return 1;
}

// Represents the "identity skeleton"
struct id_t
  : define_dag_base
{

  // note: id_t can be used to forward things with more than 1 in/out
  // this is just for computing the size when id_t is the first in a ser_t
  static constexpr size_t in_port_size = 1;
  static constexpr size_t out_port_size = 1;
};

// Represents the serial composition of some skeletons
template<class... Ts>
struct ser_t
  : define_dag_base
{
  using skeletons_t = tuple<Ts...>;

  static constexpr size_t num_skels = sizeof...(Ts);

  static_assert(num_skels > 0, "Need to have at least one skeleton");

private:
  using A = typename tuple_element<0, skeletons_t>::type;
  using B = typename tuple_element<num_skels - 1, skeletons_t>::type;
  skeletons_t skeletons;

public:
  static constexpr size_t in_port_size = A::in_port_size;
  static constexpr size_t out_port_size = get_out_port_size<B>();

  ser_t(skeletons_t const& skel)
    : skeletons(skel)
  { }

  ser_t(Ts... ts)
    : skeletons(ts...)
  { }

  skeletons_t const& get_skeletons() const
  {
    return skeletons;
  }
};

// Represents the parallel (independent) execution of some skeletons
template<class... Ts>
struct par_t
  : define_dag_base
{
  using skeletons_t = tuple<Ts...>;

  static constexpr size_t num_skels = sizeof...(Ts);

  static_assert(num_skels > 0, "Need to have at least one skeleton");

  static constexpr size_t in_port_size = num_skels;
  static constexpr size_t out_port_size = num_skels;

private:
  skeletons_t skeletons;

public:
  par_t(skeletons_t const& skel)
    : skeletons(skel)
  { }

  par_t(Ts... ts)
    : skeletons(ts...)
  { }

  skeletons_t const& get_skeletons() const
  {
    return skeletons;
  }
};

// todo: exact same code again as ser_t, and par_t
// Represents the execution of skeletons where every skeleton gets all the
// inputs and all the outputs of those that came before it
template<class... Ts>
struct shift_t
  : define_dag_base
{
  using skeletons_t = tuple<Ts...>;

  static constexpr size_t num_skels = sizeof...(Ts);

  static_assert(num_skels > 0, "Need to have at least one skeleton");

private:
  using A = typename tuple_element<0, skeletons_t>::type;
  using B = typename tuple_element<num_skels - 1, skeletons_t>::type;
  skeletons_t skeletons;

public:
  static constexpr size_t in_port_size = A::in_port_size;
  static constexpr size_t out_port_size = B::out_port_size +
                                          in_port_size + sizeof...(Ts) - 1;

  shift_t(skeletons_t const& skel)
    : skeletons(skel)
  { }

  shift_t(Ts... ts)
    : skeletons(ts...)
  { }

  skeletons_t const& get_skeletons() const
  {
    return skeletons;
  }
};


// Relabels the outputs of a skeleton
template<class Stuple, int... Is>
struct relabel
{
  static constexpr size_t in_port_size = Stuple::in_port_size;
  static constexpr size_t out_port_size = sizeof...(Is);

  static_assert(out_port_size > 0, "Need to have at least one output");

  relabel(Stuple const& s)
    : stuple(s)
  { }

  Stuple const& get_skeleton() const
  { return stuple; }

private:
  Stuple stuple;

};

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Emit inline flows for a skeleton, given a current state
///
/// @tparam S            the current skeleton to emit inline flows for
/// @tparam Ins          the tuple of input placeholders/ph's
/// @tparam X            the current x<n> to use for an emitted flow
/// @tparam I            the current in<n> to use for additonal inputs
///
/// @see inline_flow
//////////////////////////////////////////////////////////////////////
template<class S, class Ins, size_t X, size_t I>
struct emit_flow;

//////////////////////////////////////////////////////////////////////
/// @brief Emit inline flows for a skeleton inside a ser_t
///
/// @tparam S            the current skeleton to emit inline flows for
/// @tparam Ins          the tuple of input placeholders/ph's
/// @tparam X            the current x<n> to use for an emitted flow
/// @tparam I            the current in<n> to use for additonal inputs
///
/// @see inline_flow
/// @see ser_t
//////////////////////////////////////////////////////////////////////
template<class S, class Ins, size_t X, size_t I>
struct emit_serial_part;

//////////////////////////////////////////////////////////////////////
/// @brief Represents the current state of the emit flows computation
///
/// @tparam Ins          the tuple of input placeholders/ph's
/// @tparam X            the current x<n> to use for an emitted flow
/// @tparam I            the current in<n> to use for additonal inputs
/// @tparam Flows        tuple of flows emitted so far
///
/// This type is used as the accumulator value for par_fold and ser_fold
/// and as well as the emit_serial_part specializations which must
/// provide the same typedefs and static constants as emit_state.
//////////////////////////////////////////////////////////////////////
template<class Ins, size_t X, size_t I, class Flows>
struct emit_state
{
  static constexpr size_t next_x = X;
  static constexpr size_t next_i = I;
  using outputs = Ins;
  using flows = Flows;
};

//////////////////////////////////////////////////////////////////////
/// @brief The binary operation used for emitting skeletons in a ser_t
/// as a fold.
///
/// @tparam State        the accumulated emit_state
/// @tparam S            the current skeleton to output
///
/// This essentially wraps a call to emit_serial_part to also update the
/// list of outputted flows
//////////////////////////////////////////////////////////////////////
template<class State, class S>
struct ser_fold;

//////////////////////////////////////////////////////////////////////
/// @brief The binary operation used for emitting skeletons in a par_t
/// as a fold.
///
/// @tparam State        the accumulated emit_state
/// @tparam SPh          a pair of a skeleton and its input ph
//////////////////////////////////////////////////////////////////////
template<class State, class SPh>
struct par_fold;

//////////////////////////////////////////////////////////////////////
/// @brief The binary operation used for emitting skeletons in a shift_t
/// as a fold.
///
/// @tparam State        the accumulated emit_state
/// @tparam S            the current skeleton to output
///
/// @note This is the same as ser_fold except it only adds the ouptut
/// of each skeleton to the input of each succesive skeleton.
//////////////////////////////////////////////////////////////////////
template<class State, class S>
struct shift_fold;

// Definitions

// If it's an identity, just skip
template<class Ins, size_t X, size_t I>
struct emit_serial_part<id_t, Ins, X, I>
  : emit_state<Ins, X, I, tuple<>>
{ };

template<class Output, class NewInput, int... Is>
using relabel_outputs =
  tuple<typename safe_tuple_element<Is, Output, NewInput>::type...>;

// Relabel modifies just the output phs
template<class S, int... Is, class Ins, size_t X, size_t I>
struct emit_serial_part<relabel<S,Is...>, Ins, X, I>
{
  using original = emit_serial_part<S, Ins, X, I>;
  using outputs_before = typename original::outputs;
  using new_input_t = input<original::next_i>;
  static constexpr bool uses_new_input =
    pack_ops::functional::or_((Is == -1)...);

  using outputs = relabel_outputs<outputs_before, new_input_t, Is...>;
  using flows = typename original::flows;
  static constexpr size_t next_x = original::next_x;
  static constexpr size_t next_i = original::next_i + (uses_new_input ? 1 : 0);
};

//////////////////////////////////////////////////////////////////////
/// @brief Given an tuple of input phs and parallel skeletons, either
/// zip them together or if there's only one input, use it for all.
///
/// @tparam In           the tuple of input phs
/// @tparam Ss           the pack of parallel skeletons
///
/// @see par_t
//////////////////////////////////////////////////////////////////////
template<class In, class... Ss>
struct spread_to_par;

template<class In, class S0, class S1, class... Ss>
struct spread_to_par<tuple<In>, S0, S1, Ss...>
{
  using type = tuple<std::pair<S0, In>, std::pair<S1, In>, std::pair<Ss,In>...>;
};

template<class... Ins, class... Ss>
struct spread_to_par<tuple<Ins...>, Ss...>
{
  using type = tuple<std::pair<Ss, Ins>...>;
};

template<class... Ss, class Ins, size_t X, size_t I>
struct emit_serial_part<par_t<Ss...>, Ins, X, I>
{
  using init = emit_state<tuple<>, X, I, tuple<>>;
  using chains = typename spread_to_par<Ins, Ss...>::type;
  using type = typename tuple_ops::fold_types<
    par_fold, init, chains>::type;

  using flows = typename type::flows;
  using outputs = typename type::outputs;
  static constexpr size_t next_x = type::next_x;
  static constexpr size_t next_i = type::next_i;
};

template<class... Ss, class Ins, size_t X, size_t I>
struct emit_serial_part<ser_t<Ss...>, Ins, X, I>
  : emit_flow<ser_t<Ss...>, Ins, X, I>::type
{
};

template<class... Ss, class Ins, size_t X, size_t I>
struct emit_serial_part<shift_t<Ss...>, Ins, X, I>
{
  // todo: duplication from emit_flow<ser_t>
  using init = emit_state<Ins, X, I, tuple<>>;
  using type = typename tuple_ops::fold_types<
    shift_fold, init, tuple<Ss...>>::type;

  using flows = typename type::flows;
  using outputs = typename type::outputs;
  static constexpr size_t next_x = type::next_x;
  static constexpr size_t next_i = type::next_i;
};

// todo: this should be a single skeleton
// In the base case, S is just a regular skeleton.
template<class S, class Ins, size_t X, size_t I>
struct emit_serial_part
{
  using output = x<X>;

  static constexpr size_t next_x = X+1;
  static constexpr size_t next_i = I;
  using outputs = tuple<output>;
  using flows = tuple<flows::inline_flows::ph_flow<output, Ins>>;
};

template<class State, class S>
struct ser_fold
{
  using Ins = typename State::outputs;
  using Flows = typename State::flows;
  using part = emit_serial_part<S, Ins, State::next_x, State::next_i>;

  using combined_flows = typename stapl::result_of::tuple_cat<
    typename State::flows,
    typename part::flows
  >::type;
  using type = emit_state<typename part::outputs, part::next_x, part::next_i,
                          combined_flows>;
};

template<class State, class S>
struct shift_fold
{
  using Ins = typename State::outputs;
  using Flows = typename State::flows;
  using part = emit_serial_part<S, Ins, State::next_x, State::next_i>;

  using combined_flows = typename stapl::result_of::tuple_cat<
    typename State::flows,
    typename part::flows
  >::type;

  // todo: only diff from ser_fold is that it appends to Ins
  using outputs = typename stapl::result_of::tuple_cat<
    Ins,
    typename part::outputs
  >::type;

  using type = emit_state<outputs, part::next_x, part::next_i, combined_flows>;
};

template<class State, class S, class Ph>
struct par_fold<State, std::pair<S,Ph>>
{
  using Ins = typename State::outputs;
  using Flows = typename State::flows;

  using this_s =
    typename emit_flow<S, tuple<Ph>, State::next_x, State::next_i>::type;

  using outs = typename stapl::result_of::tuple_cat<
    Ins,
    typename this_s::outputs
  >::type;
  using combined_flows = typename stapl::result_of::tuple_cat<
    Flows,
    typename this_s::flows
  >::type;

  using type = emit_state<outs, this_s::next_x, this_s::next_i, combined_flows>;
};


template<class... Ts, class Ins, size_t X, size_t I>
struct emit_flow<ser_t<Ts...>, Ins, X, I>
{
  using init = emit_state<Ins, X, I, tuple<>>;
  using type = typename tuple_ops::fold_types<
    ser_fold, init, tuple<Ts...>>::type;
};

// todo: not technically true, this is how it handles the other types
// default case, it's a single skeleton
template<class S, class Ins, size_t X, size_t I>
struct emit_flow
{
  using type = emit_serial_part<S, Ins, X, I>;
};

template<class Ints> struct make_inputs_impl;
template<size_t... Is> struct make_inputs_impl<index_sequence<Is...>>
{
  using type = tuple<input<Is>...>;
};

template<size_t N> using make_inputs = typename
  make_inputs_impl<make_index_sequence<N>>::type;

//////////////////////////////////////////////////////////////////////
/// Helper struct to extract the skeletons as a tuple.
//////////////////////////////////////////////////////////////////////
template<class S>
struct rip_skels_impl
{
  using type = tuple<S>;
  static type call(S const& s)
  {
    return type(s);
  }
};

//////////////////////////////////////////////////////////////////////
/// Extract the skeletons as a tuple.
//////////////////////////////////////////////////////////////////////
template<class S>
auto rip_skels(S const& s) STAPL_AUTO_RETURN((
      rip_skels_impl<S>::call(s)
))

// Relabel just forwards to whatever was given
template<class S, int... Is>
struct rip_skels_impl<relabel<S, Is...>>
{
  using recursive = rip_skels_impl<S>;

  using type = typename recursive::type;
  static type call(relabel<S, Is...> const& s)
  {
    return recursive::call(s.get_skeleton());
  }

};

// For ser_t, call tuple_cat on all the recursively extracted skeletons
template<class... Ts>
struct rip_skels_impl<ser_t<Ts...>>
{
  using type = typename stapl::result_of::tuple_cat<
                          typename rip_skels_impl<Ts>::type...>::type;

private:
  template<size_t... Is>
  static type impl(ser_t<Ts...> const& s, index_sequence<Is...>)
  {
    return tuple_cat(rip_skels(get<Is>(s.get_skeletons()))...);
  }
public:
  static type call(ser_t<Ts...> const& s)
  {
    return impl(s, make_index_sequence<sizeof...(Ts)>{});
  }
};
// The par_t case is the exact same as ser_t
template<class... Ts>
struct rip_skels_impl<par_t<Ts...>>
{
  using type = typename stapl::result_of::tuple_cat<
                          typename rip_skels_impl<Ts>::type...>::type;

private:
  template<size_t... Is>
  static type impl(par_t<Ts...> const& s, index_sequence<Is...>)
  {
    return tuple_cat(rip_skels(get<Is>(s.get_skeletons()))...);
  }
public:
  static type call(par_t<Ts...> const& s)
  {
    return impl(s, make_index_sequence<sizeof...(Ts)>{});
  }
};
// todo: more code duplication
// The shift_t case is the exact same as ser_t
template<class... Ts>
struct rip_skels_impl<shift_t<Ts...>>
{
  using type = typename stapl::result_of::tuple_cat<
                          typename rip_skels_impl<Ts>::type...>::type;

private:
  template<size_t... Is>
  static type impl(shift_t<Ts...> const& s, index_sequence<Is...>)
  {
    return tuple_cat(rip_skels(get<Is>(s.get_skeletons()))...);
  }
public:
  static type call(shift_t<Ts...> const& s)
  {
    return impl(s, make_index_sequence<sizeof...(Ts)>{});
  }
};

template<class S>
struct to_skel
{

  using emitter = typename emit_flow<
    S, make_inputs<S::in_port_size>, 0, S::in_port_size>::type;
  using flows = typename emitter::flows;
  // todo: when last is relabel and last(outputs) != last(flow).output,
  // we will need inline_flows to allow using an intermediate ph as the output

  using rip = rip_skels_impl<S>;
  using flow = inline_flow<flows>;

  using remove_ids =
    tuple_ops::filter_types<typename rip::type, not_same, id_t>;
  using skeletons = typename remove_ids::type;

  using skeleton_t = stapl::skeletons::skeletons_impl::compose<skeletons, flow>;

  static skeleton_t call(S const& s)
  {
    return skeleton_t{ remove_ids::call(rip::call(s)) };
  }

};

} // namespace detail

template<class S>
typename detail::to_skel<S>::skeleton_t to_skeleton(S const& s)
{
  return detail::to_skel<S>::call(s);
}

template<class... S>
ser_t<S...> ser(S const&... s)
{
  return {s...};
}
template<class... S>
par_t<S...> par(S const&... s)
{
  return {s...};
}

template<class... S>
shift_t<S...> shift(S const&... s)
{
  return {s...};
}

template<size_t... Is, class S>
relabel<S, Is...> rel(S const& s)
{
  return {s};
}

template<size_t... Is, class S>
ser_t<relabel<id_t, Is...>,S> take(S const& s)
{
  return ser(rel<Is...>(id_t{}), s);
}

namespace detail
{
  // note: in_port_size does not reflect such inputs
  template<class S, class>
  struct add_input;
  template<class S, size_t... Is>
  struct add_input<S, index_sequence<Is...>>
  {
    using type = relabel<S, int{Is}..., -1>;
  };
  template<class S>
  using add_input_t = typename
    add_input<S, make_index_sequence<S::out_port_size>>::type;

  template<size_t I, size_t N, class S, class = make_index_sequence<N>>
  struct on;

  template<size_t I, size_t N, class S, size_t... Is>
  struct on<I, N, S, index_sequence<Is...>>
  {
    using type = par_t<typename std::conditional<Is == I, S, id_t>::type...>;

    static type call(S const& s)
    {
      return type{pack_ops::get<I == Is ? 0 : 1>(s, id_t{})...};
    }
  };


} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Serially compose two skeletons and give the second a unique
/// additional input.
///
/// @param s             the first skeleton
/// @param t             the second skeleton which reads all the
///   outputs from s and an additonal input
///
/// The resulting skeleton first computes s and gives t all of s's
/// outputs and an additional input as the last parameter. For example,
/// @code
/// add_input(s,t) = x0 << s | in0, x1 << t | (x0, in1)
/// @endcode
///
/// @see inline_flow
//////////////////////////////////////////////////////////////////////
template<class A, class B>
ser_t<detail::add_input_t<A>, B> add_input(A const& a, B const& b)
{
  return {detail::add_input_t<A>{a}, b};
}

//////////////////////////////////////////////////////////////////////
/// @brief Equivalent to passing the identity skeleton as the first parameter
/// to the binary add_input() function.
///
/// It is related to the binary overload by the following:
/// @code
/// add_input(s,t) = ser(s, add_input(t))
/// @endcode
//////////////////////////////////////////////////////////////////////
template<class A>
auto add_input(A const& a) STAPL_AUTO_RETURN((
  add_input(id_t{}, a)
))

//////////////////////////////////////////////////////////////////////
/// @brief Give the same input to two skeletons and give both of their
/// results as output.
///
/// @note Use this only when there is a single input coming in.
///
///
/// @param as            pack of skeletons to execute in parallel
///
/// This is exactly equivalent to
/// @code
/// ser(id_t{}, par(as...))
/// @endcode
/// Therefore, the input is forwarded to all of the skeletons in the
/// pack.
///
/// @see id_t
/// @see ser_t
/// @see par_t
//////////////////////////////////////////////////////////////////////
template<class... As>
ser_t<id_t, par_t<As...>> branch(As const&... as)
{
  return {id_t{},{as...}};
}

//////////////////////////////////////////////////////////////////////
/// @brief Provides the same input to both skeletons but the second
/// gets the output of the first as well.
//////////////////////////////////////////////////////////////////////
template<class A, class B>
ser_t<A, par_t<id_t, B>> branch_off(A const& a, B const& b)
{
  return {a, {id_t{}, b}};
}

//////////////////////////////////////////////////////////////////////
/// @brief Applies a skeleton on the Ith out of N inputs.
///
/// For example,
/// @code
/// ser(par(a,b,c), on<1,3>(d)) = par(a,ser(b,d),c)
/// @endcode
//////////////////////////////////////////////////////////////////////
template<size_t I, size_t N, class A>
typename detail::on<I,N,A>::type on(A const& a)
{
  return detail::on<I,N,A>::call(a);
}




} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_OPERATORS_DEFINE_DAG_HPP
