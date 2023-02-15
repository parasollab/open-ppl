/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_COMPOSE_ZIP_FUSION_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_COMPOSE_ZIP_FUSION_HPP

#include <stapl/skeletons/utility/tags.hpp>

#include <stapl/skeletons/flows/inline_flows.hpp>
#include <stapl/utility/tuple/back.hpp>
#include <stapl/utility/tuple/filter_type.hpp>
#include <stapl/utility/tuple/fold_type.hpp>
#include <stapl/utility/tuple/insert_sorted.hpp>
#include <stapl/utility/tuple/insert_type.hpp>
#include <stapl/utility/tuple/tuple_cat_unique.hpp>
#include <stapl/skeletons/operators/compose_impl.hpp>
#include <stapl/skeletons/utility/filters.hpp>
#include <stapl/skeletons/transformations/compose/fused_wf.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/flows/flow_topsort.hpp>

namespace stapl {
namespace skeletons {
namespace transformations {

template <typename S, typename SkeletonTag, typename CoarseTag>
struct transform;

using flows::inline_flows::inline_flow;

namespace zip_fusion_impl {

template<bool... Bs>
using bool_sequence = integer_sequence<bool, Bs...>;

using tuple_ops::result_of::push_front;
using tuple_ops::result_of::push_back;
using tuple_ops::tuple_contains_type;
using tuple_ops::not_tuple_contains_type;
using tuple_ops::tuple_cat_unique;
using tuple_ops::filter_types;
using tuple_ops::fold_types;
using tuple_ops::insert_sorted_less;
using tuple_ops::insert_type;
using stapl::result_of::tuple_cat;

template<class>
struct is_zip
  : std::false_type
{ };

template<class Tag, int Arity>
struct is_zip<tags::zip<Tag, Arity>>
  : std::true_type
{ };

template<class Tuple>
struct get_firsts;

template<class... Elements>
struct get_firsts<tuple<Elements...>>
{
  using type = tuple<typename Elements::first_type...>;
};

template<class Ph, class Flow, class Set>
struct ph_flows_only_into
  : std::integral_constant<bool,
      find_first_index<
        typename get_firsts<typename Flow::template get_outputs_of_t<Ph>>::type,
        not_tuple_contains_type,
        Set
      >::value == -1>
{ };

template<int N, typename IntValues>
struct safe_sequence_element
  : sequence_element<N, IntValues>
{ };

// If -1, default construct a value
template<typename IntValues>
struct safe_sequence_element<-1, IntValues>
  : std::integral_constant<typename IntValues::value_type,
      // todo(intel): cannot use default ctor for value with icc17
      (typename IntValues::value_type) 0>
{ };

// Get the value in a sequence indexed by a key in a corresponding list, i.e.
// it gives IntValues[i], i = first index for which Comp<Key, Keys[i]> is true
//
template<class Key, template<class, class> class Comp, class Keys,
  class IntValues>
using get_assoc_value = safe_sequence_element<
  find_first_index<Keys, Comp, Key>::value, IntValues>;

using flows::inline_flows::writes_to_ph;

// Return the eleement of Int
template<class Key, class Keys, class IntValues>
using get_ph_assoc_value = get_assoc_value<Key, writes_to_ph, Keys, IntValues>;

// Get all the inputs in a ph_flow that are zips
template<class PhFlow, class IsZips, class PhFlows>
using get_zip_inputs = typename filter_types<typename PhFlow::input_t,
      get_ph_assoc_value, PhFlows, IsZips>::type;

template<class Ph, class PhFlows>
using find_ph_flow_for = typename tuple_element<
        find_first_index<PhFlows, writes_to_ph, Ph>::value, PhFlows>::type;

template<class Flow, class IsZips, class PhFlow, class PhFlows, class Frontier>
using expand_frontier = typename filter_types<
  get_zip_inputs<PhFlow, IsZips, PhFlows>,
  ph_flows_only_into,
  Flow, Frontier>::type;

// Expand a given frontier through iterative calls to expand_frontier
template<class Flow, class IsZips, class PhFlows>
struct maximally_expand
{
  template<class Expanded, class Frontier>
  struct apply;

  // Being called to expand an empty frontier, don't expand
  template<class Ph>
  struct apply<tuple<Ph>, tuple<>>
  {
    using type = Ph;
  };

  // Base case: nothing left to expand, return that
  template<class Expanded>
  struct apply<Expanded, tuple<>>
  {
    using type = Expanded;
  };

  // Expand from T and add the skeletons to Expanded and Frontier
  template<class Expanded, class T, class... Ts>
  struct apply<Expanded, tuple<T, Ts...>>
  {
    using PhFlow = find_ph_flow_for<T, PhFlows>;
    using added = expand_frontier<Flow, IsZips, PhFlow, PhFlows, Expanded>;

    using next = apply<
      typename tuple_cat<added, Expanded>::type,
      typename tuple_cat<tuple<Ts...>, added>::type
    >;
    using type = typename next::type;
  };
};


template<class Flow>
struct fuse_flows
{
  template<class SeenFlows, class SeenPhs>
  struct iterate_fuse
  {
    template<class IsZips, class PhFlows>
    struct apply;

    template<bool Seen, class IsZips, class PhFlow, class PhFlows>
    struct check_already_seen;

    template<class PhFlow, class Exapanded, class IsZips, class PhFlows>
    struct expand_and_go;

    template<class PhFlow>
    using unfused = PhFlow;

    template<class IsZips, class PhFlows, class PhFlow>
    using dont_fuse = typename iterate_fuse<
        typename push_front<SeenFlows, unfused<PhFlow>>::type,
        typename push_back<SeenPhs, typename PhFlow::output_t>::type
    >::template apply<IsZips, PhFlows>;

    template<class IsZips, class PhFlows, class... Fused>
    using do_fuse = typename iterate_fuse<
        typename push_front<SeenFlows, tuple<Fused...>>::type,
        typename tuple_cat<SeenPhs, tuple<typename Fused::output_t...>>::type
    >::template apply<IsZips, PhFlows>;

    // Base case: nothing to do with just one skeleton
    template<class PhFlow>
    struct apply<bool_sequence<false>, tuple<PhFlow>>
      : push_front<SeenFlows, PhFlow>
    {
    };

    // Base case: no more skeletons left to process, yield the computed flows
    // note: can't explicitly specialize
    template<class IsZip>
    struct apply<IsZip, tuple<>>
    {
      using type = SeenFlows;
    };

    template<bool... IsZips, class PhFlow, class... PhFlows>
    struct apply<bool_sequence<true, IsZips...>, tuple<PhFlow, PhFlows...>>
    : check_already_seen<
        tuple_contains_type<typename PhFlow::output_t, SeenPhs>::value,
        bool_sequence<IsZips...>,
        PhFlow, tuple<PhFlows...>>
    {
    };

    // Not fusable, leave it as is and go to next
    template<bool... IsZips, class PhFlow, class... PhFlows>
    struct apply<bool_sequence<false, IsZips...>, tuple<PhFlow, PhFlows...>>
      : dont_fuse<bool_sequence<IsZips...>, tuple<PhFlows...>, PhFlow>
    {
    };

    // Already fused in somewhere so move to next
    template<class IsZips, class PhFlow, class PhFlows>
    struct check_already_seen<true, IsZips, PhFlow, PhFlows>
      : apply<IsZips, PhFlows>
    {
    };


    // A new zip skeleton was found, so expand from it
    template<class IsZips, class PhFlow, class PhFlows>
    struct check_already_seen<false, IsZips, PhFlow, PhFlows>
    {
      using this_ph = tuple<typename PhFlow::output_t>;
      // Find those inputs that only flow into this zip
      using fusable_phs =
            expand_frontier<Flow, IsZips, PhFlow, PhFlows, this_ph>;

      using expand_t = typename
        maximally_expand<Flow, IsZips, PhFlows>::template
             apply<typename tuple_cat<fusable_phs, this_ph>::type, fusable_phs>;
      using expanded = typename expand_t::type;

      using next = expand_and_go<PhFlow, expanded, IsZips, PhFlows>;
      using type = typename next::type;

    };

    // If there is nothing on the frontier, leave it alone and proceed
    template<class PhFlow, class Ph, class IsZips, class PhFlows>
    struct expand_and_go
      : dont_fuse<IsZips, PhFlows, PhFlow>
    {
    };

    template<class PhFlow, class... Fs, class IsZips, class PhFlows>
    struct expand_and_go<PhFlow, tuple<Fs...>, IsZips, PhFlows>
      : do_fuse<IsZips, PhFlows, find_ph_flow_for<Fs,
        typename push_back<PhFlows, PhFlow>::type>...>
    {
    };

  };

};

template<class IsZips, class Flow>
struct fuse_zips
  : fuse_flows<Flow>::template iterate_fuse<tuple<>, tuple<>> ::template apply<
     IsZips, typename stapl::result_of::reverse<typename Flow::ph_flows>::type>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Transform a compose based on the computed clustering.
//////////////////////////////////////////////////////////////////////
template<class FusedPhs, class Flow>
struct make_fused;

//////////////////////////////////////////////////////////////////////
/// @brief Transform an individual cluster if needed.
//////////////////////////////////////////////////////////////////////
template<class PhFlow, class Flow>
struct make_fused_part
{
private:
  static constexpr size_t index =
    Flow::template get_ph_index<typename PhFlow::output_t>::value;

public:
  using flow = PhFlow;

  template<class Skeletons>
  using skeleton_t = typename tuple_element<index,
                      typename std::decay<Skeletons>::type>::type;

  template<class Skeletons>
  static skeleton_t<Skeletons> get_skeleton(Skeletons&& skels)
  {
    return get<index>(std::forward<Skeletons>(skels));
  }
};


template<class Filter, class Op>
struct filtered_wf
{
private:
  Filter m_filter;
  Op     m_op;

public:
  filtered_wf(Filter filter, Op op) :
    m_filter(std::move(filter)),
    m_op(std::move(op))
  { }

  template<class... Ts>
  auto operator()(Ts&&... ts) STAPL_AUTO_RETURN((
    m_op(filters::apply_filter(m_filter, std::forward<Ts>(ts))...)
  ))

  void define_type(typer& t)
  {
    t.member(m_filter);
    t.member(m_op);
  }
};

template<class Filter, class Op>
filtered_wf<typename std::decay<Filter>::type, typename std::decay<Op>::type>
make_filtered_wf(Filter&& f, Op&& op)
{
  return {std::forward<Filter>(f), std::forward<Op>(op)};
}

// note: in zip_pd.hpp, get_filter() only returns the first from a homogenous
// tuple of filters. If this is changed and each filter can be different, we
// will need to pass all of them to make_filtered_wf()
template<class S>
auto get_filtered_op(S const& s) STAPL_AUTO_RETURN ((
  make_filtered_wf(s.get_filter(), s.get_op())
))


template<class... PhFlows, class Flow>
struct make_fused_part<tuple<PhFlows...>, Flow>
{
private:
  using last = typename tuple_ops::result_of::back<tuple<PhFlows...>>::type;
  using all_inputs =
    typename tuple_cat_unique<typename PhFlows::input_t...>::type;
  using sorted_inputs = typename fold_types<insert_sorted_less, tuple<>,
      all_inputs>::type;

  using phs = tuple<typename PhFlows::output_t...>;

  using inputs = typename tuple_ops::result_of::difference<
                  sorted_inputs, phs, tuple_ops::tuple_contains_type>::type;

  using all_phs = typename tuple_cat<inputs, phs>::type;

  using deps = tuple<typename find_first_indices_seq<
          all_phs, typename PhFlows::input_t, std::is_same>::type...>;

  template<class Skeletons, class Ph>
  using skel_for_ph = typename tuple_element<
                        Flow::template get_ph_index<Ph>::value,
                        Skeletons>::type;

  template<class Skeletons>
  using wf_for = decltype(make_fused_wf<deps>(get_filtered_op(
      std::declval<skel_for_ph<Skeletons, typename PhFlows::output_t>>())...));

public:
  using flow = flows::inline_flows::ph_flow<typename last::output_t, inputs>;

  template<class Skeletons>
  using skeleton_t = result_of::zip<tuple_size<inputs>::value,
                      wf_for<Skeletons>,
                      skeletons_impl::default_skeleton_traits>;

  template<class Skeletons>
  static skeleton_t<typename std::decay<Skeletons>::type>
  get_skeleton(Skeletons&& skels)
  {
    return zip<tuple_size<inputs>::value>(make_fused_wf<deps>(get_filtered_op(
      get<Flow::template get_ph_index<typename PhFlows::output_t>::value>(
        skels))...));
  }
};

template<class... FusedPhs, class Flow>
struct make_fused<tuple<FusedPhs...>, Flow>
{
private:
  using flows = tuple<typename make_fused_part<FusedPhs, Flow>::flow...>;
  using flow = stapl::skeletons::flows::inline_flows::inline_flow<flows>;

  template<class Skeletons>
  using skeleton_tuple = tuple<typename make_fused_part<
                          FusedPhs, Flow>::template skeleton_t<
                            typename std::decay<Skeletons>::type>...>;

  template<class Skeletons>
  static skeleton_tuple<Skeletons> get_skeleton_tuple(Skeletons&& skels)
  {
    return make_tuple(make_fused_part<FusedPhs, Flow>::get_skeleton(skels)...);
  }
public:

  template<class Skeletons>
  using new_skeleton_t = skeletons_impl::compose<
    skeleton_tuple<Skeletons>, flow>;

  template<class Skeletons>
  static new_skeleton_t<Skeletons> get_skeleton(Skeletons&& skels)
  {
    return new_skeleton_t<Skeletons>{ get_skeleton_tuple(skels) };
  }
};

// Edge case: everything is fused into one zip
template<class FusedPhFlow, class Flow>
struct make_fused<tuple<FusedPhFlow>, Flow>
{
  template<class Skeletons>
  static auto get_skeleton(Skeletons&& skels)
  STAPL_AUTO_RETURN((
    make_fused_part<FusedPhFlow, Flow>::get_skeleton(skels)
  ))

  template<class Skeletons>
  using new_skeleton_t = decltype(get_skeleton(std::declval<Skeletons>()));
};


// Return a reversed bool sequence of whether each element in the tuple is a zip
template<
  typename Tuple,
  typename IdxList = make_index_sequence<tuple_size<Tuple>::value>>
struct rev_is_zip;

template<typename Tuple, std::size_t... Indices>
struct rev_is_zip<Tuple, index_sequence<Indices...>>
{
   using type = integer_sequence<bool,
     is_zip<
       typename tuple_element<sizeof...(Indices) - 1 - Indices,Tuple>::type
     >::value...
   >;
};

} // namespace zip_fusion_impl

//////////////////////////////////////////////////////////////////////
/// @brief Inspect a inline_flow compose skeleton and fuse candidate
///        sub-DAGs of zips into single zips.
///
/// @note this assumes that skeletons are provided in a topological
///       order.
/// @note this does not use the skeletons tag type to match skeletons
///       since it needs to inspect the inline flows themselves. This
///       means it may be necessary to have zip_fusion as the first
///       transformation pass before the skeleton is wrapped in
///       another transform<> specialization.
///
/// This transformation works by identifying sub-DAGs only containing
/// zip skeletons in the larger compose DAG. They have the property
/// that except for one "sink" vertex, the outputs of each vertex is
/// only used internally, and not by any other skeleton in the larger
/// compose DAG. Then these sub-DAGs are fused into a single zip. The
/// outputs of the new skeleton are exactly those of the sink and the
/// input is the union of all the component zips' inputs.
///
/// This then returns a new compose skeleton with updated flows and
/// skeletons. In the case where all the skeletons in the compose
/// can be fused into a single node, a single zip is returned instead.
///
/// @see zip
/// @see inline_flow
//////////////////////////////////////////////////////////////////////
template<class... Skeletons, class... PhFlows, typename SkeletonTag>
struct transform<skeletons_impl::compose<tuple<Skeletons...>,
  inline_flow<tuple<PhFlows...>>>, SkeletonTag, tags::zip_fusion>
{
private:
  using flow_t = inline_flow<tuple<PhFlows...>>;

#ifndef STAPL_NDEBUG
  // Get a list of sorted output phs, the topsort algorithm is stable
  // so if it is already sorted, it should be in the same order
  using sorted =
    typename flows::inline_flows::topsort<tuple<PhFlows...>>::type;

  // and check to make sure our flows are in the same order
  static_assert(std::is_same<tuple<typename PhFlows::output_t...>, sorted>(),
      "Inline flows should be specified in topsort order for Zip Fusion");
#endif

  using skeletons_t = tuple<Skeletons...>;
  using tags = tuple<typename Skeletons::skeleton_tag_type...>;

  using skeleton_t = skeletons_impl::compose<skeletons_t, flow_t>;

  using is_zips = typename zip_fusion_impl::rev_is_zip<tags>::type;

  using apply = zip_fusion_impl::fuse_zips<is_zips, flow_t>;

  using result_t = typename apply::type;

  using new_t = zip_fusion_impl::make_fused<result_t, flow_t>;

  using new_skeleton_t = typename new_t::template new_skeleton_t<skeletons_t>;

public:
  static new_skeleton_t call(skeleton_t const& skeleton)
  {
    return new_t::get_skeleton(skeleton.get_skeletons());
  }
};

} // namespace transformations
} // namespace skeletons
} // namespace stapl
#endif // STAPL_SKELETONS_TRANSFORMATIONS_COMPOSE_ZIP_FUSION_HPP
