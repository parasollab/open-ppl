/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_SCHEDULER_TASK_PLACEMENT_HPP
#define STAPL_RUNTIME_EXECUTOR_SCHEDULER_TASK_PLACEMENT_HPP

#include <algorithm>
#include <array>
#include <tuple>
#include <utility>

#include "default_info.hpp"
#include <stapl/paragraph/utility.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/loc_qual.hpp>
#include <stapl/runtime.hpp>

#include <boost/mpl/has_xxx.hpp>

namespace stapl {

namespace detail {

struct locality_compare
{
  bool operator()(locality_info const& lhs,
                  locality_info const& rhs) const noexcept
  {
    // If both locality descriptors have an validly initialized
    // affinity, try to use this to compare.
    if (lhs.affinity() != invalid_affinity_tag
        && lhs.affinity() == rhs.affinity())
      return true;

    // Are the gangs equivalent?  If so, compare their locations.
    if (compare_gangs(lhs.handle(), rhs.handle()) == 0)
      return lhs.location() == rhs.location();

    // Otherwise, conservatively treat them as not equivalent
    // (thought it's possible they are).
    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns if the first element of the left @c std::pair is less than
///        the first element of the second @c std::pair.
//////////////////////////////////////////////////////////////////////
class pair_compare
{
private:
  typedef std::pair<locality_info, std::size_t> pair_t;

public:
  bool operator()(pair_t const& lhs, pair_t const& rhs) const noexcept
  {
    return locality_compare()(lhs.first, rhs.first);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns if the second element of the stored @c std::pair is equal
///        to the second element of the given @c std::pair.
//////////////////////////////////////////////////////////////////////
class pair_compare_loc
{
private:
  typedef std::pair<locality_info, std::size_t> pair_t;

  locality_info m_locality;

public:
  pair_compare_loc(locality_info locality)
    : m_locality(std::move(locality))
  { }

  bool operator()(pair_t const& element) const noexcept
  {
    return locality_compare()(element.first, m_locality);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns the preferred location for a task to run.
//////////////////////////////////////////////////////////////////////
struct register_vote
{
private:
  typedef std::pair<locality_info, std::size_t> pair_t;

  pair_t*        m_certain;
  pair_t*        m_lookup;
  std::size_t&   m_anywhere_cnt;

public:
  typedef void result_type;

  register_vote(pair_t* certain,
                pair_t* lookup,
                std::size_t& anywhere_cnt) noexcept
    : m_certain(certain),
      m_lookup(lookup),
      m_anywhere_cnt(anywhere_cnt)
  { }

private:
  void register_vote_impl(pair_t* votes,
                          locality_info const& locality) const noexcept
  {
    bool assigned = false;

    for (size_t idx = 0; !assigned; ++idx)
    {
      if (locality_compare()(locality, votes[idx].first))
      {
        ++votes[idx].second;
        assigned = true;
      }
      // reached the end of list, add a new entry...
      else if (votes[idx].second == 0)
      {
        votes[idx].first  = locality;
        votes[idx].second = 1;
        assigned          = true;
      }
    }
  }

public:
  template<typename ComponentId>
  void operator()(ComponentId& cid) const noexcept
  {
    const locality_info locality = (cid.first)->locality(cid.second);

    switch (locality.qualifier())
    {
      case LQ_CERTAIN:
        register_vote_impl(m_certain, locality);
        break;
      case LQ_LOOKUP:
        register_vote_impl(m_lookup, locality);
        break;
      case LQ_DONTCARE:
        ++m_anywhere_cnt;
        break;
      default:
        abort("Unexpected location qualifier in tally_location_votes");
        break;
    }
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Default policy for initially mapping a task to a PARAGRAPH
///   location for execution.
///
/// @see Examples of specialization in serial() skeleton.
/// @todo This code does not belong to the runtime. It belongs to the PARAGRAPH.
//////////////////////////////////////////////////////////////////////
struct default_task_placement_impl
{
private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of @ref execution_location for no views.
  //////////////////////////////////////////////////////////////////////
  static locality_info execution_location(size_t) noexcept
  {
    return LQ_DONTCARE;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of @ref execution_location for one view.
  //////////////////////////////////////////////////////////////////////
  template<typename View0>
  static locality_info execution_location(size_t, View0 const& view0)
  {
    const locality_info vw_vote = (view0.first)->locality(view0.second);

    // if the view has a preference (either LQ_LOOKUP or LQ_CERTAIN),
    /// return that.
    if (vw_vote.qualifier() != LQ_DONTCARE)
    {
      // make sure the view0 didn't tell us to lookup on this location
      stapl_assert(!((vw_vote.qualifier() == LQ_LOOKUP)
        && (vw_vote.affinity() == get_affinity())),
        "execution_location(...): view0 requested recursive LQ_LOOKUP");
      return vw_vote;
    }

    return LQ_DONTCARE;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of @ref execution_location for two views.
  //////////////////////////////////////////////////////////////////////
  template<typename View0, typename View1>
  static locality_info
  execution_location(size_t, View0 const& view0, View1 const& view1)
  {
    const locality_info vw0_vote = (view0.first)->locality(view0.second);

    // LQ_CERTAIN takes precedence over LQ_LOOKUP, secondarily vw0 over vw1
    if (vw0_vote.qualifier() == LQ_CERTAIN)
      return vw0_vote;

    const locality_info vw1_vote = (view1.first)->locality(view1.second);

    if (vw1_vote.qualifier() == LQ_CERTAIN)
      return vw1_vote;

    stapl_assert(!(vw0_vote.qualifier() == LQ_LOOKUP
                   && vw0_vote.affinity() == get_affinity()),
                 "view requested recursive LQ_LOOKUP");

    stapl_assert(!(vw1_vote.qualifier() == LQ_LOOKUP
                 && vw1_vote.affinity() == get_affinity()),
                 "view requested recursive LQ_LOOKUP");

    if (vw0_vote.qualifier() == LQ_LOOKUP)
      return vw0_vote;

    if (vw1_vote.qualifier() == LQ_LOOKUP)
      return vw1_vote;

    return LQ_DONTCARE;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the preferred location for a task to run based on it
  ///   views.
  ///
  /// @todo This code does not belong to the runtime. It belongs to the
  ///   PARAGRAPH.
  ///
  /// @todo Remove redundant call to locality on first view (in source read
  ///   heuristic) by wrapping assignment as cache of first view's vote in
  ///   vs_map call above.
  //////////////////////////////////////////////////////////////////////
  template<typename View0, typename View1, typename ...Views>
  static locality_info
  execution_location(size_t anywhere_cnt, View0 const& view0,
                     View1 const& view1, Views const&... views)
  {
    // NOTE - We avoid heap allocated maps (we initially used std::map)
    // and use a fixed sized (num subviews), stack allocated structure.
    // Basic idea store vote pairs (location_id, tally) for certain and lookup
    // O(V^2) for scanning since votes are stored in location ordered but no big
    // deal for small V which practically will likely always be single digits.
    //
    enum { svs_size = sizeof...(Views) + 2};

    static_assert(svs_size > 0, "general policy assumes non empty viewset");

    // Locality Vote and Vote Count
    typedef std::pair<locality_info, size_t> pair_t;
    typedef std::array<pair_t, svs_size>     array_t;
    typedef typename array_t::iterator       iterator_t;

    array_t certain{};
    array_t lookup{};

    // std::size_t anywhere_cnt = 0;

    auto spec = std::forward_as_tuple(view0, view1, views...);

    vs_map(detail::register_vote(certain.begin(), lookup.begin(), anywhere_cnt),
           spec);

    // if no one is certain, all we can do is lookup
    if (certain[0].second == 0)
    {
      // i.e., !lookup.empty()
      //
      if (lookup[0].second != 0)
      {
        // Otherwise select location for revote if lookups are needed.
        iterator_t lookup_max = std::max_element(lookup.begin(), lookup.end(),
                                                 detail::pair_compare());

        return lookup_max->first;
      }

      // no one is certain and no one wants to lookup
      return LQ_DONTCARE;
    }

    // find location with most certain votes.  while loop is similar to
    // max_element() but short circuits if current max is already a majority.
    auto iter             = certain.begin();
    const auto end_iter   = certain.end();
    auto max_certain_iter = certain.begin();

    while (max_certain_iter->second < svs_size / 2 && ++iter != end_iter)
      if (detail::pair_compare()(*max_certain_iter, *iter))
        max_certain_iter = iter;

    const pair_t max_certain_vote =
     max_certain_iter != certain.end() ? *max_certain_iter
     : pair_t(locality_info(LQ_CERTAIN, get_affinity(),
                            rmi_handle::reference(), invalid_location_id),
               0);

    // if there is a majority, place the task.
    if (max_certain_vote.second + anywhere_cnt > svs_size / 2)
      return max_certain_vote.first;

    if (lookup[0].second != 0)
    {
      iterator_t lookup_max =
        std::max_element(lookup.begin(), lookup.end(), detail::pair_compare());

      return lookup_max->first;
    }

    // Finally, assume first view is read, and prefer it in case of ties.
    locality_info src_vote = (view0.first)->locality(view0.second);

    if (src_vote.qualifier() == LQ_CERTAIN)
    {
      const iterator_t iter =
        std::find_if(certain.begin(), certain.end(),
        detail::pair_compare_loc(src_vote));

      if (iter != certain.end() && max_certain_vote.second == iter->second)
        return src_vote;
    }

    // If we can't reach a majority, and the first view heuristic above fails,
    // place the task at the location winning a plurality of the votes.
    return max_certain_vote.first;
  }

public:
  template<typename Views, std::size_t ...Indices>
  static locality_info
  apply_impl(size_t anywhere_cnt, Views const& views,
             index_sequence<Indices...>)
  {
    return execution_location(anywhere_cnt, get<Indices>(views)...);
  }


  template<typename ...Views>
  static locality_info
  apply(size_t anywhere_cnt, tuple<Views...> const& views)
  {
    return apply_impl(anywhere_cnt, views,
                      make_index_sequence<sizeof...(Views)>());
  }
}; // struct default_task_placement_impl


BOOST_MPL_HAS_XXX_TRAIT_DEF(task_placement_dontcare)


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction returns true if View type parameter is statically
///   known to have an LQ_DONTCARE vote. Primary template covers case
///   when task_placement_dontcare typedef is not reflected in class.
///
///   Reflects @ref std::false_type.
//////////////////////////////////////////////////////////////////////
template<typename View, bool b = has_task_placement_dontcare<View>::value>
struct is_static_dontcare
  : std::false_type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization covers when task_placement_dontcare is defined.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct is_static_dontcare<View, true>
  : View::task_placement_dontcare
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Counts the number of LQ_DONTCARE votes known at compile time.
//////////////////////////////////////////////////////////////////////
template<typename RuntimeVotes, typename ...Views>
struct count_static_votes_impl;


//////////////////////////////////////////////////////////////////////
/// @brief Base case when all views have been inspected.  Return the
///  number of static LQ_DONTCARE votes and the tuple of views that
///  must be queried at runtime.
//////////////////////////////////////////////////////////////////////
template<typename ...RuntimeViews>
struct count_static_votes_impl<tuple<RuntimeViews const&...>>
{
  typedef tuple<size_t, tuple<RuntimeViews const&...>> type;

  static type apply(size_t anywhere_cnt,
                    tuple<RuntimeViews const&...> const& runtime_voters)
  { return make_tuple(anywhere_cnt, runtime_voters); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Recursive case.  Inspect @p View0 and either increment static
///   vote count or add to tuple of views requiring runtime evaluation.
//////////////////////////////////////////////////////////////////////
template<typename ...RuntimeViews, typename View0, typename ...Views>
struct count_static_votes_impl<tuple<RuntimeViews const&...>, View0, Views...>
{
private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Helper method covering case when vote is statically known.
  //////////////////////////////////////////////////////////////////////
  template<std::size_t ...Indices>
  static
  typename count_static_votes_impl<
    tuple<RuntimeViews const&...>, Views...
  >::type
  apply_impl(size_t anywhere_cnt, std::true_type, index_sequence<Indices...>,
             tuple<RuntimeViews const&...> const& runtime_voters,
             View0 const&, Views const&... views)
  {
    return count_static_votes_impl<tuple<RuntimeViews const&...>, Views...>::
      apply(anywhere_cnt + 1, runtime_voters, views...);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Helper method covering case when vote must be resolved at runtime.
  //////////////////////////////////////////////////////////////////////
  template<std::size_t ...Indices>
  static
  typename count_static_votes_impl<
    tuple<RuntimeViews const&..., View0 const&>,
    Views...
  >::type
  apply_impl(size_t anywhere_cnt, std::false_type,
             index_sequence<Indices...>,
             tuple<RuntimeViews const&...> const& runtime_voters,
             View0 const& view0, Views const&... views)
  {
    return count_static_votes_impl<
      tuple<RuntimeViews const&..., View0 const&>, Views...>::
        apply(anywhere_cnt,
              std::tuple<RuntimeViews const&..., View0 const&>
                (get<Indices>(runtime_voters)..., view0),
              views...);
  }


  typedef std::integral_constant<
    bool,
      is_static_dontcare<
        typename std::remove_pointer<
          typename std::decay<View0>::type::first_type
        >::type
      >::value
    >                                                             bool_static_t;

public:
  typedef typename std::conditional<bool_static_t::value,
    typename count_static_votes_impl<
      tuple< RuntimeViews const&...>, Views...
    >::type,
    typename count_static_votes_impl<
      tuple<RuntimeViews const&..., View0 const&>, Views...
    >::type
  >::type                                                         type;


  static type
  apply(size_t anywhere_cnt,
        tuple<RuntimeViews const&...> const& runtime_voters,
        View0 const& view0, Views const&... views)
  {
    return apply_impl(anywhere_cnt, bool_static_t(),
                      make_index_sequence<sizeof...(RuntimeViews)>(),
                      runtime_voters, view0, views...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Inspect input views and return a tuple with two elements.
///   The first is the number of statically resolvable LQ_DONTCARE votes.
///   The second is a tuple of views requiring runtime evaluation.
///   Both are passed to the runtime portion of the default task placement
///   policy.
//////////////////////////////////////////////////////////////////////
template<typename... Views>
auto count_static_votes(Views const&... views)
STAPL_AUTO_RETURN((
  count_static_votes_impl<tuple<>, Views...>::apply(0, tuple<>(), views...)
))


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper around @ref default_task_placement_impl that intercepts
///   tasks with factory workfunctions and applies a basic nested
///   PARAGRAPH placement policy (place on the location of the input
///   view).
//////////////////////////////////////////////////////////////////////
struct default_task_placement
{
  template<typename WF, typename ...Views>
  typename std::enable_if<!is_factory<WF>::value, locality_info>::type
  execution_location(WF const&, Views const&... views)
  {
    const auto static_vote_result = count_static_votes(views...);

    return default_task_placement_impl::apply(get<0>(static_vote_result),
                                              get<1>(static_vote_result));
  }

  template<typename WF>
  typename std::enable_if<
    is_factory<WF>::value,
    std::pair<rmi_handle::reference, std::vector<unsigned int>>
  >::type
  execution_location(WF const&)
  {
    return std::make_pair(rmi_handle::reference(), std::vector<unsigned int>());
  }

  template<typename WF, typename View, typename ...Views>
  typename std::enable_if<
    is_factory<WF>::value,
    std::pair<rmi_handle::reference, std::vector<unsigned int>>
  >::type
  execution_location(WF const& wf, View const& view, Views const&...)
  {
    return std::make_pair(view.first->nested_locality(view.second),
                          std::vector<unsigned int>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return priority of the PARAGRAPH being scheduled.
  ///
  /// Method required by paragraph::operator() when the executor for the
  /// paragraph is added to the parent executor.
  //////////////////////////////////////////////////////////////////////
  constexpr default_info get_sched_info(void) const noexcept
  { return default_info{}; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Hook to allow tracking of when tasks are executed.
  //////////////////////////////////////////////////////////////////////
  void notify_finished(void) noexcept
  { }
}; // struct default_task_placement

} // namespace stapl

#endif
