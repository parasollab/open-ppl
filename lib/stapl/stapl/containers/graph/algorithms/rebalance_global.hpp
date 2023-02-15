/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_REBALANCE_GLOBAL_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_REBALANCE_GLOBAL_HPP

#include <stapl/views/counting_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/generators/single.hpp>
#include <stapl/containers/graph/algorithms/rebalance_diffusive.hpp>

#include <algorithm>
#include <utility>
#include <vector>

#include <boost/static_assert.hpp>
#include <boost/type_traits/is_same.hpp>

namespace stapl {

namespace global_redistribute_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to compare two pairs, as needed by the algorithm.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pair_greater_weight_op
{
  template<typename T>
  bool operator() (T const& t1, T const& t2) const
  {
    if (t2.second < t1.second)
      return true;
    else if (t2.second == t1.second && t2.first > t1.first)
      return true;
    else
      return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to return the total weight on each location.
/// @tparam Weight The type of the weight.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Weight>
struct size_wf
{
  typedef std::vector<std::pair<size_t, Weight> > result_type;

  template<typename NativeView, typename Count, typename WMap>
  result_type operator() (NativeView const& native_view, Count const& my_id,
                          WMap weight_map)
  {
    typename NativeView::const_iterator it = native_view.begin(),
                                      it_e = native_view.end();
    size_t total_weight = 0;
    for (; it != it_e; ++it)
      total_weight += weight_map.get(*it);

    return result_type(1, std::make_pair(my_id, total_weight));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to merge two vectors.
/// @tparam Weight The type of the weight.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Weight>
struct size_reduce_wf
{
  typedef std::vector<std::pair<size_t, Weight> > result_type;

  template<typename T>
  result_type operator() (T const& t1, T const& t2)
  {
    result_type a(t1), b(t2), c(t1.size() + t2.size());
    std::merge(a.begin(), a.end(),
               b.begin(), b.end(),
               c.begin(), pair_greater_weight_op());
    return c;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to add a weight-value with a pair's second member.
/// @tparam Weight The type of the weight.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Weight>
struct add_second_wf
{
  typedef Weight result_type;
  template<typename T1, typename T2>
  result_type operator() (T1 const& t1, T2 const& t2)
  { return t1 + t2.second; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to set a pair's first member to the stored id.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pair_second_eq
{
  size_t m_id;
  pair_second_eq(size_t id)
    : m_id(id)
  { }

  template<typename T>
  bool operator() (T const& t)
  { return t.first == m_id; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to compute and address the global imbalance
/// based on vertex-weights.
/// @ingroup pgraphAlgoDetails
///
/// Figures out the global imbalance information and broadcasts it to all
/// locations. The locations can then independently compute if, where, which,
/// and how many vertices they need to migrate.
/// @tparam MigrationActionMap A map specifying an action to be performed when a
/// vertex is migrated.
/// @tparam Weight Type of the weight on each vertex.
//////////////////////////////////////////////////////////////////////
template<typename MigrationActionMap, typename Weight>
struct global_redistribute_wf
{
private:
  typedef std::vector<std::pair<size_t, Weight> > vec_pair_loc_wt_t;
  vec_pair_loc_wt_t  m_num_lcl_elts;
  MigrationActionMap m_migration_action_map;

public:
  typedef void result_type;

  global_redistribute_wf(vec_pair_loc_wt_t const& num_lcl_elts,
                         MigrationActionMap const& migration_action_map)
    : m_num_lcl_elts(num_lcl_elts),
      m_migration_action_map(migration_action_map)
  { }

  template<typename NativeView, typename Count, typename GlblVw, typename WMap>
  void operator() (NativeView const& native_view, Count const& my_id,
                   GlblVw const& glbl_cont_vw,
                   WMap weight_map)
  {
    typedef typename WMap::value_type weight_t;

    BOOST_STATIC_ASSERT((boost::is_same<Weight, weight_t>::value));

    // find the average elements per location:
    weight_t total_glbl_wt = std::accumulate(m_num_lcl_elts.begin(),
                                             m_num_lcl_elts.end(),
                                             0, add_second_wf<weight_t>());
    weight_t avg = (double)total_glbl_wt / (double)m_num_lcl_elts.size();

    // find total surplus and deficit elements:
    weight_t total_surplus=0, total_deficit=0;
    for (size_t i=0; i<m_num_lcl_elts.size(); ++i) {
      if (m_num_lcl_elts[i].second > avg)
        total_surplus += (m_num_lcl_elts[i].second - avg);
      else if (m_num_lcl_elts[i].second < avg)
        total_deficit += (avg - m_num_lcl_elts[i].second);
    }

    // find my location in the vector:
    typename vec_pair_loc_wt_t::iterator my_order_it =
      std::find_if(m_num_lcl_elts.begin(), m_num_lcl_elts.end(),
                   pair_second_eq(my_id));

    weight_t num_elts_i_have = my_order_it->second;

    // I should be active only if I have a surplus of elements:
    // (If I have a deficit of elements -- do nothing...)
    if (num_elts_i_have > avg) {
      // find total surplus before me:
      weight_t total_surplus_before_me=0;
      for (typename vec_pair_loc_wt_t::iterator it = m_num_lcl_elts.begin();
          it != my_order_it; ++it) {
        total_surplus_before_me += (it->second - avg);
      }

      // find out how much is needed to contribute assuming procs with more
      // elements than mine have already contributed:
      int needed = (int)total_deficit - (int)total_surplus_before_me;
      if (needed > 0) {  // if we still need more elements...
        int what_i_can_contribute = 0;
        if (needed > (int)(num_elts_i_have - avg))
          what_i_can_contribute = num_elts_i_have - avg;
        else
          what_i_can_contribute = needed;

        // find out who (all) to contribute to:
        int filled_deficit_quota = total_surplus_before_me;

        typedef typename vec_pair_loc_wt_t::reverse_iterator r_iter_t;

        r_iter_t r_iter = m_num_lcl_elts.rbegin();

        while (filled_deficit_quota > 0 && r_iter != m_num_lcl_elts.rend()) {
          filled_deficit_quota -= (avg - r_iter->second);
          ++r_iter;
        }

        // handle partial filling of a location's deficit
        if (filled_deficit_quota < 0) {
          --r_iter;
          r_iter->second = (avg + filled_deficit_quota);
        }

        vec_pair_loc_wt_t move_to_vec;

        typename NativeView::const_iterator it_begin = native_view.begin(), it;
        typename NativeView::const_iterator it_end = native_view.end();
        size_t native_vw_size = native_view.size();
        std::vector<bool> available_vec(native_vw_size, 1);
        // available weights: <descriptor, weight>
        vec_pair_loc_wt_t available_weights(native_vw_size);

        // fill-in the available weights, and get them in sorted-order:
        size_t ii=0;
        for (it = it_begin; it != it_end; ++it, ++ii)
          available_weights[ii] = std::make_pair((*it).descriptor(),
                                                 weight_map.get(*it));
        std::sort(available_weights.begin(), available_weights.end(),
                  pair_greater_weight_op());

        // find what I can contribute to each location.
        while (what_i_can_contribute > 0 &&
               r_iter < (m_num_lcl_elts.rend() - 1)) {
          int to_send = avg-r_iter->second;

          if (to_send > what_i_can_contribute)
            to_send = what_i_can_contribute;

          int send = 0;
          // figure out what to migrate to a given location:
          for (size_t jj=0;
               (jj < available_weights.size()) && (send < to_send); ++jj) {
            // get the largest weighted vertex that is available:
            if ((available_weights[jj].second <= (weight_t)to_send)
                && (available_vec[jj] == true)) {
              move_to_vec.push_back(std::make_pair(available_weights[jj].first,
                                                   r_iter->first));
              send += available_weights[jj].second;
              available_vec[jj] = 0; // mark this as unavailable for the future.
            }
          }
          to_send += send;
          what_i_can_contribute -= send;
          ++r_iter;
        }

        for (size_t i=0; i<move_to_vec.size(); ++i) {
          // call pre-migration action from migration_action_map of that gid.
          // action function takes the actual element, the gid of it,
          // and the new location.
          m_migration_action_map[move_to_vec[i].first](
            glbl_cont_vw[move_to_vec[i].first],
            move_to_vec[i].first,
            move_to_vec[i].second
          );

          glbl_cont_vw.container().migrate(move_to_vec[i].first,
                                           move_to_vec[i].second);
        }
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_num_lcl_elts);
    t.member(m_migration_action_map);
  }
};

}  // end namespace global_redistribute_impl


//////////////////////////////////////////////////////////////////////
/// @brief Parallel global rebalancer based on vertex-weights.
///
/// Balances the graph based on vertex-weights by migrating vertices to
/// create an even weight-distribution among the locations.
/// Does one-shot global rebalancing.
/// @param vw The @ref graph_view over the input graph.
/// @param weights The vertex property map storing weights for each vertex.
/// @param migration_action_map A map from vertex descriptor to functor
/// describing an action to perform when a vertex is migrated.
/// @warning Invalidates the input graph_view.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename View, typename Weights, typename MigrationActionMap>
void rebalance_global(View const& vw, Weights const& weights,
                      MigrationActionMap const& migration_action_map)
{
  // get the number of local elements on each location:
  // sorted by greatest number of elements first
  // (smaller location first in case of tie)
  typename stapl::result_of::native_view<View>::type native_vw
    = stapl::native_view(vw);

  typedef typename Weights::value_type weight_t;

  std::vector<std::pair<size_t, weight_t> > num_lcl_elts
    = stapl::map_reduce(global_redistribute_impl::size_wf<weight_t>(),
                        global_redistribute_impl::size_reduce_wf<weight_t>(),
                        native_vw,
                        counting_view<size_t>(native_vw.size(),0),
                        make_repeat_view(weights));

  // calculate and do the rebalancing:
  global_redistribute_impl::global_redistribute_wf<
    MigrationActionMap, weight_t> global_redist_wf(num_lcl_elts,
                                                   migration_action_map);
  stapl::map_func(global_redist_wf,
                  stapl::native_view(vw),
                  counting_view<size_t>(native_vw.size(),0),
                  make_repeat_view(vw),
                  make_repeat_view(weights));
}


//////////////////////////////////////////////////////////////////////
/// @brief Parallel global rebalancer based on vertex-weights.
///
/// Balances the graph based on vertex-weights by migrating vertices to
/// create an even weight-distribution among the locations.
/// Does one-shot global rebalancing.
/// @param vw The @ref graph_view over the input graph.
/// @param weight_map The vertex property map storing weights for each vertex.
/// @warning Invalidates the input graph_view.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename View, typename Weights>
void rebalance_global(View const& vw, Weights const& weight_map)
{
  typedef single_container<detail::do_nothing> migration_action_map_type;

  detail::do_nothing dn;
  migration_action_map_type action_function(dn);

  rebalance_global(vw, weight_map, action_function);
}


//////////////////////////////////////////////////////////////////////
/// @brief Parallel global rebalancer based on number of vertices.
///
/// Balances the graph to even-out the number of vertices on each location.
/// Does one-shot global rebalancing.
/// @param vw The @ref graph_view over the input graph.
/// @warning Invalidates the input graph_view.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename View>
void rebalance_global(View& vw)
{
  graph_internal_property_map<View, detail::return_1> weight(vw,
    detail::return_1());

  rebalance_global(vw, weight);
};

}  // end namespace stapl

#endif
