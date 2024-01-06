/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_BENCHMARK_LONESTAR_DELAUNAY_TRIANGULATION_MERGE_PARALLEL_HPP
#define STAPL_BENCHMARK_LONESTAR_DELAUNAY_TRIANGULATION_MERGE_PARALLEL_HPP

#include <benchmarks/lonestar/delaunay/triangulation/delaunay_triangulation_merge_sequential.hpp>


namespace stapl
{


namespace delaunay
{


///////////////////////////////////////////////////////////////////////////////
/// @brief Workfunction used in the map_reduce to merge the partitions across
/// processes.
/// @todo Need to implement better way of parallelizing the merge process by
/// ditching the map_reduce and spawning tasks. Specifically the find left and
/// right candidates can be found in parallel. Current method attempts to do
/// this, but can be greatly improved upon.
///////////////////////////////////////////////////////////////////////////////
template <typename View>
struct merge_wf
  : seq_merge_wf<View>
{
  typedef result_data<delaunay_edge> result_type;

  typedef merge_wf<View> this_type;
  typedef seq_merge_wf<View> base_type;

  typedef edge_descriptor_impl<size_t> edge_descriptor;
  typedef std::pair<delaunay_edge, delaunay_edge> edge_pair;

  typedef typename base_type::find_lcand_wf find_lcand_wf;
  typedef typename base_type::find_rcand_wf find_rcand_wf;

  counter<default_timer>& m_merge_timer;
  size_t m_loc;

  merge_wf(View* view, counter<default_timer>& merge_timer)
    : base_type(view), m_merge_timer(merge_timer),m_loc(view->get_location_id())
  { }

  template <typename WF>
  merge_wf(View* view, WF&& wf)
    : base_type(view, wf), m_merge_timer(wf.m_merge_timer),
      m_loc(view->get_location_id())
  { }


  /////////////////////////////////////////////////////////////////////////////
  /// @brief Merges two partial triangulations into a complete triangulation.
  /// Since the majority of the updates will be on two locations, this function
  /// uses a rmi call to run primarily on one of those locations.
  /// @todo Change pattern to automatically call workfunction on correct
  /// location instead of making an additional rmi call.
  /////////////////////////////////////////////////////////////////////////////
  result_type operator()(result_type left, result_type right)
  {
    m_merge_timer.start();

    this->m_dom_low  = left.first.source();
    this->m_dom_high = right.second.source();
    this->m_dom_mid  = left.second.source();
    this->m_loc = this->m_view->get_location_id();
    result_type result;

    if (left.m_loc2 != m_loc) {
      auto parallel_future =
            async_command(left.m_loc2, &this_type::parallel_merge, left, right);
      result = parallel_future.get();
    } else {
      result = parallel_merge(left, right);
    }

    m_merge_timer.stop();
    return result;
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @todo Implement parallel versions of the inner and outer update edges.
  /////////////////////////////////////////////////////////////////////////////
  result_type parallel_merge(result_type left, result_type right) {
    this->update_inner_edges(left, right);
    auto basel = this->connect_basel_edge(right.first.get_Sym(), left.second);
    this->update_outer_edges(left, right, basel);

    while (true) {
      auto rcand_future = find_rcand_wf(*this, basel).async();
      auto lcand_future = find_lcand_wf(*this, basel).async();
      auto lcand = lcand_future.get();
      auto rcand = rcand_future.get();
      if (!this->connect_basel_edge(basel, lcand, rcand)) {
        break;
      }
    }

    return result_type(left.first, right.second, left.m_loc1, right.m_loc2);
  }


  template <typename Func, typename... Args>
  auto async_command(size_t loc, Func func, Args const&... args) ->
                                 stapl::future<decltype((this->*func)(args...))>
  {
    return this->m_view->async_command(loc, *this, func, args...);
  }


  void define_type(stapl::typer& t)
  {
    base_type::define_type(t);
    t.member(m_loc);
  }
};


} // namespace delaunay


} // namespace stapl


#endif /* STAPL_BENCHMARK_LONESTAR_DELAUNAY_TRIANGULATION_MERGE_PARALLEL_HPP */
