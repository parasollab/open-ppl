/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_BENCHMARK_LONESTAR_DELAUNAY_LOCAL_TRIANGULATION_HPP
#define STAPL_BENCHMARK_LONESTAR_DELAUNAY_LOCAL_TRIANGULATION_HPP

#include <stack>

#include <benchmarks/lonestar/delaunay/triangulation/delaunay_triangulation_merge_sequential.hpp>


namespace stapl
{


namespace delaunay
{


///////////////////////////////////////////////////////////////////////////////
/// @brief Used in the merge process and contains an edge/loc from both left
/// and right partitions. It is also equivalent to a pair of edges with the
/// addition of location information for each edge.
///////////////////////////////////////////////////////////////////////////////
template <typename Edge>
struct result_data
{
  typedef std::pair<Edge, Edge> edge_pair;
  Edge first;
  Edge second;
  size_t m_loc1;
  size_t m_loc2;

  result_data()
  { }

  result_data(Edge const& e1, Edge const& e2)
    : first(e1), second(e2)
  { }

  result_data(Edge const& e1, Edge const& e2, size_t loc)
    : first(e1), second(e2), m_loc1(loc), m_loc2(loc)
  { }

  result_data(Edge const& e1, Edge const& e2, size_t loc1, size_t loc2)
    : first(e1), second(e2), m_loc1(loc1), m_loc2(loc2)
  { }

  result_data(edge_pair const& pair)
    : first(pair.first), second(pair.second)
  { }

  result_data(result_data const& data)
    : first(data.first), second(data.second),
      m_loc1(data.m_loc1), m_loc2(data.m_loc2)
  { }

  result_data(edge_pair const& pair, size_t loc)
    : first(pair.first), second(pair.second), m_loc1(loc), m_loc2(loc)
  { }

  result_data(result_data const& data, size_t loc)
    : first(data.first), second(data.second), m_loc1(loc), m_loc2(loc)
  { }

  result_data(edge_pair const& pair, size_t loc1, size_t loc2)
    : first(pair.first), second(pair.second), m_loc1(loc1), m_loc2(loc2)
  { }

  result_data(result_data const& data, size_t loc1, size_t loc2)
    : first(data.first), second(data.second), m_loc1(loc1), m_loc2(loc2)
  { }

  operator edge_pair()
  {
    return edge_pair(first, second);
  }

  void define_type(stapl::typer& t)
  {
    t.member(first);
    t.member(second);
    t.member(m_loc1);
    t.member(m_loc2);
  }
};


template <typename>
struct local_tri;


///////////////////////////////////////////////////////////////////////////////
/// @brief Workfunction for triangulation that is passed into the map_reduce.
/// Works on the native_view of input graph, recursively dividing the input in
/// half until reaching the a size of either 2 or 3 points. The points are then
/// merged back together producing partial triangulations, until the entire
/// native_view is fully triangulated.
///////////////////////////////////////////////////////////////////////////////
template <typename GView>
struct local_tri_wf
{
  typedef edge_descriptor_impl<size_t>              edge_descriptor;
  typedef std::pair<delaunay_edge, delaunay_edge>   edge_pair;
  typedef result_data<delaunay_edge>                result_type;

  GView* m_gview;
  counter<default_timer>& m_local_timer;
  size_t m_loc;
  size_t m_dom_low;
  size_t m_dom_high;


  local_tri_wf(GView* gview, counter<default_timer>& local_timer)
    : m_gview(gview), m_local_timer(local_timer)
  { }

  local_tri_wf(GView* gview, counter<default_timer>& local_timer, size_t loc,
               size_t low, size_t high)
    : m_gview(gview),  m_local_timer(local_timer), m_loc(loc),
      m_dom_low(low), m_dom_high(high)
  { }

  template <typename NView>
  result_type operator()(NView&& nview)
  {
    m_local_timer.start();

    auto& domain = nview.domain();
    m_dom_low = domain.first();
    m_dom_high = domain.last();

    local_tri<NView> tri(&nview);
    result_type result(tri.triangulate_non_rec(m_dom_low, m_dom_high), m_loc);

    m_local_timer.stop();
    return result;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_gview);
    t.member(m_loc);
    t.member(m_dom_low);
    t.member(m_dom_high);
  }
};

///////////////////////////////////////////////////////////////////////////////
/// @brief Implements both the recursive and iterative versions of the 'divide'
/// portion of the the algorithm. Called from @ref local_tri_wf, it also stores
/// a reference to the native_view object
///////////////////////////////////////////////////////////////////////////////
template <typename View>
struct local_tri
{
  typedef std::pair<delaunay_edge, delaunay_edge> edge_pair;
  typedef edge_pair result_type;

  View* m_view;

  local_tri(View* view)
    : m_view(view)
  { }

  edge_pair triangulate(size_t dom_low, size_t dom_high)
  {
    int num_points = dom_high - dom_low + 1;
    stapl_assert(num_points <= 1,    "ERROR");
    stapl_assert(dom_high > dom_low, "ERROR");

    if (dom_low == dom_high) {
      return edge_pair();
    }

    if (num_points == 2) {
      return m_view->add_delaunay_edge(dom_low, dom_high);
    } else if (num_points == 3) {
      return m_view->add_delaunay_edge(dom_low, dom_low+1, dom_high);
    } else {
      size_t dom_mid = (dom_high + dom_low) / 2;
      edge_pair left  = triangulate(dom_low,   dom_mid);
      edge_pair right = triangulate(dom_mid+1, dom_high);

      seq_merge_wf<View> wf(m_view, dom_low, dom_high);
      return wf.merge(left, right);
    }
  }

  struct tri_stack_data
  {
    size_t m_dom_low;
    size_t m_dom_high;
    int m_stage;

    tri_stack_data()
    { }

    tri_stack_data(size_t dom_low, size_t dom_high)
      : m_dom_low(dom_low), m_dom_high(dom_high), m_stage(0)
    { }

    tri_stack_data(size_t dom_low, size_t dom_high, int stage)
      : m_dom_low(dom_low), m_dom_high(dom_high), m_stage(stage)
    { }
  };

  edge_pair triangulate_non_rec(size_t dom_low, size_t dom_high)
  {
    std::stack<tri_stack_data> tri_stack;
    std::stack<edge_pair> left_stack;

    edge_pair result;

    tri_stack_data current_data(dom_low, dom_high);
    tri_stack.push(current_data);

    stapl_assert(dom_high > dom_low, "ERROR");

    if (dom_high <= dom_low) {
      return result;
    }

    while (!tri_stack.empty()) {
      current_data = tri_stack.top();
      tri_stack.pop();
      size_t dom_low = current_data.m_dom_low;
      size_t dom_high = current_data.m_dom_high;
      switch(current_data.m_stage) {
      case 0:
        stapl_assert(dom_high > dom_low, "ERROR");
        if (dom_high - dom_low == 1) { // 2 Vertices
          result = m_view->add_delaunay_edge(dom_low, dom_high);
        } else if (dom_high - dom_low == 2) { // 3 Vertices
          result = m_view->add_delaunay_edge(dom_low, dom_low+1, dom_high);
        } else {
          current_data.m_stage = 1;
          tri_stack.push(current_data);

          size_t dom_mid = (dom_high + dom_low)/2;
          tri_stack_data left_data(dom_low, dom_mid);
          tri_stack.push(left_data);
        }
        continue;
        break;
      case 1: {
        current_data.m_stage = 2;
        left_stack.push(result);
        tri_stack.push(current_data);

        size_t dom_mid = (dom_high + dom_low) / 2;
        tri_stack_data right_data(dom_mid+1, dom_high);
        tri_stack.push(right_data);
        continue;
        break;
      }
      case 2: {
        seq_merge_wf<View> merge_wf(m_view, dom_low, dom_high);
        result = merge_wf.merge(left_stack.top(), result);
        left_stack.pop();
        continue;
        break;
      }
      }
    }
    return result;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_view);
  }
};



} // namespace delaunay


} // namespace stapl


#endif /* STAPL_BENCHMARK_LONESTAR_DELAUNAY_LOCAL_TRIANGULATION_HPP */
