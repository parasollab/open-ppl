/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/hierarchical_graph.hpp>
#include <stapl/containers/graph/views/hgraph_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <boost/lexical_cast.hpp>
#include "test_util.h"

using namespace stapl;

struct test_aggregate_apply_wf
{
  size_t m_target;
  typedef void result_type;
  test_aggregate_apply_wf(size_t t) : m_target(t) { }
  test_aggregate_apply_wf() : m_target(999999) { }
  size_t target() const { return m_target; }

  template<typename P>
  void operator() (P& p) const { p.set_color(m_target*10); }

  void define_type(stapl::typer& t)
  {
    t.member(m_target);
  }
};

struct test_vertex_wf
{
  typedef size_t result_type;

  template<typename E>
  result_type operator() (E e) const {
    e.property().set_color(35);
    result_type result = 0;
    for (typename E::adj_edge_iterator aei = e.begin();
         aei != e.end(); ++aei) {
      result += (*aei).target();
    }
    return result;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cerr << "usage: exe n ef" << std::endl;
    exit(1);
  }
  srand(1000*stapl::get_location_id() + time(NULL));
  size_t n  = boost::lexical_cast<size_t>(argv[1]) * get_num_locations();
  size_t ef = atoi(argv[2]);

  one_print("Testing hierarchical graph add vertex...");
  typedef hierarchical_graph<stapl::DIRECTED, stapl::MULTIEDGES,
                             my_vertex_property> graph_type;
  graph_type g;

  size_t blk_sz = n/get_num_locations();
  size_t blk_st=blk_sz*get_location_id(), blk_end=blk_st+blk_sz;
  for (size_t i=blk_st; i<blk_end; ++i)
    g.add_vertex(i, i);
  rmi_fence();
  one_print(g.size() == n);

  one_print("Testing hierarchical graph copy construction...");
  graph_type g2(g);
  one_print(g2.size() == n);

  one_print("Testing hierarchical graph add-edge...\t");
  std::vector<std::pair<size_t, size_t> > edges;
  for (size_t i=0; i<ef/2; ++i) {
    size_t s = rand()%(n-1);
    size_t t = rand()%(n-1);
    graph_type::edge_descriptor ed = g.add_edge(s, t);
    edges.push_back(std::make_pair(ed.source(), ed.target()));
  }
  for (size_t i=0; i<ef/2; ++i) {
    size_t s = rand()%(n-1);
    size_t t = rand()%(n-1);
    g.add_edge_async(s, t);
    edges.push_back(std::make_pair(s,t));
  }
  stapl::rmi_fence();
  one_print(g.num_edges() == stapl::get_num_locations()*ef);

  one_print("Testing hierarchical graph global-traversal...");
  size_t vertex_sum=0, edge_cnt=0, ref_target_total=0;
  for (graph_type::vertex_iterator vi = g.begin(); vi != g.end(); ++vi) {
    vertex_sum += (*vi).descriptor();
    for (graph_type::adj_edge_iterator aei = (*vi).begin();
         aei != (*vi).end(); ++aei) {
      ++edge_cnt;
      ref_target_total += (*aei).target();
    }
  }
  one_print( (edge_cnt==stapl::get_num_locations()*ef) &&
                 (vertex_sum==(n*(n-1))/2) );

  stapl::rmi_fence();

  one_print("Testing operator [] :\t\t\t");
  one_print(g[n/2+1].descriptor() == n/2+1);

  one_print("Testing view over hierarchical graph...\t");
  hgraph_view<graph_type> v(g, false);
  size_t target_total = stapl::map_reduce(test_vertex_wf(),
                                          stapl::plus<size_t>(), v);
  one_print(target_total == ref_target_total);

  one_print("Testing property proxy...\t\t");
  one_print(g[3].property().get_color() == 35);

  one_print("Testing property proxy global access...\t");
  g[n/2+1].property().set_color(67);
  one_print(g[n/2+1].property().get_color() == 67);

  one_print("Testing vp apply async...\t\t");
  g.vp_apply_async(1, test_aggregate_apply_wf(1));
  one_print(g[1].property().get_color() == 10);

  one_print("Testing delete edges...\t\t\t");
  for (size_t i=0; i<edges.size(); ++i)
    g.delete_edge(edges[i].first, edges[i].second);
  stapl::rmi_fence();
  one_print((g.num_edges() == 0) && (g.num_local_edges() == 0));

  one_print("Testing graph clear...\t\t\t");
  g.clear();
  stapl::rmi_fence();
  one_print(g.size() == 0);

  return EXIT_SUCCESS;
}
