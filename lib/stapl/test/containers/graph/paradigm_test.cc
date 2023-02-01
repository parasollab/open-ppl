/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/algorithms/paradigms/graph_paradigm.hpp>
#include <stapl/containers/graph/generators/torus.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "test_util.h"

using namespace stapl;


struct init_wf
{
  typedef void result_type;

  template<typename V>
  result_type operator() (V v) const
  {
    if (v.descriptor() % 2 == 0)
      v.property() = true;
    else
      v.property() = false;
  }
};


struct n_op
{
  typedef bool result_type;

  template<typename V>
  result_type operator() (V v) const
  {
    if (v.descriptor() % 2 == 1)
      v.property() = true;
    return true;
  }
};


struct v_op
{
  size_t m_nedges;
  size_t m_size;
  bool m_add;

  v_op(size_t nedges, size_t size, bool add)
    : m_nedges(nedges), m_size(size), m_add(add)
  { }

  typedef bool result_type;

  template<typename V, typename GraphVisitor>
  result_type operator() (V v, GraphVisitor vis) const
  {
    if (vis.level() == 1) {
      if (v.descriptor() % 2 == 0) {
        v.property() = false;
        vis.visit_all_edges(v, n_op());
        return true;
      } else {
        return false;
      }
    } else if (vis.level() == 2) {
      if (m_add == true) {
        for (size_t i=0; i<m_nedges; ++i)
          vis.add_edge(i, v.descriptor());
      } else {
        for (size_t i=0; i<m_nedges; ++i)
          vis.delete_edge(i, v.descriptor());
      }
      return true;
    } else {
      return false;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_nedges);
    t.member(m_size);
    t.member(m_add);
  }
};



struct check_wf
{
  typedef bool result_type;

  template<typename V>
  result_type operator() (V v) const
  {
    if (v.descriptor() % 2 == 1)
      return v.property() == true;
    else
      return v.property() == false;
  }
};


exit_code stapl_main(int argc,char** argv)
{
  size_t nx, ny;
  if (argc > 2) {
    nx = atol(argv[1]);
    ny = atol(argv[2]);
  } else {
    std::cout<<"usage: exe x-dim y-dim\n";
    return EXIT_FAILURE;
  }

  typedef multidigraph<bool> PGR_static;
  typedef graph_view<PGR_static> graph_view_t;
  graph_view_t vgraph = generators::make_torus<graph_view_t>(nx, ny);
  rmi_fence();

  typedef graph_view_t::vertex_descriptor vd_type;

  size_t added_edges = 3;
  size_t g_size = vgraph.size();
  size_t num_edges = vgraph.num_edges();
  rmi_fence();  // needed for num_edges().

  one_print("Testing Lsync Paradigm...\t\t");
  map_func(init_wf(), vgraph);
  // level-sync paradigm:
  size_t iter1
    = graph_paradigm(v_op(added_edges, g_size, true), n_op(), vgraph, 0);
  bool passed = map_reduce(check_wf(), logical_and<bool>(), vgraph);
  one_print(passed);

  one_print("Testing Lsync Paradigm (dynamic)...\t");
  size_t num_edges2 = vgraph.num_edges();
  rmi_fence();  // needed for num_edges().
  passed = true;
  if (num_edges2 - num_edges !=  added_edges*g_size)
    passed = false;
  one_print(passed);

  one_print("Testing KLA Paradigm...\t\t\t");
  passed = true;
  map_func(init_wf(), vgraph);
  // kla BFS:
  size_t iter2
    = graph_paradigm(v_op(added_edges, g_size, false), n_op(), vgraph, 2);
  passed = map_reduce(check_wf(), logical_and<bool>(), vgraph);
  one_print(passed);

  one_print("Testing KLA Paradigm (dynamic)...\t");
  size_t num_edges3 = vgraph.num_edges();
  rmi_fence();  // needed for num_edges().
  passed = true;
  if (num_edges2 <= num_edges3)
    passed = false;
  one_print(passed);

  one_print("Testing Lsync vs. KLA Paradigm...\t");
  passed = (iter2 < iter1);
  one_print(passed);

  return EXIT_SUCCESS;
}
