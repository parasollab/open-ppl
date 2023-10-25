/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/k_core_dynamic.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/generators/random_neighborhood.hpp>

#include "test_util.h"
using namespace stapl;

template<typename Comp>
struct vertex_sz_wf
{
  typedef bool result_type;

  size_t m_k;
  Comp m_comp;

  vertex_sz_wf(size_t k, Comp comp)
    : m_k(k), m_comp(comp)
  { }

  template <typename T>
  result_type operator()(T v) const
  {
    if (!m_comp(v.size(), m_k))
      return true;
    else
      return false;
  }

  void define_type(typer& t)
  {
    t.member(m_k);
    t.member(m_comp);
  }
};


void test_graph(size_t n, size_t ef, size_t k, bool greater_flag)
{
  typedef dynamic_graph<stapl::DIRECTED, stapl::MULTIEDGES> graph_type;
  typedef graph_view<graph_type> graph_view_t;

  graph_view_t v = generators::make_random_neighborhood<graph_view_t>(n, ef, n);

  size_t num_deleted = 0;
  if (greater_flag) {
    one_print("Testing k-Core<greater> Algorithm [Dynamic]...");
    num_deleted = k_core_dynamic(v, k, stapl::greater<size_t>());
  } else {
    one_print("Testing k-Core Algorithm [Dynamic]...\t");
    num_deleted = k_core_dynamic(v, k);
  }
  bool passed = true;
  if (v.size() != 0) {
    if (greater_flag) {
      vertex_sz_wf<stapl::greater<size_t> > pred(k, stapl::greater<size_t>());
      passed = all_of(graph_view_t(v.container()), pred);
    } else {
      vertex_sz_wf<stapl::less<size_t> > pred(k, stapl::less<size_t>());
      passed = all_of(graph_view_t(v.container()), pred);
    }
  }
  passed &= (v.size() == n-num_deleted);
  one_print(passed);

  rmi_fence();
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 4) {
    std::cerr << "usage: exe n ef k" << std::endl;
    exit(1);
  }
  srand(1000*stapl::get_location_id() + time(NULL));
  size_t n  = atoi(argv[1]) * get_num_locations();
  size_t ef = atoi(argv[2]);
  size_t k  = atoi(argv[3]);

  test_graph(n, ef, k, false);
  test_graph(n, ef, k, true);

  return EXIT_SUCCESS;
}
