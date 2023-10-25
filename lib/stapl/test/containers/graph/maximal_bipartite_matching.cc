/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/maximal_bipartite_matching.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/torus.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/utility/do_once.hpp>

#include "test_util.h"

using namespace std;

class set_partitions
{
  size_t m_sz;

public:
  set_partitions(size_t sz = 0)
    : m_sz(sz)
  { }

  typedef void result_type;

  template <typename V>
  result_type operator() (V v) const
  {
    if (v.descriptor() < m_sz)
      v.property().partition(stapl::properties::bipartite_partition::LEFT);
    else
      v.property().partition(stapl::properties::bipartite_partition::RIGHT);
  }

  void define_type(stapl::typer& t)
  { t.member(m_sz); }
};


template <class GraphView>
GraphView generate_random_bipartite_graph(size_t nv, size_t ne,
                                          size_t group_size)
{
  typedef typename GraphView::view_container_type graph_t;
  graph_t* g = new graph_t(nv);
  GraphView gvw(g);
  stapl::map_func(set_partitions(group_size), gvw);

  stapl::do_once([&]() {
    stapl::generators::rand_gen randg(0);
    for (size_t i = 0; i < ne; ++i) {
      gvw.add_edge(randg.rand()%group_size,
                   randg.rand()%(nv-group_size) + group_size);
    }
    });

  return gvw;
}


struct incr_wf
{
  template<typename V>
  bool operator() (V target) const
  {
    target.property().level(target.property().level() + 1);
    return true;
  }
};


struct check_matching
{
  template<typename V, typename Visitor>
  bool operator() (V v, Visitor vis) const
  {
    if (vis.level() == 1) {
      v.property().level(0);
      return true;
    } else if (vis.level() == 2) {
      if (v.property().matched())
        vis.visit(v.property().match(), incr_wf());
      return true;
    }
    return false;
  }
};


struct all_ones
{
  typedef bool result_type;
  template<typename V>
  result_type operator() (V v) const
  {
    // A vertex should be visited once iff it is matched,
    // and unmatched vertices must never be visited.
    if ( (v.property().matched() && v.property().level() == 1) ||
         (!v.property().matched() && v.property().level() == 0) )
      return true;
    else
      return false;
  }
};


struct incr2_wf
{
  template<typename V>
  bool operator() (V target) const
  {
    if (!target.property().matched())
      target.property().level(target.property().level() + 1);
    return true;
  }
};


struct check_maximal
{
  template<typename V, typename Visitor>
  bool operator() (V v, Visitor vis) const
  {
    if (vis.level() == 1) {
      v.property().level(0);
      return true;
    } else if (vis.level() == 2) {
      if (!v.property().matched())
        vis.visit_all_edges(v, incr2_wf());
      return true;
    }
    return false;
  }
};


struct all_zeroes
{
  typedef bool result_type;
  template<typename V>
  result_type operator() (V v) const
  {
    // A vertex that is not matched must have all its neighbors matched,
    // otherwise, we could have added this edge to the matching.
    return v.property().level() ==  0;
  }
};


template <class GraphView>
void test_core_graph(GraphView& vgraph)
{
  one_print("Testing Maximal Bipartite Matching...\t");

  stapl::maximal_bipartite_matching(vgraph);

  // check if output is a valid matching.
  stapl::graph_paradigm(check_matching(), incr_wf(), vgraph);
  bool passed
    = stapl::map_reduce(all_ones(), stapl::logical_and<bool>(), vgraph);

  // check if output is maximal.
  stapl::graph_paradigm(check_maximal(), incr_wf(), vgraph);
  passed &= stapl::map_reduce(all_zeroes(), stapl::logical_and<bool>(), vgraph);

  one_print(passed);
}


template<typename G>
void print_graph(G& g)
{
  if (stapl::get_location_id() == 0) {
    for (auto v : g) {
      std::cout << v.descriptor() << " [group: " << v.property().partition()
                << "] <" << v.property().match() << ","
                << std::boolalpha << v.property().matched()<< "> : ";
      for (auto e : v) {
        std::cout << e.target() << ", ";
      }
      std::cout << std::endl;
    }
  }
}

stapl::exit_code stapl_main(int argc,char** argv)
{
  size_t nv, ne, group_size;
  if (argc > 2) {
    nv = atol(argv[1]);
    ne = atol(argv[2]);
    group_size = nv / 2;
  } else {
    cout<<"usage: exe #vertices #edges [--group_size m]\n";
    return EXIT_FAILURE;
  }

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--group_size", argv[i]))
      group_size = atoi(argv[i+1]);
  }

  typedef stapl::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES,
                       stapl::properties::mbm_property> PGR_static;
  typedef stapl::graph_view<PGR_static> graph_view_t;

  graph_view_t vw
    = generate_random_bipartite_graph<graph_view_t>(nv, ne, group_size);
  stapl::rmi_fence();

#if 0
  print_graph(vw);
#endif

  test_core_graph(vw);

#if 0
  print_graph(vw);
#endif

  return EXIT_SUCCESS;
}
