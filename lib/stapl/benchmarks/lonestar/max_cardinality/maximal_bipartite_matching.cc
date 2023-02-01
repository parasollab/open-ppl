/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

//////////////////////////////////////////////////////////////////////
/// @file
/// The Maximal Bipartite Matching benchmark from the Lonestar benchmark suite.
///
/// The benchmark computes the maximal matching of the given bipartite input
/// graph.
//////////////////////////////////////////////////////////////////////

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/maximal_bipartite_matching.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/torus.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/utility/do_once.hpp>

#include "../utility/test_util.h"

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
    if (v.descriptor() % 2 == 0 && v.descriptor() < 2*m_sz)
      v.property().partition(stapl::properties::bipartite_partition::LEFT);
    else
      v.property().partition(stapl::properties::bipartite_partition::RIGHT);
  }

  void define_type(stapl::typer& t)
  { t.member(m_sz); }
};


class Rank
{
  size_t m_start;

public:
  Rank(size_t start)
    : m_start(start)
  { }

  size_t operator()(size_t const& idx) const
  {
    if (m_start == 0) {
      return 2*idx;
    } else {
      return 2*idx+1;
    }
  }
};


template <class GraphView>
GraphView generate_input_bipartite_graph(
  size_t num_a, size_t num_b, size_t num_edges, size_t num_groups)
{
  typedef typename GraphView::view_container_type graph_t;
  graph_t* g = new graph_t(num_a + num_b);
  GraphView gvw(g);
  stapl::map_func(set_partitions(num_a), gvw);

  size_t d = num_edges/num_a;
  if (num_groups > num_a)
    num_groups = num_a;
  if (num_groups > num_b)
    num_groups = num_b;

  // Each location produces the same sequence of random numbers,
  // and filters the edges needed for local addition.
  stapl::generators::rand_gen randg(0);

  Rank A(0);
  Rank B(num_a);

  size_t count = 0;
  if (num_groups > 0) {
    size_t a_size = num_a/num_groups;
    size_t b_size = num_b/num_groups;

    for (size_t ii = 0; ii < num_a; ++ii, ++count) {
      size_t group = count/a_size;
      if (group == num_groups)
        break;
      size_t base1 = group == 0 ? (num_groups-1)*b_size : (group-1)*b_size;
      size_t base2 = group == num_groups-1 ? 0 : (group+1)*b_size;
      for (size_t i = 0; i < d; ++i) {
        size_t b = randg.rand(100) < 50 ? base1 : base2;
        size_t off = randg.rand(b_size);
        if (gvw.container().is_local(A(ii)))
          gvw.add_edge_async(A(ii), B(b+off));
      }
    }
  }

  size_t r = num_edges - count*d;
  while (r--) {
    size_t ind_a = randg.rand(num_a);
    size_t ind_b = randg.rand(num_b);
    if (gvw.container().is_local(A(ind_a)))
      gvw.add_edge_async(A(ind_a), B(ind_b));
  }

  return gvw;
}


//////////////////////////////////////////////////////////////////////
/// @brief Increments the property of the target vertex by one.
//////////////////////////////////////////////////////////////////////
struct increment_wf
{
  template<typename V>
  bool operator() (V target) const
  {
    target.property().level(target.property().level() + 1);
    return true;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Increments the property of the target vertex by one if the
/// target is not matched.
//////////////////////////////////////////////////////////////////////
struct increment_if_unmatched_wf
{
  template<typename V>
  bool operator() (V target) const
  {
    if (!target.property().matched())
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
        vis.visit(v.property().match(), increment_wf());
      return true;
    }
    return false;
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
        vis.visit_all_edges(v, increment_if_unmatched_wf());
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
std::pair<bool, std::pair<double, size_t> >
run(GraphView& vgraph, size_t k)
{
  stapl::counter<stapl::default_timer> t;
  t.start();
  size_t max_iter = stapl::maximal_bipartite_matching(vgraph, k);
  double time = t.stop();

  // check if output is a valid matching.
  stapl::graph_paradigm(check_matching(), increment_wf(), vgraph);
  bool passed
    = stapl::map_reduce(all_ones(), stapl::logical_and<bool>(), vgraph);

  // check if output is maximal.
  stapl::graph_paradigm(check_maximal(), increment_wf(), vgraph);
  passed &= stapl::map_reduce(all_zeroes(), stapl::logical_and<bool>(), vgraph);

  return std::make_pair(passed, std::make_pair(time, max_iter));
}


template<typename G>
void print_graph(G& g)
{
  stapl::do_once([&]() {
    for (auto v : g) {
      std::cout << v.descriptor() << " [group: " << v.property().partition()
                << "] <" << v.property().match() << ","
                << std::boolalpha << v.property().matched()<< "> : ";
      for (auto e : v) {
        std::cout << e.target() << ", ";
      }
      std::cout << std::endl;
    }
    });
}


stapl::exit_code stapl_main(int argc,char** argv)
{
  size_t nv, ne, group_size, k=0, num_exp=1;
  if (argc > 3) {
    nv = atol(argv[1]);
    ne = atol(argv[2]);
    group_size = atol(argv[3]);
  } else {
    cout<<"usage: exe #vertices #edges group_size [--k k]\n";
    return EXIT_FAILURE;
  }

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--k", argv[i]))
      k = atoi(argv[i+1]);
    if (!strcmp("--nexp", argv[i]))
      num_exp = atoi(argv[i+1]);
  }

  typedef stapl::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES,
                       stapl::properties::mbm_property> PGR_static;
  typedef stapl::graph_view<PGR_static> graph_view_t;

  stapl::counter<stapl::default_timer> t;
  t.start();
  graph_view_t vw
    = generate_input_bipartite_graph<graph_view_t>(nv, nv, ne, group_size);
  stapl::rmi_fence();
  vw.sort_edges();
  double gen_time = t.stop();
  stapl::do_once([&]() {
      std::cout << "Graph: " << vw.size() << " vertices, "
                << vw.num_edges() << " edges." << std::endl
                << "Generation Time= " << gen_time << " seconds" << std::endl;
    });


#if 0
  print_graph(vw);
#endif

  stapl::do_once([&]() {
      std::cout << "Testing Maximal Bipartite Matching...\t\t";
    });

  bool passed = true;
  size_t max_iter = 0;
  std::vector<double> result_vec;
  for (size_t i=0; i<num_exp; ++i) {
    auto result = run(vw, k);
    result_vec.push_back(result.second.first);
    passed &= result.first;
    max_iter += result.second.second;
  }

  stapl::do_once([&]() {
      std::string pass_str = passed ? "PASSED" : "FAILED";
      std::cout << pass_str << std::endl
                << "Maximal Cardinality= " << max_iter/num_exp << std::endl;
    });


  compute_stats("maximal_cardinality_matching", result_vec);

#if 0
  print_graph(vw);
#endif

  return EXIT_SUCCESS;
}
