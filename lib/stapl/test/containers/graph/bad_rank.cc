/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "test_util.h"
#include <stapl/utility/do_once.hpp>
#include <stapl/views/balance_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/bad_rank.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>

using namespace stapl;
using namespace std;

size_t SCALE = 100;
size_t NUM_BL = 1500;
size_t ITER = 20;
double DAMP = 0.85;

struct rand_gen
{
  typedef std::uniform_int_distribution<int> rng_dist_t;
  std::mt19937 m_rng;

  rand_gen(unsigned int seed = 42)
    : m_rng(seed)
  { }

  int rand(int min, int max)
  { return rng_dist_t(min, max)(m_rng); }
};

struct blacklist_wf
{
  typedef void result_type;
  template<typename View0, typename View1>
  result_type operator()(View0 count, View1 graph)
  {
    std::random_device gen;

    rand_gen r(gen());
    int loc = r.rand(count.domain().first(), count.domain().last());

    graph[loc].property().set_blacklisted(true);
  }
};

struct vertex_extract_wf
{
  template<typename V>
  void operator()(V v)
    {
      std::cout << "GID: " << v.descriptor() <<
        " Rank: " << v.property().rank() <<
        (v.property().is_blacklisted() ? " Black" : "") << endl;
    }
};

struct check_ranks_wf
{
  typedef bool result_type;
  template<typename V>
  result_type operator()(V v)
  {
    bool result;

    if (v.property().is_blacklisted())
      result = (v.descriptor() == (SCALE * SCALE - 1)
          ? (v.property().rank() == DAMP / NUM_BL)
          : (v.property().rank() >= DAMP / NUM_BL));
    else
      result =  true; // Cannot be sure of rank otherwise. Typically should be
                      // lower than blacklisted vertex ranks, but not always
    return result;
  }
};

template<typename G>
void test_bad_rank(G& g)
{
  typedef graph_view<G> graph_view_t;
  graph_view_t gvw(g);

  one_print("Testing BadRank...\t\t\t");

  stapl::bad_rank(stapl::sgl::make_execution_policy("lsync", gvw),
      gvw, ITER, NUM_BL, DAMP);

#ifdef STAPL_DEBUG
  stapl::map_func(vertex_extract_wf(), gvw);
#endif

  bool passed = stapl::map_reduce(
      check_ranks_wf(), stapl::logical_and<bool>(), gvw);

  one_print(passed);
  stapl::rmi_fence();

  one_print("Testing Hierarchical BadRank...\t\t\t");

  stapl::bad_rank(stapl::sgl::make_execution_policy("hier", gvw),
      gvw, ITER, NUM_BL, DAMP);

  bool passed_h = stapl::map_reduce(
      check_ranks_wf(), stapl::logical_and<bool>(), gvw);

  one_print(passed_h);
  stapl::rmi_fence();

  one_print("Testing Hierarchical + Hubs BadRank...\t\t\t");

  // Anything with more than 3 edges is considered a hub
  stapl::bad_rank(stapl::sgl::make_execution_policy("hubs", gvw, 3),
      gvw, ITER, NUM_BL, DAMP);

  bool passed_hubs = stapl::map_reduce(
      check_ranks_wf(), stapl::logical_and<bool>(), gvw);

  one_print(passed_hubs);
  stapl::rmi_fence();

}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef stapl::multidigraph<stapl::properties::bad_rank_property> PGR_static;
  typedef graph_view<PGR_static> graph_view_t;

  stapl::do_once([&argc, &argv] {
    if (argc == 5)
    {
      SCALE = atoi(argv[1]);
      NUM_BL = atoi(argv[2]);
      DAMP = atof(argv[3]);
      ITER = atoi(argv[4]);
    }
    else
    {
      cout<<"Usage: " << argv[0] << " scale num_bl damping iterations\n"
        << "using scale = 100\n      blacklist = 1500\n      damping = 0.85\n"
        << "      iterations = 20" << endl;

    }
  });

  size_t nv = SCALE * SCALE;

  PGR_static p_static(nv);
  graph_view_t gvw(p_static);

  gvw = generators::make_mesh<graph_view_t>(gvw, SCALE, SCALE, false);
  stapl::map_func(blacklist_wf(), balance_view(counting_view(nv, 0), NUM_BL),
      make_repeat_view(gvw));
  stapl::rmi_fence();

  test_bad_rank(p_static);

  return EXIT_SUCCESS;
}
