/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/generators/single.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/numeric.hpp>

#include <boost/lexical_cast.hpp>
#include <stapl/containers/graph/algorithms/rebalance_diffusive.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>

#include "../../test_report.hpp"
#include "test_util.h"

using namespace stapl;

template<typename G>
class migrate_second_graph
{
private:
  pointer_wrapper<G> m_g;

public:
  migrate_second_graph(G& g)
    : m_g(&g)
  { }

  template<typename T, typename GID>
  void operator()(T t, GID gid, size_t location) const
  {
    std::vector<GID> related_gids = t.property();
    for (auto i : related_gids)
      m_g->migrate(i, location);
  }

  void define_type(typer& t)
  {
    t.member(m_g);
  }
};


struct set_related
{
  size_t m_num_related;

  typedef void result_type;

  set_related(size_t num) : m_num_related(num) { }

  template<typename T, typename G>
  void operator()(T t, G g)
  {
    for (size_t i = 0; i < m_num_related; ++i)
      if (rand() % 2 == 0)
        t.property().push_back(g.add_vertex());
  }

  void define_type(typer& t)
  {
    t.member(m_num_related);
  }
};


struct size_functor
{
  typedef size_t value_type;

  template<typename T>
  value_type get(T x) const
  {
    return x.size();
  }
};


template<typename W>
class transform_wf
{
  W m_w;

public:
  typedef double return_type;

  transform_wf(W const& w) : m_w(w) { }

  template<typename V>
  return_type operator()(V v)
  {
    double sum = 0.;

    for (typename V::iterator it = v.begin(); it != v.end(); ++it)
      sum += m_w.get(*it);

    return sum;
  }

  void define_type(typer& t)
  {
    t.member(m_w);
  }
};


template<typename G, typename W>
double coefficient_of_variation(const G& g, const W& w)
{
  using stapl::transform;
  using stapl::accumulate;
  using stapl::inner_product;

  stapl::array<double> a(get_num_locations());
  array_view<stapl::array<double> > v(a);
  const size_t n = v.size();
  if (n == 1)
    return 0.;

  transform(stapl::native_view(g), v, transform_wf<W>(w));

  double mean = accumulate(v, 0.) / n;

  transform(v, v, boost::bind(stapl::minus<double>(), _1, mean));
  double deviation = sqrt(inner_product(v, v, 0.) / (n-1));

  return deviation / mean;
}


struct colocation_wf
{
  typedef bool result_type;

  template<typename T, typename G>
  result_type operator()(T t, G g)
  {
    typename T::property_type related_gids = t.property();

    bool passed = true;
    for (typename T::property_type::iterator it = related_gids.begin();
         it != related_gids.end(); ++it)
      if (!g.container().distribution().container_manager().contains(*it))
        passed = false;
    return passed;
  }
};


template<typename G, typename H>
bool colocated(const G& g, const H& h)
{
  return map_reduce(colocation_wf(), stapl::logical_and<bool>(),
                    g, make_repeat_view(h));
}


void test_unit_weight(size_t n)
{
  typedef dynamic_graph<stapl::DIRECTED, stapl::MULTIEDGES> graph_type;
  typedef graph_view<graph_type> view_type;

  graph_type g;
  view_type v(g);

  for (size_t i = 0; i < n/(get_location_id()+1); ++i)
    v.add_vertex();

  rmi_fence();

  typedef graph_internal_property_map<view_type, detail::return_1>
    property_map_type;
  property_map_type weights(v, detail::return_1());
  double cv_before = coefficient_of_variation(v, weights);

  rebalance_diffusive(v);

  // rebalance_diffusive invalidates v, so we need to reconstruct it before
  // further usage
  v = view_type(g);

  double cv_after = coefficient_of_variation(v, weights);

  STAPL_TEST_REPORT(cv_before >= cv_after,
    "Testing coefficient of variation decreased for unit weight")

  rmi_fence();
}


void test_weights(size_t n, size_t m)
{
  // create two graphs and add vertices of g
  typedef dynamic_graph<stapl::DIRECTED, stapl::MULTIEDGES,
    std::vector<size_t> > graph_type;
  typedef graph_view<graph_type> view_type;
  graph_type g;
  graph_type h;

  view_type v(g);
  view_type w(h);

  v = generators::make_mesh<view_type>(v, n, m);

  // add vertices to w and associate them with vertices of v
  map_func(set_related(m), v, make_repeat_view(w));

  // create a property map where the number of associated vertices is the weight
  typedef graph_internal_property_map<view_type, size_functor>
    property_map_type;
  property_map_type weights(v, size_functor());

  double cv_before = coefficient_of_variation(v, weights);

  rebalance_diffusive(v, weights);

  // rebalance_diffusive invalidates v, so we need to reconstruct it before
  // further usage
  v = view_type(g);

  double cv_after = coefficient_of_variation(v, weights);

  STAPL_TEST_REPORT(cv_before >= cv_after,
    "Testing coefficient of variation decreased for non-unit weight")

  rmi_fence();
}


void test_associated(size_t n, size_t m)
{
  // create two graphs and add vertices of g
  typedef dynamic_graph<stapl::DIRECTED, stapl::MULTIEDGES,
    std::vector<size_t> > graph_type;
  typedef graph_view<graph_type> view_type;
  graph_type g;
  graph_type h;

  view_type v(g);
  view_type w(h);

  v = generators::make_mesh<view_type>(v, n, m);

  // add vertices to w and associate them with vertices of v
  map_func(set_related(m), v, make_repeat_view(w));

  // create a property map where the number of associated vertices is the weight
  typedef graph_internal_property_map<view_type, size_functor>
    property_map_type;
  property_map_type weights(v, size_functor());

  // set up the migration action to migrate associated vertices of w
  typedef migrate_second_graph<graph_type> migration_action_type;
  typedef single_container<migration_action_type> migration_action_map_type;

  migration_action_type a(h);
  migration_action_map_type action_function(a);

  // compute cv of v and sizes of v and w
  double cv_before = coefficient_of_variation(v, weights);
  size_t g_size_before = g.size();
  size_t h_size_before = h.size();

  size_t g_edges_before = g.num_edges();
  size_t h_edges_before = h.num_edges();

  // compute independent sets of g to consider for diffusive rebalancing
  std::vector<std::vector<std::pair<size_t, size_t> > > independent_sets =
    mesh_independent_sets(get_num_locations(), 1);

  // rebalance g which should also migrate h
  rebalance_diffusive(v, weights, action_function, independent_sets);

  // rebalance_diffusive invalidates v, so we need to reconstruct it before
  // further usage
  v = view_type(g);

  // compute new cv and sizes
  double cv_after = coefficient_of_variation(v, weights);

  STAPL_TEST_REPORT(cv_before >= cv_after,
    "Testing coefficient of variation decreased for associated graphs")

  STAPL_TEST_REPORT(g.size() == g_size_before,
    "Testing size of first graph remains unchanged")

  STAPL_TEST_REPORT(h.size() == h_size_before,
    "Testing size of second graph remains unchanged")

  STAPL_TEST_REPORT(g.num_edges() == g_edges_before,
    "Testing number of edges of first graph remains unchanged")

  STAPL_TEST_REPORT(h.num_edges() == h_edges_before,
    "Testing number of edges  of second graph remains unchanged")

  view_type v2(g);
  view_type w2(h);

  const bool colocated_result = colocated(v2, w2);

  STAPL_TEST_REPORT(colocated_result,
    "Testing colocation of first and second graph")
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cerr << "usage: exe n related_factor" << std::endl;
    exit(1);
  }

  srand(1000*stapl::get_location_id() + time(NULL));

  size_t n = boost::lexical_cast<size_t>(argv[1]);
  size_t m = boost::lexical_cast<size_t>(argv[2]);

  test_unit_weight(n);
  test_weights(n, m);
  test_associated(n, m);

  return EXIT_SUCCESS;
}
