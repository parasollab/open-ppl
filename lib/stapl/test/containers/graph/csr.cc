/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/csr_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/array/array.hpp>

#include "test_util.h"

using namespace stapl;

struct test_aggregate_apply_wf
{
  size_t m_target;
  typedef void result_type;
  test_aggregate_apply_wf(size_t t)
    : m_target(t)
  { }

  test_aggregate_apply_wf()
    : m_target(std::numeric_limits<size_t>::max())
  { }

  size_t target() const
  { return m_target; }

  template<typename P>
  void operator() (P& p) const
  { p.set_color(m_target*10); }

  void define_type(stapl::typer& t)
  { t.member(m_target); }
};

struct test_vertex_wf
{
private:
  bool m_undirected;
public:
  typedef size_t result_type;

  test_vertex_wf(bool undirected) : m_undirected(undirected) {}

  template<typename E>
  result_type operator() (E e) const
  {
    e.property().set_color(35);
    result_type result = 0;
    for (typename E::adj_edge_iterator aei = e.begin(); aei != e.end(); ++aei)
    {
      result += (*aei).target();
      // Undirected self edges need to be counted twice because they are
      // counted twice in ref_target_total
      if (m_undirected && (*aei).source() == (*aei).target()) {
        result += (*aei).target();
      }
    }
    return result;
  }

  void define_type(stapl::typer& t)
  { t.member(m_undirected); }
};


template <typename Graph>
void test_graph(size_t n, size_t ef)
{
  stapl_assert(ef % 2 == 0,
      "The edge factor must be even number because we divide by 2 below.");
  one_print("Testing CSR graph construction...\t");
  Graph g(n);
  stapl::rmi_fence();
  one_print(g.size() == n);

  {
    one_print("Testing CSR graph copy construction...\t");
    Graph g2(g);
    one_print(g2.size() == n);
  }

  one_print("Testing CSR graph add-edge...\t\t");
  size_t ref_target_total=0;

  std::vector<std::pair<size_t, size_t> > edges;

  for (size_t i=0; i<ef/2; ++i)
  {
    size_t s = rand()%(n-1);
    size_t t = rand()%(n-1);
    typename Graph::edge_descriptor ed = g.add_edge(s, t);
    edges.push_back(std::make_pair(ed.source(), ed.target()));
    ref_target_total += t;
    if (!g.is_directed())
      ref_target_total += s;
  }

  for (size_t i=0; i<ef/2; ++i)
  {
    size_t s = rand()%(n-1);
    size_t t = rand()%(n-1);
    g.add_edge_async(s, t);
    edges.push_back(std::make_pair(s,t));
    ref_target_total += t;
    if (!g.is_directed())
      ref_target_total += s;
  }
  stapl::rmi_fence();
  one_print(g.num_edges() == stapl::get_num_locations()*ef);

  typedef stapl::array<size_t> array_t;
  array_t target_totals(get_num_locations());
  stapl::array_view<array_t> av(target_totals);
  target_totals[get_location_id()] = ref_target_total;
  ref_target_total = accumulate(av, 0);

  one_print("Testing commit for CSR...\t\t");
  // commit this graph -- only used for CSR!
  g.commit();
  stapl::rmi_fence();
  one_print(true);
  stapl::rmi_fence();

  one_print("Testing operator [] :\t\t\t");
  one_print(g[n/2+1].descriptor() == n/2+1);

  one_print("Testing view over CSR graph...\t\t");
  graph_view<Graph> v(g);
  size_t target_total
    = stapl::map_reduce(test_vertex_wf(!g.is_directed()),
                        stapl::plus<size_t>(), v);
  one_print(target_total == ref_target_total);

  one_print("Testing property proxy...\t\t");
  one_print(g[3].property().get_color() == 35);

  one_print("Testing property proxy global access...\t");
  g[n/2+1].property().set_color(67);
  one_print(g[n/2+1].property().get_color() == 67);

  if (get_num_locations() > 1) {
    one_print("Testing aggregate apply async...\t");
    std::vector<test_aggregate_apply_wf> aggv;
    size_t blk_sz = n/get_num_locations();
    size_t blk_st=0, blk_end=n/2, l=0;
    // if my location is 0 and there are more locations,
    // apply to elements in location 1.
    if (stapl::get_location_id() == 0 && stapl::get_num_locations() > 1) {
      // compute elements on location 1.
      blk_st = blk_sz;
      blk_end = blk_st+blk_sz;
      l=1;

      // create the aggregate vector for all work-functions to be applied.
      for (size_t i=blk_st; i<blk_end; ++i)
        aggv.push_back(test_aggregate_apply_wf(i));

      g.aggregate_vp_apply_async(l, aggv);
    }
    stapl::rmi_fence();
    one_print(g[blk_st+1].property().get_color() == (blk_st+1)*10);
  }

  one_print("Testing vp apply async...\t\t");
  g.vp_apply_async(1, test_aggregate_apply_wf(1));
  stapl::rmi_fence();
  one_print(g[1].property().get_color() == 10);
  stapl::rmi_fence();

  one_print("Testing v.size()...\t\t");
  g.uncommit();
  g.add_edge(n-1, 0);
  stapl::rmi_fence(); // So the add_edge precedes the commit
  g.commit();
  stapl::rmi_fence(); // So size commit preceds size()
  std::size_t degree = g[n-1].size();
  one_print(degree > 0);
  stapl::rmi_fence(); // So the graph is still around
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cerr << "usage: exe n ef" << std::endl;
    exit(1);
  }
  srand(1000*stapl::get_location_id() + time(NULL));
  size_t n  = atol(argv[1]) * get_num_locations();
  size_t ef = atol(argv[2]);

  one_print("***Testing DIRECTED Graph***\n");
  test_graph<stapl::csr_graph<stapl::DIRECTED, stapl::MULTIEDGES,
                              my_vertex_property, int> >(n, ef);

  one_print("***Testing UNDIRECTED Graph***\n");
  test_graph<stapl::csr_graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
                              my_vertex_property, int> >(n, ef);

  return EXIT_SUCCESS;
}
