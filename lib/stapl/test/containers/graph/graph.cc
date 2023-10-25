/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/array/array.hpp>
#include <boost/lexical_cast.hpp>

#include <algorithm>

#include "test_util.h"

using namespace stapl;

template<typename Comp>
struct is_sorted_wf
{
  Comp m_comp;

  is_sorted_wf(Comp const& comp)
    : m_comp(comp)
  { }

  typedef bool result_type;

  template<typename V>
  result_type operator() (V v) const
  {
    return std::is_sorted(v.begin(), v.end(), m_comp);
  }

  void define_type(stapl::typer& t)
  { t.member(m_comp); }
};

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
  void operator() (P&& p) const
  { p.set_color(m_target*10); }

  void define_type(stapl::typer& t)
  { t.member(m_target); }
};


struct test_ep_apply_wf
{
  int m_prop;
  typedef void result_type;

  test_ep_apply_wf(int t)
    : m_prop(t)
  { }

  template<typename P>
  void operator() (P& p) const
  { p = m_prop; }

  void define_type(stapl::typer& t)
  { t.member(m_prop); }
};


struct test_ep_apply_get_wf
{
  int m_prop;
  int m_ret_val;
  typedef int result_type;

  test_ep_apply_get_wf(int t, int r)
    : m_prop(t), m_ret_val(r)
  { }

  template<typename P>
  int operator() (P& p) const
  {
    p = m_prop;
    return 1234;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_prop);
    t.member(m_ret_val);
  }
};


struct test_vertex_wf
{
  typedef size_t result_type;

  template<typename E>
  result_type operator() (E e) const
  {
    e.property().set_color(35);
    result_type result = 0;
    for (typename E::adj_edge_iterator aei = e.begin(); aei != e.end(); ++aei)
    {
      result += (*aei).target();
    }
    return result;
  }
};


struct edge_ident
{
  typedef int result_type;

  std::pair<size_t, size_t> m_edge;

  edge_ident()
    : m_edge(std::make_pair(0,0))
  { }

  edge_ident(std::pair<size_t, size_t> const& edge_pair)
    : m_edge(edge_pair)
  { }

  template<typename V>
  result_type operator() (V v) const
  {
    if (v.descriptor() == m_edge.first)
      for (typename V::adj_edge_iterator aei = v.begin(); aei != v.end(); ++aei)
        if ((*aei).target() == m_edge.second)
          return (*aei).property();
    return 0;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_edge);
  }
};


struct edge_to_find_wf
{
  typedef std::pair<size_t, size_t> result_type;

  result_type m_edge;

  edge_to_find_wf()
    : m_edge(std::make_pair(0,0))
  { }

  edge_to_find_wf(std::pair<size_t, size_t> const& edge)
    : m_edge(edge)
  { }

  template<typename T>
  result_type operator() (T t)
  {
    if (t == (size_t)0)
      return m_edge;
    else return std::make_pair(0,0);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_edge);
  }
};


struct pair_plus
{
  typedef std::pair<size_t, size_t> result_type;

  template<typename T>
  result_type operator() (T const& t1, T const& t2)
  {
    return std::make_pair(t1.first + t2.first,
                          t1.second + t2.second);
  }
};


struct test_nat_part_wf
{
  typedef size_t result_type;

  template<typename G>
  result_type operator() (G g) const
  {
    result_type result = 0;
    for (typename G::vertex_iterator vi = g.begin(); vi != g.end(); ++vi)
    {
      for (typename G::adj_edge_iterator aei = (*vi).begin();
           aei != (*vi).end(); ++aei)
      {
        result += (*aei).target();
      }
    }
    return result;
  }
};



struct reserve_adjacents_wf
{
  size_t m_num_reserved;

  typedef void result_type;

  reserve_adjacents_wf(size_t num_reserved)
    : m_num_reserved(num_reserved)
  { }

  template<typename V, typename G>
  void operator() (V v, G g) const
  {
    g.container().reserve_adjacency(v.descriptor(), m_num_reserved);
  }

  void define_type(typer& t)
  { t.member(m_num_reserved); }
};


template <typename Graph>
void test_graph(size_t n, size_t ef)
{
  one_print("Testing size_t, mapper constructor...\t");
  typedef typename Graph::mapper_type mapper_tp;
  typedef typename Graph::partition_type part_tp;
  typedef typename Graph::domain_type dom_tp;

  dom_tp dom(0, n-1, true);
  part_tp part(dom, get_num_locations());
  mapper_tp mapper1(part.domain());
  Graph gdh(n, mapper1);
  one_print(gdh.size() == n);

  one_print("Testing static graph construction...\t");
  Graph g(n);
  stapl::rmi_fence();
  one_print(g.size() == n);

  {
    one_print("Testing static graph copy construction...");
    Graph g2(g);
    one_print(g2.size() == n);
    rmi_fence();
  }

  one_print("Testing static graph add-edge...\t");
  std::vector<std::pair<size_t, size_t> > edges;

  size_t ef_loc = ef/2;
  size_t vertex_sum=0, edge_cnt=0, ref_target_total=0, target_total=0;

  rmi_fence();
  graph_view<Graph> v(g);
  rmi_fence();
  map_func(reserve_adjacents_wf(ef), v, make_repeat_view(v));
  rmi_fence();

  if (get_location_id() == 0) {
    // add a local-edge for loc-0:
    g.add_edge(0, 1);
    // add a remote-edge for loc-0:
    g.add_edge(n-1, n-2);

    edges.push_back(std::make_pair(0, 1));
    edges.push_back(std::make_pair(n-1, n-2));

    ref_target_total += (n-2);
    ref_target_total += 1;
    if (!g.is_directed())
      ref_target_total += (n-1);

    // add two less edges if on loc-0 to compensate for these two:
    ef_loc -= 2;
  }

  for (size_t i=0; i<ef_loc; ++i)
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
  one_print(g.num_edges_collective() == stapl::get_num_locations()*ef);

  // Test for sorted edges after add_edge.
  one_print("Testing sorted edges post add_edge...\t");
  g.sort_edges();
  typedef detail::edge_target_location_comp<Graph> edge_comp_t;
  edge_comp_t edge_comp(&g);

  bool is_sorted_edges
    = stapl::map_reduce(is_sorted_wf<edge_comp_t>(edge_comp),
                        stapl::logical_and<bool>(),
                        graph_view<Graph>(g));
  one_print(is_sorted_edges == true);

  stapl::array<size_t> ref_tot_arr(get_num_locations());
  ref_tot_arr[get_location_id()] = ref_target_total;
  ref_target_total
    = accumulate(array_view<array<size_t> >(ref_tot_arr), 0);

  one_print("Testing static graph global-traversal...");
  for (typename Graph::vertex_iterator vi = g.begin(); vi != g.end(); ++vi)
  {
    vertex_sum += (*vi).descriptor();
    for (typename Graph::adj_edge_iterator aei = (*vi).begin();
         aei != (*vi).end(); ++aei)
    {
      ++edge_cnt;
      target_total += (*aei).target();
    }
  }

  if (g.is_directed())
    one_print( (edge_cnt==stapl::get_num_locations()*ef)
               && (vertex_sum==(n*(n-1))/2)
               && (target_total == ref_target_total));
  else
    one_print( (edge_cnt==stapl::get_num_locations()*ef*2)
               && (vertex_sum==(n*(n-1))/2)
               && (target_total == ref_target_total));

  stapl::rmi_fence();

  one_print("Testing operator [] :\t\t\t");
  one_print(g[n/2+1].descriptor() == n/2+1);

  one_print("Testing view over static graph...\t");
  target_total = stapl::map_reduce(test_vertex_wf(), stapl::plus<size_t>(), v);
  one_print(target_total == ref_target_total);

  one_print("Testing native view over static graph...");
  target_total = stapl::map_reduce(test_nat_part_wf(), stapl::plus<size_t>(),
                                   stapl::native_view(v));
  one_print(target_total == ref_target_total);

  // write graph to file.
  if (g.is_directed())
    write_edge_list(v, "graph.out");

  rmi_fence();

  // read graph from file.
  if (g.is_directed()) {
    size_t blk_sz = 23;
    one_print("Testing graph I/O...\t\t\t");
    graph_view<Graph> x = read_edge_list<Graph>("graph.out", blk_sz);

    target_total = stapl::map_reduce(test_nat_part_wf(), stapl::plus<size_t>(),
                                     stapl::native_view(x));

    one_print((x.size() == v.size()) &&
              (x.num_edges() == v.num_edges()) &&
              (target_total == ref_target_total));

    rmi_fence();
  }

  one_print("Testing property proxy...\t\t");
  one_print(g[3].property().get_color() == 35);

  one_print("Testing property proxy global access...\t");
  g[n/2+1].property().set_color(67);
  one_print(g[n/2+1].property().get_color() == 67);

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

  one_print("Testing vp apply async...\t\t");
  g.vp_apply_async(1, test_aggregate_apply_wf(1));
  one_print(g[1].property().get_color() == 10);

  one_print("Testing ep apply async [local]...\t");
  int set_prop_value = 9898;

  if (stapl::get_location_id() == 0) {
    // one for the local-edge:
    g.ep_apply_async(typename Graph::edge_descriptor(edges[0].first,
                                                     edges[0].second),
                     test_ep_apply_wf(set_prop_value));
    // one for the remote-edge:
    g.ep_apply_async(typename Graph::edge_descriptor(edges[1].first,
                                                     edges[1].second),
                     test_ep_apply_wf(set_prop_value));
  }
  stapl::rmi_fence();

  std::pair<size_t, size_t> edge_to_find
    = map_reduce(edge_to_find_wf(edges[0]), pair_plus(),
                 counting_view<size_t>(get_num_locations()));

  int ep_prop = map_reduce(edge_ident(edge_to_find), stapl::plus<int>(), v);
  one_print(ep_prop == set_prop_value);

  one_print("Testing ep apply async [remote]...\t");
  edge_to_find
    = map_reduce(edge_to_find_wf(edges[1]), pair_plus(),
                 counting_view<size_t>(get_num_locations()));

  ep_prop = map_reduce(edge_ident(edge_to_find), stapl::plus<int>(), v);
  one_print(ep_prop == set_prop_value);

  one_print("Testing ep apply [local]...\t\t");
  int local_ep_apply_result = 0, remote_ep_apply_result = 0;
  int applied_ret = 1234;
  set_prop_value = 8989;

  if (stapl::get_location_id() == 0) {
    // one for the local-edge:
    local_ep_apply_result
      = g.ep_apply(typename Graph::edge_descriptor(edges[0].first,
                                                   edges[0].second),
                   test_ep_apply_get_wf(set_prop_value, applied_ret));
    // one for the remote-edge:
    remote_ep_apply_result
      = g.ep_apply(typename Graph::edge_descriptor(edges[1].first,
                                                   edges[1].second),
                   test_ep_apply_get_wf(set_prop_value, applied_ret));
  }
  stapl::rmi_fence();

  edge_to_find
    = map_reduce(edge_to_find_wf(edges[0]), pair_plus(),
                 counting_view<size_t>(get_num_locations()));

  ep_prop = map_reduce(edge_ident(edge_to_find), stapl::plus<int>(), v);
  one_print(ep_prop == set_prop_value && local_ep_apply_result == applied_ret);

  one_print("Testing ep apply [remote]...\t\t");
  edge_to_find
    = map_reduce(edge_to_find_wf(edges[1]), pair_plus(),
                 counting_view<size_t>(get_num_locations()));

  ep_prop = map_reduce(edge_ident(edge_to_find), stapl::plus<int>(), v);
  one_print(ep_prop == set_prop_value && remote_ep_apply_result == applied_ret);


  one_print("Testing delete edges...\t\t\t");
  for (size_t i=0; i<edges.size(); ++i)
    g.delete_edge(edges[i].first, edges[i].second);
  stapl::rmi_fence();
  one_print((g.num_edges_collective() == 0) && (g.num_local_edges() == 0));
  stapl::rmi_fence();
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cerr << "usage: exe n ef" << std::endl;
    exit(1);
  }
  srand(1000*stapl::get_location_id() + time(NULL));
  size_t n  = boost::lexical_cast<size_t>(argv[1]) * get_num_locations();
  size_t ef = atoi(argv[2]);

  one_print("***Testing DIRECTED Graph***\n");
  test_graph<stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES,
                          my_vertex_property, int> >(n, ef);

  one_print("***Testing UNDIRECTED Graph***\n");
  test_graph<stapl::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
                          my_vertex_property, int> >(n, ef);

  return EXIT_SUCCESS;
}
