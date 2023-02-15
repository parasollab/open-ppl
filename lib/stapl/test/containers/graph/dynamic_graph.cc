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
#include <stapl/containers/graph/functional.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/lazy_graph_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/repeated_view.hpp>
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
  result_type operator()(V&& v) const
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
  void operator() (P& p) const
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
    for (typename E::adj_edge_iterator aei = e.begin(); aei != e.end(); ++aei) {
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
private:
  bool m_undirected;
public:
  typedef size_t result_type;

  test_nat_part_wf(bool undirected) : m_undirected(undirected) {}

  template<typename G>
  result_type operator() (G g) const
  {
    result_type result = 0;
    for (typename G::vertex_iterator vi = g.begin(); vi != g.end(); ++vi)
    {
      for (auto aei = (*vi).begin(); aei != (*vi).end(); ++aei)
      {
        result += (*aei).target();
        // Undirected self edges need to be counted twice because they are
        // counted twice in ref_target_total
        if (m_undirected && (*aei).source() == (*aei).target()) {
         result += (*aei).target();
        }
      }
    }
    return result;
  }
  void define_type(stapl::typer& t)
  { t.member(m_undirected); }
};

struct lazy_delete_add_wf
{
  size_t max;

  typedef void result_type;

  lazy_delete_add_wf(size_t m)
  : max(m)
  { }

  void define_type(typer& t)
  { t.member(max); }

  template <typename T, typename LazyGVw>
  void operator()(T v, LazyGVw lgv) const
  {
    if (v.descriptor() == 1
        || v.descriptor() == 2
        || v.descriptor() == 3)
      lgv.delete_vertex(v.descriptor());
    else if (v.descriptor() > 5 && v.descriptor() < 9)
      lgv.delete_vertex(v.descriptor()+1);
    else if (v.descriptor() == max-1)
      lgv.add_vertex(max+1);
  }
};

struct increment_property
{
  template <typename V>
  void operator()(V&& v) const
  { v.property().set_color(v.property().get_color() + 1); }
};

struct add_apply_check
{
  typedef bool result_type;

  size_t m_nlocs;

  add_apply_check(size_t nlocs)
    : m_nlocs(nlocs)
  { }

  template <typename V>
  result_type operator()(V v) const
  {
    size_t vd = v.descriptor();
    if (vd < m_nlocs - 2)
      return v.property().get_color() == vd;
    else if (vd < m_nlocs)
      return v.property().get_color() == vd + 1;
    else
      return v.property().get_color() == (vd - m_nlocs + 2 + 1);
  }

  void define_type(typer& t)
  {
    t.member(m_nlocs);
  }
};

template<typename Graph>
void test_graph(size_t n, size_t ef)
{
  stapl_assert(ef % 2 == 0,
      "The edge factor must be even number because we divide by 2 below.");
  Graph g;

  one_print("Testing dynamic graph add vertex...\t");
  size_t blk_sz = n/get_num_locations();
  size_t blk_st=blk_sz*get_location_id(), blk_end=blk_st+blk_sz;
  for (size_t i=blk_st; i<blk_end; ++i)
    g.add_vertex(i, i);
  rmi_fence();
  one_print(g.size() == n);

  {
    one_print("Testing dynamic graph copy construction...");
    Graph g2(g);
    one_print(g2.size() == n);
    rmi_fence();
  }

  {
    one_print("Testing dynamic graph size construction...");
    Graph g2(n);
    one_print(g2.size() == n);
    rmi_fence();
  }

  one_print("Testing dynamic graph add-edge...\t");
  std::vector<std::pair<size_t, size_t> > edges;
  size_t vertex_sum=0, edge_cnt=0, ref_target_total=0, target_total=0;
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

  one_print("Testing dynamic graph global-traversal...");
  for (typename Graph::vertex_iterator vi = g.begin(); vi != g.end(); ++vi)
  {
    vertex_sum += (*vi).descriptor();
    for (typename Graph::adj_edge_iterator aei = (*vi).begin();
         aei != (*vi).end(); ++aei)
    {
      ++edge_cnt;
      target_total += (*aei).target();
      // Undirected self edges need to be counted twice because they are
      // counted twice in ref_target_total
      if (!g.is_directed() && (*aei).source() == (*aei).target()) {
        ++edge_cnt;
        target_total += (*aei).target();
      }
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

  one_print("Testing view over dynamic graph...\t");
  graph_view<Graph> v(g);
  target_total
    = stapl::map_reduce(test_vertex_wf(!g.is_directed()),
                        stapl::plus<size_t>(), v);
  one_print(target_total == ref_target_total);

  one_print("Testing native view over dynamic graph...");
  target_total = stapl::map_reduce(test_nat_part_wf(!g.is_directed()),
                                   stapl::plus<size_t>(),
                                   stapl::native_view(v));
  one_print(target_total == ref_target_total);

  one_print("Testing property proxy...\t\t");
  one_print(g[3].property().get_color() == 35);

  one_print("Testing property proxy global access...\t");
  g[n/2+1].property().set_color(67);
  one_print(g[n/2+1].property().get_color() == 67);

  one_print("Testing vp apply async...\t\t");
  g.vp_apply_async(1, test_aggregate_apply_wf(1));
  one_print(g[1].property().get_color() == 10);

  one_print("Testing ep apply async...\t\t");
  int set_prop_value = 9898;

  if (stapl::get_location_id() == 0)
    g.ep_apply_async(typename Graph::edge_descriptor(edges[0].first,
                                                     edges[0].second),
                     test_ep_apply_wf(set_prop_value));
  stapl::rmi_fence();

  std::pair<size_t, size_t> edge_to_find
    = map_reduce(edge_to_find_wf(edges[0]), pair_plus(),
                 counting_view<size_t>(get_num_locations()));

  int ep_prop = map_reduce(edge_ident(edge_to_find), stapl::plus<int>(), v);
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

  one_print("Testing delete vertex...\t\t");
  size_t prev_count = g.size();
  stapl::rmi_fence();
  if (get_location_id() == 0)
    g.delete_vertex(0);
  stapl::rmi_fence();
  one_print((g.size() == prev_count-1));

  one_print("Testing lazy delete...\t\t\t");
  prev_count = g.size();
  stapl::rmi_fence();

  v = graph_view<Graph>(g);
  stapl::map_func(lazy_delete_add_wf(prev_count+1), v,
                  make_repeat_view(lazy_graph(g)));
  rmi_fence();
  one_print(g.size() == prev_count-5);
  stapl::rmi_fence();

  one_print("Testing graph clear...\t\t\t");
  g.clear();
  one_print(g.size() == 0);
  stapl::rmi_fence();

  one_print("Testing add_vertex after graph clear...\t");
  g.add_vertex();
  stapl::rmi_fence();
  one_print(g.size() == get_num_locations());
  stapl::rmi_fence();

  g.clear();
  one_print("Testing add_vertex(vd) after graph clear...");
  g.add_vertex(get_location_id(), get_location_id());
  stapl::rmi_fence();
  one_print(g.size() == get_num_locations());
  stapl::rmi_fence();

  one_print("Testing add_vertex(vd, vp, f)...\t");
  size_t vde = get_location_id() + get_num_locations() - 2;
  g.add_vertex(vde, get_location_id() + 1, increment_property());
  stapl::rmi_fence();
  bool pass = map_reduce(add_apply_check(get_num_locations()),
                         stapl::logical_and<bool>(),
                         graph_view<Graph>(g));
  one_print(pass && g.size() == (2*get_num_locations())-2);
  rmi_fence();
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

  one_print("***Testing DIRECTED Dynamic Graph***\n");
  test_graph<stapl::dynamic_graph<stapl::DIRECTED, stapl::MULTIEDGES,
                                  my_vertex_property, int> >(n, ef);

  one_print("***Testing UNDIRECTED Dynamic Graph***\n");
  test_graph<stapl::dynamic_graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
                          my_vertex_property, int> >(n, ef);

  return EXIT_SUCCESS;
}
