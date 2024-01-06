/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_EULER_TOUR_HPP
#define STAPL_ALGORITHMS_EULER_TOUR_HPP

#include <stapl/map.hpp>
#include <stapl/vector.hpp>
#include <stapl/domains/continuous.hpp>
#include <stapl/containers/partitions/explicit.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/views/repeated_view.hpp>

namespace stapl {

template<typename T>
struct edge_descriptor_impl;

template<>
struct hash<edge_descriptor_impl<size_t> >
{
  std::size_t operator()(const edge_descriptor_impl<size_t>& t) const
  {
    return t.source()*t.target();
  }
};

template<>
struct index_bounds<edge_descriptor_impl<size_t> >
{
  static edge_descriptor_impl<size_t>  invalid ()
  {
    return edge_descriptor_impl<size_t>(stapl::index_bounds<size_t>::invalid(),
                                        stapl::index_bounds<size_t>::invalid());
  }
};

namespace algo_details {
//////////////////////////////////////////////////////////////////////
/// @brief Find the next Edge
/// @tparam Edge The edge type used.
//////////////////////////////////////////////////////////////////////
template<typename Edge>
struct find_next
{
  typedef Edge ED;

  template<typename PGView>
  ED operator()(ED e, PGView& pgview)
  {
    typedef typename PGView::vertex_iterator             vertex_iterator;
    typedef typename PGView::vertex_descriptor           VD;
    typedef typename PGView::adj_edge_iterator           adj_edge_iterator;
    //successor function
    VD source = e.source();
    VD target = e.target();
    VD v;
    vertex_iterator vi = pgview.find_vertex(target);

    // This copy of the edgelist can be replaced with iteration over
    // the edgelist when gForge Bug 778 is resolved.
    typename vertex_iterator::value_type::adj_edges_type edges((*vi).edges());
    adj_edge_iterator it_source_found = graph_find(edges.begin(), edges.end(),
                                 eq_target<VD>(source));
    //if it_source_find is the last edge in the adjacent list
    if (++it_source_found == edges.end())
      v = edges.begin()->target();
    else
      v = it_source_found->target();
    return ED(target, v);
  }
};

struct apply_val
{
  typedef void result_type;

  template<typename T1, typename T2>
  void operator()(T1& t, T2 const& nval) const
  {
    t.second = nval.second;
  }
};

////////////////////////////////////////////////////////////////////
// @brief Work function applying the new rank in the list-ranking
////////////////////////////////////////////////////////////////////
template<typename MapListView>
struct apply_rank
{
  typedef typename MapListView::value_type T;
  mutable MapListView m_out;
  T m_x;

  apply_rank(MapListView out, T x)
    : m_out(out), m_x(x)
  {}

  template<typename U>
  void operator()(U y) const
  {
    typedef typename U::second_type::first_type type1;//size_t
    typedef typename U::second_type::second_type type2;//ED

    type1 rank = y.second.first;
    type2 next = y.second.second;

    //m_out[key]=value;  not working ??
    m_out.insert(m_x.first,
                std::make_pair(rank+m_x.second.first, next),
                apply_val());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_x);
    t.member(m_out);
  }
};

////////////////////////////////////////////////////////////////////
// @brief Work function performing a list-ranking
////////////////////////////////////////////////////////////////////
struct list_rank
{
  template<typename Pair, typename MapListView>
  void operator()(Pair p, MapListView read, MapListView write)
  {
    typedef typename MapListView::value_type value;
    typedef typename value::second_type::second_type type2;//ED

    type2 next = p.second.second;
    value val = p;

    read.apply_set(next, apply_rank<MapListView>(write, val));
  }
};

////////////////////////////////////////////////////////////////////
// @brief Work function initializing the list-ranking
////////////////////////////////////////////////////////////////////
template<typename MapListView, typename ED, typename PGView>
struct list_rank_init
{
  MapListView m_read;
  ED m_fin;
  PGView m_pgview;

  list_rank_init(MapListView read, ED fin, PGView pgview)
    : m_read(read), m_fin(fin), m_pgview(pgview)
  {}

  template<typename Vertex>
  void operator()(Vertex vertex)
  {
    typedef typename PGView::vertex_iterator             vertex_iterator;
    typedef typename PGView::adj_edge_iterator           adj_edge_iterator;
    typedef typename PGView::edge_descriptor             ED;

    ED e0, e;

    // This copy of the edgelist can be replaced with iteration over
    // the edgelist when gForge Bug 778 is resolved.
    typename vertex_iterator::value_type::adj_edges_type edges(vertex.edges());
    for (adj_edge_iterator aei = edges.begin(); aei != edges.end(); ++aei) {
      e0 = reverse((*aei).descriptor());
      if (e0 == m_fin){
        m_read.insert(e0, std::make_pair(0, e0));
      }
      else{
        find_next <ED>fn;
        e = fn(e0, m_pgview);
        m_read.insert(e0, std::make_pair(1, e));
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_read);
    t.member(m_fin);
  }
};

////////////////////////////////////////////////////////////////////
// @brief Work function returning the rank of an element.
////////////////////////////////////////////////////////////////////
template<typename T>
struct get_rank
{
  typedef size_t result_type;

  template<typename Pair>
  size_t operator()(Pair p)
  {
    T x = p;
    return x.second.first;
  }
};

////////////////////////////////////////////////////////////////////
// @brief Work function storing the result of the list-ranking in a vector.
// @param m_inv
////////////////////////////////////////////////////////////////////
template<typename T>
struct tour
{
  size_t m_inv;

  tour(size_t inv)
    : m_inv(inv)
  {}

  template<typename Pair, typename PVView>
  void operator()(Pair p, PVView pvview)
  {
    T value = p;
    pvview[m_inv - value.second.first] = value.first;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_inv);
  }
};

} //namespace algo_details

//////////////////////////////////////////////////////////////////////
/// @brief The Euler tour algorithm traverses each of the graph edges only once.
/// @param pgview Input graph.
/// @param pvview Vector which will contain a cycle of vertices.
///
/// The graph has to be connected.
//////////////////////////////////////////////////////////////////////
template<typename PGView, typename PVView>
void euler_tour(PGView pgview, PVView pvview)
{
  typedef typename PGView::edge_descriptor                   ED;
  typedef typename PGView::vertex_iterator                  vertex_iterator;

  //------------- map -------------
  typedef continuous_domain<ED>                              domain_type;
  typedef explicit_partition<domain_type>                    partition_type;
  typedef stapl::map<ED, std::pair<size_t, ED>, stapl::less<ED>, partition_type>
                                                          map_rank_type;
  typedef map_view<map_rank_type>                         map_rank_view;
  typedef typename map_rank_view::value_type              map_rank_view_value;

  //------------- map -------------
  size_t N = pgview.num_vertices();
  domain_type dom(ED(0,0), ED(N-1,N-1));
  std::vector<domain_type> doms;
  size_t num_loc = get_num_locations();

  for (size_t i = 0; i < num_loc-1; ++i){
    doms.push_back(domain_type(ED(i*N/num_loc, i*N/num_loc),
                   ED((i+1)*N/num_loc, (i+1)*N/num_loc-1)));
  }
  doms.push_back(domain_type(ED((num_loc-1)*N/num_loc, (num_loc-1)*N/num_loc),
                   ED(N-1, N-1)));
  partition_type part(dom, doms);

//-------------List Ranking-----------
  //------------- map -------------
  map_rank_type read(part);
  map_rank_type write(part);
  map_rank_view read_view(read);
  map_rank_view write_view(write);
  map_rank_view temp;

  typename vertex_iterator::value_type::adj_edges_type
                                              edges((*pgview.begin()).edges());

  algo_details::list_rank_init<map_rank_view, ED, PGView>
    init(read_view, (*edges.begin()).descriptor(), pgview);
  map_func(init, pgview);

  size_t sum=0;
  size_t n = 2*pgview.num_edges()-1;
  while (sum < n*(n+1)/2){
    map_func(algo_details::list_rank(),
                     read_view,
                     make_repeat_view(read_view),
                     make_repeat_view(write_view));
    temp = read_view;
    read_view = write_view;
    write_view = temp;
    sum = map_reduce(algo_details::get_rank<map_rank_view_value>(),
                     stapl::plus<size_t>(), read_view);
  }

//-------------Storing results in Vector View--------------
  map_func(algo_details::tour<map_rank_view_value>(n),
           read_view, make_repeat_view(pvview));
}

} //namespace stapl

#endif
