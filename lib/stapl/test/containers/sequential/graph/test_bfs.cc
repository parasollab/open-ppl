/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/sequential/graph/bgl_undirected_graph_adapter.hpp>
#include <stapl/containers/sequential/graph/algorithms/graph_algo.h>
#include <stapl/containers/sequential/graph/algorithms/find_cycle.h>
#include <stapl/containers/sequential/graph/algorithms/connected_components.h>
#include <stapl/containers/sequential/graph/algorithms/breadth_first_search.h>
#include "test_util.h"
#include <boost/graph/breadth_first_search.hpp>
#include <papi.h>

using namespace boost;
using namespace stapl;
using namespace stapl::sequential;
using namespace std;

namespace bfs_test {

template< class GRAPH> 
class visitor_test{
  typedef typename GRAPH::vertex_iterator   vertex_iterator;
  typedef typename GRAPH::adj_edge_iterator adj_edge_iterator;
  typedef typename GRAPH::vertex_descriptor VD;
  typedef typename GRAPH::edge_descriptor ED;
  public:

  size_t m_sum;

  visitor_test(){m_sum=0;}
  visitor_test(GRAPH& _g){m_sum=0;}

  void discover_vertex(vertex_iterator _vi){
    m_sum += (*_vi).descriptor();
    //m_sum += 1;
  }

  void examine_vertex(vertex_iterator _vi){
    m_sum -= (*_vi).descriptor();
    //m_sum -= 1;
  }

  void examine_edge(vertex_iterator _vi, adj_edge_iterator _ei){
    //m_sum += 1;
    this->m_sum += (*_ei).source();
    this->m_sum -= (*_ei).target(); 
  }
  void tree_edge(vertex_iterator _vi, adj_edge_iterator _ei){
    //m_sum += 1;
    this->m_sum += (*_ei).source();
    this->m_sum -= (*_ei).target(); 
  }

  void non_tree_edge(vertex_iterator _vi, adj_edge_iterator _ei){
    //m_sum += 1;
    this->m_sum += (*_ei).source();
    this->m_sum -= (*_ei).target();
  }

  void gray_target(vertex_iterator _vi, adj_edge_iterator _ei) {
    //m_sum += 1;
    this->m_sum += (*_ei).source();
    this->m_sum -= (*_ei).target();
  }

  void black_target(vertex_iterator _vi, adj_edge_iterator _ei){
    //m_sum += 1;
    this->m_sum += (*_ei).source();
    this->m_sum -= (*_ei).target();
  }
  
  void finish_vertex(vertex_iterator _vi, int =-1){
    //m_sum += 1;
    m_sum += (*_vi).descriptor();
  }
  ~visitor_test() { }
};

template< class GRAPH> 
class visitor_bgl{
  public:

  size_t& m_sum;

  visitor_bgl(size_t& b) : m_sum(b){
    m_sum = 0;
  }

  template < typename Vertex, typename Graph >
  void discover_vertex(Vertex u, Graph & g) {
    m_sum += u;
  }

  template < typename Vertex, typename Graph >
  void examine_vertex(Vertex u, Graph & g) {
    m_sum -= u;
  }

  template < typename Vertex, typename Graph >
  void initialize_vertex(Vertex s, Graph& g) {
  }


  template < typename Edge, typename Graph >
  void examine_edge(Edge e, Graph& g) {
    this->m_sum += source(e,g);
    this->m_sum -= target(e,g);
  }

  template < typename Edge, typename Graph >
  void tree_edge(Edge e, Graph& g) {
    this->m_sum += source(e,g);
    this->m_sum -= target(e,g);
  }

  template < typename Edge, typename Graph >
  void non_tree_edge(Edge e, Graph& g) {
    this->m_sum += source(e,g);
    this->m_sum -= target(e,g);
  }

  template < typename Edge, typename Graph >
  void black_target(Edge e, Graph& g) {
    this->m_sum += source(e,g);
    this->m_sum -= target(e,g);
  }

  template < typename Edge, typename Graph >
  void gray_target(Edge e, Graph& g) {
    this->m_sum += source(e,g);
    this->m_sum -= target(e,g);
  }

  template < typename Vertex, typename Graph >
  void finish_vertex(Vertex s, Graph& g) {
    m_sum += s;
  }

};

template <class G, class ColorMap>
void bfs_test_core_graph(G& g, ColorMap& cmap){
  typedef typename G::vertex_descriptor VD;
  stapl::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,typename G::vertex_property, double> sssptree;
  
  bool err;
  err=false;
  
  size_t tw1=0, tw2=0, tw3=0, tw4=0, tw5=0;
  cmap.reset();
  
  // Initialize the library
  PAPI_library_init(PAPI_VER_CURRENT);
  
  /////////////////STAPL_ALGO ON STAPL_GRAPH///////////////////////////////////  
  visitor_test<G> vis;
  long_long t = PAPI_get_real_usec();
  stapl::breadth_first_search(g,g.begin().descriptor(),vis,cmap);
  t = PAPI_get_real_usec() - t;
  std::cout <<"\t"<< t << ", ";
  tw1 = vis.m_sum;
  /////////////////BGL_ALGO ON STAPL_GRAPH///////////////////////////////////  
  /*
    {
    using namespace boost;
    
    std::vector<typename G::edge_property> weights_map(g.get_num_edges());
    for (typename G::edge_iterator ei = g.edges_begin(); ei != g.edges_end(); ++ei)
    weights_map[ei.descriptor().id()] = ei.property();
    
    vector<typename G::edge_property>* w_vec_ptr = const_cast<vector<typename G::edge_property>* >(&weights_map);
    
    std::vector<default_color_type> colors(num_vertices(g),
    color_traits<boost::default_color_type>::white());
    visitor_bgl<G> vis2(tw2);
    
    PAPI_library_init(PAPI_VER_CURRENT);
    long_long t = PAPI_get_real_usec();
    
    boost::breadth_first_search(g, vertex(0,g),
    boost::color_map(make_iterator_property_map(colors.begin(), get(vertex_index,g))).visitor (vis2));
    
    t = PAPI_get_real_usec() - t;
    std::cout <<"\t"<< t << ", ";
    tw2 = vis2.m_sum;
    }
    
    /////////////////BGL_ALGO ON BGL_GRAPH///////////////////////////////////  
    {
    using namespace boost;
    typedef adjacency_list <vecS,
    vecS,
    undirectedS,
    property<vertex_distance_t, int>,
    property <edge_weight_t, double> > BOOST_GRAPH;
    
    BOOST_GRAPH bg;
    
    property_map<BOOST_GRAPH, edge_weight_t>::type weightmap;
    generate_boost_graph_from_stapl(g, bg, weightmap);
    
    std::vector<default_color_type> colors(num_vertices(bg),
    color_traits<boost::default_color_type>::white());
    visitor_bgl<BOOST_GRAPH> vis3(tw5);
    
    PAPI_library_init(PAPI_VER_CURRENT);
    long_long t = PAPI_get_real_usec();
    
    boost::breadth_first_search(bg, vertex(0,bg),
    boost::color_map(make_iterator_property_map(colors.begin(), get(vertex_index,bg))).visitor (vis3));
    
    t = PAPI_get_real_usec() - t;
    std::cout << "\t"<< t << ", ";
    tw5 = vis3.m_sum;				
    }
  */
  cout << endl;
  cout << tw1 << ", "  << tw2 << ", "  << tw3 /*<< ", " << tw4 */ << ", "  << tw5 << "\n"; 
}

}  // end namespace bfs_test
