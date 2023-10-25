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
#include <stapl/containers/sequential/graph/algorithms/mst_graph.h>
#include "test_util.h"
#include <boost/graph/prim_minimum_spanning_tree.hpp>

using namespace boost;
using namespace stapl;
using namespace std;
using namespace __gnu_cxx;

namespace mst_test {

template <class G, class ColorMap>
void mst_test_core_graph(G& g, ColorMap& cmap){
  typedef typename G::vertex_descriptor VD;
  stapl::sequential::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,typename G::vertex_property, double> mst_graph;
  
  bool err;
  err=false;
  
  double tw1=0, tw2=0, tw3=0, tw4=0, tw5=0;

  /////////////////STAPL_ALGO ON STAPL_GRAPH///////////////////////////////////  

  cout<<"\ntest prims_mst....." << endl;
  // Initialize the library
  // PAPI_library_init(PAPI_VER_CURRENT);
  // long_long t = PAPI_get_real_usec();

  vector<VD> parentmap(g.get_num_vertices());
  vector<double> weightmap(g.get_num_vertices());
  cout << "calling the actual algo" << endl;
  mst_prims(g, parentmap, weightmap, 0);

  // t = PAPI_get_real_usec() - t;
  // std::cout <<"\t"<< t << ", " << endl; 

  stapl::convert_to_graph(parentmap, weightmap, mst_graph);

  cmap.reset();
  if((mst_graph.get_num_vertices() == g.get_num_vertices())
     && (mst_graph.get_num_vertices() == mst_graph.get_num_edges()+1)
     && (is_cycle(mst_graph, cmap) == false)) {
    cmap.reset();
    if (get_cc_count(mst_graph, cmap) == 1) {
      tw1 = total_weight(mst_graph);
      cout<<"<PASSED>\t"; // << endl;
    }
  } else {
    cout<<"{FAILED}\t";
    cmap.reset();
  }
  if(N < 15) display1(mst_graph);

  /*		
  //cout<<"\ntest sollins_mst.....";
  mst_graph.clear();
  mst_sollins(g, mst_graph, 0);
  cmap.reset();
  if((mst_graph.get_num_vertices() == g.get_num_vertices())
     && (mst_graph.get_num_vertices() == mst_graph.get_num_edges()/2+1)
     && (is_cycle(mst_graph, cmap) == false)) {
    cmap.reset();
    if (get_cc_count(mst_graph, cmap) == 1) {
      tw2 = total_weight(mst_graph)/2;
      cout<<"<PASSED>\t"; // << endl;
    }
  } else {
    cout<<"{FAILED}\t";
    cmap.reset();
  }
  if(N < 15) display1(mst_graph);
  */
		/*
  //cout<<"\ntest edges_mst.....";
  mst_graph.clear();
  MST_Edges(g, mst_graph, 0);
  cmap.reset();
  if((mst_graph.get_num_vertices() == g.get_num_vertices())
     && (mst_graph.get_num_vertices() == mst_graph.get_num_edges()/2+1)
     && (is_cycle(mst_graph, cmap) == false)) {
    cmap.reset();
    if (get_cc_count(mst_graph, cmap) == 1) {
      tw4 = total_weight(mst_graph)/2;
      cout<<"<PASSED>\t"; // << endl;
    }
  } else {
    cout<<"{FAILED}\t";
    cmap.reset();
    if(N < 15) display1(mst_graph);
  }
  */

  /*
/////////////////BGL_ALGO ON STAPL_GRAPH///////////////////////////////////  
  std::vector < typename G::vertex_descriptor >
    p(g.get_num_vertices());
  
  hash_map<size_t, double> distances(g.get_num_vertices());  // better performance.
  //std::map<typename G::vertex_descriptor, int> distances;
  boost::associative_property_map< hash_map<size_t, double> >
    dist_map(distances);
  
  {
    using namespace boost;
    
    // boost::property_map<G, edge_weight_t>::type weightmap = boost::get(edge_weight, g);
    // boost::property_map<G, vertex_distance_t>::type dist_map = get(vertex_distance, g);
    
    std::vector<typename G::edge_property> weights_map(g.get_num_edges());
    for (typename G::edge_iterator ei = g.edges_begin(); ei != g.edges_end(); ++ei)
      weights_map[ei.descriptor().id()] = ei.property();
    
    vector<typename G::edge_property>* w_vec_ptr = const_cast<vector<typename G::edge_property>* >(&weights_map);
    
    PAPI_library_init(PAPI_VER_CURRENT);
    long_long t = PAPI_get_real_usec();
    
    prim_minimum_spanning_tree(g, *vertices(g).first, &p[0], dist_map,
                               stapl_graph_edge_wt_id_map<typename G::vertex_property,
                               typename G::edge_property>(const_cast<G*>(&g),
                                                          w_vec_ptr),
                               stapl_graph_id_map<typename G::vertex_property, typename G::edge_property>(),
                               default_dijkstra_visitor());
    
    t = PAPI_get_real_usec() - t;
    std::cout <<"\t"<< t << ", ";
    
    stapl::graph<stapl::DIRECTED,stapl::MULTIEDGES,typename G::vertex_property, int> boost_out_graph;
    convert_to_graph(p, distances, boost_out_graph);
    
    cmap.reset();
    if((boost_out_graph.get_num_vertices() == g.get_num_vertices())
       && (boost_out_graph.get_num_vertices() == boost_out_graph.get_num_edges()+1)
       && (is_cycle(boost_out_graph, cmap) == false)) {
      cmap.reset();
      if (get_cc_count(boost_out_graph, cmap) == 1) {
        tw3 = total_weight(boost_out_graph);
        cout<<"<PASSED>\t"; // << endl;
      }
    } else {
      cout<<"{FAILED}\t";
      cmap.reset();
    }
    if(N < 15) display2(boost_out_graph);
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
    
    PAPI_library_init(PAPI_VER_CURRENT);
    long_long t = PAPI_get_real_usec();
    std::vector < graph_traits < BOOST_GRAPH >::vertex_descriptor >
      parentmap(num_vertices(bg));
    property_map<BOOST_GRAPH, vertex_distance_t>::type distmap = get(vertex_distance, bg);
    property_map<BOOST_GRAPH, vertex_index_t>::type indexmap = get(vertex_index, bg);
    
    //vector<VD> parentmap(g.get_num_vertices());
    //vector<double> distmap(g.get_num_vertices());
    //my_mst_prims(bg, parentmap, distmap, 0);
    
    prim_minimum_spanning_tree
      (bg, *vertices(bg).first, &parentmap[0], distmap, weightmap, indexmap, 
       default_dijkstra_visitor());
    
    t = PAPI_get_real_usec() - t;
    std::cout << "\t"<< t << ", ";
    

    stapl::graph<stapl::DIRECTED,stapl::MULTIEDGES,typename G::vertex_property, int> boost_out_graph;
    convert_to_graph(parentmap, distmap, boost_out_graph);
    
    cmap.reset();
    if((boost_out_graph.get_num_vertices() == g.get_num_vertices())
       && (boost_out_graph.get_num_vertices() == boost_out_graph.get_num_edges()+1)
       && (is_cycle(boost_out_graph, cmap) == false)) {
      cmap.reset();
      if (get_cc_count(boost_out_graph, cmap) == 1) {
        tw5 = total_weight(boost_out_graph);
        cout<<"<PASSED>\t"; // << endl;
      }
    } else {
      cout<<"{FAILED}\t";
      cmap.reset();
    }
    if(N < 15) display1(boost_out_graph);
}
  
  ///////////END///////
*/
  
  cout << endl;
  cout << tw1 << ", "  << tw2 << ", "  << tw3 /*<< ", " << tw4 */ << ", "  << tw5 << "\n"; 
}

}  //end namespace mst_test
