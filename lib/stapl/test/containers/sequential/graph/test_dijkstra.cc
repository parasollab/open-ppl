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
#include <stapl/containers/sequential/graph/algorithms/dijkstra.h>
#include "test_util.h"
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/unordered_map.hpp>
#include <papi.h>

using namespace boost;
using namespace stapl;
using namespace std;
using namespace __gnu_cxx;

namespace dijkstra_test {

template <class G, class ColorMap>
void dijkstra_test_core_graph(G& g, ColorMap& cmap){
  typedef typename G::vertex_descriptor VD;
  stapl::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,typename G::vertex_property, double> sssptree;
  
  bool err;
  err=false;
  
  double tw1, tw2, tw3, tw4, tw5;

  // Initialize the library
  PAPI_library_init(PAPI_VER_CURRENT);

		/////////////////STAPL_ALGO ON STAPL_GRAPH///////////////////////////////////  
  long_long t = PAPI_get_real_usec();

		vector<VD> parentmap(g.get_num_vertices());
		vector<double> weightmap(g.get_num_vertices());

  dijkstra_sssp(g, parentmap, weightmap, 0);
  t = PAPI_get_real_usec() - t;
  std::cout <<"\t"<< t << ", "; 

		stapl::convert_to_graph(parentmap, weightmap, sssptree);

  cmap.reset();
  if((sssptree.get_num_vertices() == g.get_num_vertices())
     && (sssptree.get_num_vertices() == sssptree.get_num_edges()+1)
     && (is_cycle(sssptree, cmap) == false)) {
    cmap.reset();
    if (get_cc_count(sssptree, cmap) == 1) {
      tw1 = total_weight(sssptree);
      cout<<"<PASSED>\t"; // << endl;
    }
  } else {
    cout<<"{FAILED}\t";
    cmap.reset();
  }
  if(N < 15) display1(sssptree);

		/////////////////BGL_ALGO ON STAPL_GRAPH///////////////////////////////////  
  std::vector < typename G::vertex_descriptor >
    p(g.get_num_vertices());

  typedef boost::unordered_map<size_t, int> map_type;
  //typedef std::map<size_t, int> map_type;
  map_type distances(g.get_num_vertices());  // better performance.
  boost::associative_property_map<map_type> dist_map(distances);

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

				std::less<double> lsop;
				std::plus<double> combine;

				dijkstra_shortest_paths
						(g, *vertices(g).first, &p[0], dist_map,
							stapl_graph_edge_wt_id_map<typename G::vertex_property,
							typename G::edge_property>(const_cast<G*>(&g),
																																		w_vec_ptr),
						stapl_graph_id_map<typename G::vertex_property, typename G::edge_property>(),
							lsop, combine, std::numeric_limits<double>::max(), 0,
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
    
    
    std::less<double> lsop;
    // detail::_project2nd<double,double> combine;  // for Prim's MST.
    std::plus<double> combine;
    
    dijkstra_shortest_paths
      (bg, *vertices(bg).first, &parentmap[0], distmap, weightmap, indexmap,
       lsop, combine, std::numeric_limits<double>::max(), 0,
       default_dijkstra_visitor());
    
    t = PAPI_get_real_usec() - t;
    std::cout << "\t"<< t << ", ";
    
    stapl::graph<stapl::DIRECTED,stapl::MULTIEDGES,typename G::vertex_property, int> boost_out_graph1;
    convert_to_graph(parentmap, distmap, boost_out_graph1);
    
    cmap.reset();
    if((boost_out_graph1.get_num_vertices() == g.get_num_vertices())
       && (boost_out_graph1.get_num_vertices() == boost_out_graph1.get_num_edges()+1)
       && (is_cycle(boost_out_graph1, cmap) == false)) {
      cmap.reset();
      if (get_cc_count(boost_out_graph1, cmap) == 1) {
        tw5 = total_weight(boost_out_graph1);
        cout<<"<PASSED>\t"; // << endl;
      }
    } else {
      cout<<"{FAILED}\t";
      cmap.reset();
    }
    if(N < 15) display1(boost_out_graph1);
  }
		  
  cout << endl;
  cout << tw1 << ", "  << tw2 << ", "  << tw3 /*<< ", " << tw4 */ << ", "  << tw5 << "\n"; 
}

}  // end namespace dijkstra_test
