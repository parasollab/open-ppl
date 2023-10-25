/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/betweenness_centrality.hpp>
#include <stapl/skeletons/serial.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "../utility/test_util.h"
using namespace stapl;

typedef graph<DIRECTED, MULTIEDGES, properties::bc_property> Graph;
typedef graph_view<Graph> GView;

//////////////////////////////////////////////////////////////////////
/// @brief Prints edges of graph to standard output.
//////////////////////////////////////////////////////////////////////
struct print_edges
{
  typedef void result_type;

  template <typename Vertex>
  void operator()(Vertex v)
  {
    typedef typename Vertex::adj_edge_iterator edge_iterator;
    edge_iterator it_e = v.end();
    for (edge_iterator it = v.begin(); it != it_e; ++it) {
      std::cout << (*it).source() << " -> " << (*it).target() << '\n';
    }
  }
};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc != 5 && argc != 6) {
    if (get_location_id() == 0) {
      std::cout << "Usage: " << argv[0]
                << " x y KSources v_id print[0,1]\n";
      std::cout << "Usage: " << argv[0]
                << " filename KSources v_id print[0,1]\n\n";
      std::cout << "x,y:      "
                << "Parameters to generate a mesh with x*y vertices\n";
      std::cout << "filename: A dimacs file\n";
      std::cout << "KSources: Number of sources to use per iteration "
                << "(if KSources=0, all sources used)\n";
      std::cout << "v_id:     Prints BC value for vertex with id=v_d\n";
      std::cout << "print:    "
                << "0 - Prints timing, BC of vertex v_id\n";
      std::cout << "          "
                << "1 - Prints timing, edges, and BC of all vertices\n\n";
    }
    return EXIT_FAILURE;
  }

  size_t numSources = atoi(argv[argc-3]);
  size_t id = atoi(argv[argc-2]);
  const bool print = atoi(argv[argc-1]);

  GView gv;
  if (argc == 6) {
    gv = generators::make_mesh<GView>(atoi(argv[1]), atoi(argv[2]), false);
  } else {
    gv = read_dimacs<Graph>(std::string(argv[1]));
  }

  if (get_location_id() == 0) {
    std::cerr << "\nProcessors: " << get_num_locations();
    std::cerr << "\nK Sources: " << numSources;
    std::cerr << "\nn: " << gv.num_vertices();
    std::cerr << "\nm: " << gv.num_edges();
    std::cerr << "\nn*m: " << gv.num_vertices()*gv.num_edges() << '\n';
  }

  if (print) {
    stapl::serial_io(print_edges(), gv);
  }

  counter<default_timer> BC_time;

  BC_time.start();
  betweenness_centrality(gv, numSources);
  double BC_seconds = BC_time.stop();

  if (get_location_id() == 0) {
    std::cerr << "\nBC of " << id << ": "
              << gv[id].property().BC() << '\n';
    std::cerr << "BC calculation time: " << BC_seconds << " seconds\n\n";
    if (print) {
      std::cout << "\n";
      for (size_t i = 0; i < gv.num_vertices(); i++) {
        std::cout << i << ": " << gv[i].property().BC() << '\n';
      }
    }
  }


  return EXIT_SUCCESS;
}

