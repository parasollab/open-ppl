/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <utility>
#include <vector>

#include <boost/program_options.hpp>

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/sssp.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/utility/do_once.hpp>

#include "../utility/test_util.h"


using namespace stapl;
using namespace std;


struct init_edges
{
  typedef void result_type;

  template<typename V>
  void operator()(V v) const
  {
    for (typename V::adj_edge_iterator ei = v.begin(); ei != v.end(); ++ei)
      (*ei).property() = 1.0;
      // @todo modify it to set your own value for each edges
  }
};


template <class G>
void check_correctness_sssp(G& g, double dist)
{
  if (get_location_id() == get_num_locations() - 1)
  {
    double last_distance = 0;
    bool err = false;
    typename G::vertex_iterator vi1;
    vi1 = g.find_vertex(g.num_vertices()-1);
    last_distance = (*vi1).property().distance();

    if (fabs(last_distance - dist ) >0.000001)
      err = true;

    if (err)
      cout << "[FAILED]\n";
    else
      cout << "[PASSED]\n";
  }

  rmi_fence();
}


struct print_distance
{
  size_t m_v;
  typedef void result_type;

  print_distance(size_t v)
    : m_v(v)
  { }

  template<typename V>
  void operator()(V v)
  {
    if (v.descriptor() == m_v)
      std::cout << "Distance to vertex " << m_v << " is "
                << v.property().distance() << std::endl;
  }

  void define_type(typer& t)
  {
    t.member(m_v);
  }
};


boost::program_options::variables_map parse_command_line(int argc, char* argv[])
{
  stringstream banner;
  banner << "STAPL Graph Library (SGL) \nCopyright (C) 2012 Texas A&M "
         << "University \nhttp://parasol.tamu.edu/stapl/ \n\nLonestar Benchmark"
         << " Suite \nSingle Source Shortest Path \nComputes the shortest path"
         << " from a source node to all nodes in a directed graph using a "
         << "modified Bellman-Ford algorithm. \n";
  banner << "Usage: " << argv[0] << " --help for command line options\n\n";

  do_once([&](void) { std::cout << banner.str(); });

  boost::program_options::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce this message")
      ("source", boost::program_options::value<size_t>(),
       "source vertex to start from")
      ("report-vertex", boost::program_options::value<size_t>(),
       "vertex to report shortest path to")
      ("no-verify", "do not verify correctness")
      ("mesh-x", boost::program_options::value<size_t>(),
       "number of columns in the mesh")
      ("mesh-y", boost::program_options::value<size_t>(),
       "number of rows in the mesh")
  ;

  boost::program_options::variables_map vm;

  try {
    boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);
  } catch(boost::program_options::error e) {
    if (get_location_id() == 0)
      std::cout << e.what() << std::endl;
    exit(1);
  }

  if (vm.count("help")) {
    if (get_location_id() == 0)
      cout << desc << std::endl;
    exit(1);
  }

  return vm;
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  boost::program_options::variables_map vm = parse_command_line(argc, argv);

  bool verify = vm.count("no-verify") == 0;

  size_t x = vm.count("mesh-x") == 1 ? vm["mesh-x"].as<size_t>() : 10;
  size_t y = vm.count("mesh-y") == 1 ? vm["mesh-y"].as<size_t>() : 4;
  x *= get_num_locations();
  srand(x);

  size_t source = vm.count("source") == 1 ? vm["source"].as<size_t>() : 0;
  size_t report_vertex = vm.count("report-vertex") == 1 ?
                         vm["report-vertex"].as<size_t>() : x*y-1;


  typedef dynamic_graph<DIRECTED, MULTIEDGES,
                        properties::sssp_property, double> graph_type;
  typedef graph_view<graph_type> view_type;
  graph_type g(x*y);
  view_type view = generators::make_mesh<view_type>(x, y);

  map_func(init_edges(), view);

  if (get_location_id() == 0)
    std::cout << "Running SSSP" << std::endl;

  counter<default_timer> t;
  t.start();

  sssp(view, source);

  double sssp_time = t.stop();

  if (verify)
    check_correctness_sssp(view, x + y - 2.0);
  if (get_location_id() == 0)
    std::cout << "time= " << sssp_time << " seconds." << std::endl;

  map_func(print_distance(report_vertex), view);

  return EXIT_SUCCESS;
}
