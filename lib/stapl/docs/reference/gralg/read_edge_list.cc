// gralg/read_edge_list.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>

#include "gralghelp.hpp"
using namespace std;

typedef stapl::dynamic_graph<stapl::UNDIRECTED,
                             stapl::NONMULTIEDGES, int> graf_int_tp;
typedef stapl::dynamic_graph<stapl::DIRECTED,
                             stapl::NONMULTIEDGES,
                             stapl::properties::no_property,
                             stapl::properties::no_property > graf_null_tp;

typedef stapl::graph_view<graf_int_tp> graf_int_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // apply algorithm to construct graph
  string filename = string("");
  graf_int_vw_tp gr_vw= stapl::read_edge_list<graf_int_tp>(filename);

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_rd_edge.txt");

  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ));
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ));

  return EXIT_SUCCESS;
}
