// gralg/sharded_graph_reader.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>

#include "gralghelp.hpp"
using namespace std;

typedef stapl::dynamic_graph<stapl::UNDIRECTED,
                             stapl::NONMULTIEDGES, int > graf_int_tp;
typedef stapl::dynamic_graph<stapl::DIRECTED,
                             stapl::NONMULTIEDGES,
                             stapl::properties::no_property,
                             stapl::properties::no_property > graf_null_tp;

typedef stapl::graph_view<graf_int_tp> graf_int_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // shard file for input
  string filename = string("");
  stapl::graph_sharder(filename,100);

  // apply algorithm to construct graph
  graf_int_vw_tp graf_int_vw_tp= stapl::sharded_graph_reader<graf_int_tp>
                        (filename, stapl::read_adj_list_line() );

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_gr_shard.txt");

  stapl::do_once( msg_val<int>( zout, "VERT ", graf_int_vw_tp.num_vertices() ));
  stapl::do_once( msg_val<int>( zout, "EDGE ", graf_int_vw_tp.num_edges() ));

  return EXIT_SUCCESS;
}
