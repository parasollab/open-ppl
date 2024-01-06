// gralg/write_dot.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/binary_tree.hpp>

#include "gralghelp.hpp"
using namespace std;

typedef stapl::graph<stapl::UNDIRECTED,
                     stapl::MULTIEDGES, int> graf_int_tp;
typedef stapl::graph<stapl::DIRECTED,
                     stapl::MULTIEDGES,
                     stapl::properties::no_property,
                     stapl::properties::no_property > graf_null_tp;

typedef stapl::graph_view<graf_int_tp> graf_int_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct graph
  size_t n =0;
  graf_int_vw_tp gr_vw = stapl::generators::make_binary_tree
                       <graf_int_vw_tp>(n, false);

  // apply algorithm
  string filename = string("");
  stapl::write_dot<graf_int_vw_tp>(gr_vw, filename );

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_wr_dot.txt");

  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ));
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ));

  return EXIT_SUCCESS;
}
