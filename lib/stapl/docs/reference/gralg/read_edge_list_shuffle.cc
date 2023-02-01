/* ****************************** NOT DONE ****************************** */
// gralg/read_edge_list_shuffle.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>

#include "gralghelp.hpp"
using namespace std;

typedef stapl::dynamic_graph<stapl::UNDIRECTED,
                             stapl::MULTIEDGES, int> graf_int_tp;
typedef stapl::dynamic_graph<stapl::DIRECTED,
                             stapl::MULTIEDGES,
                             stapl::properties::no_property,
                             stapl::properties::no_property> graf_null_tp;

typedef stapl::graph_view<graf_int_tp> graf_int_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // define shuffle_map_vw

  // apply algorithm to construct graph
  string filename = string("");
#if 0
  graf_int_vw_tp gr_vw= stapl::read_edge_list_shuffle<graf_int_tp>
                        (shuffle_map_vw, filename);
#endif

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_rd_edge_shuf.txt");

#if 0
  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ));
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ));
#endif

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
