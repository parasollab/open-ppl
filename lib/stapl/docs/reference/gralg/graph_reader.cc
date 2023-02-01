/* ****************************** NOT DONE ****************************** */
// gralg/graph_reader.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>

#include "gralghelp.hpp"
using namespace std;

typedef stapl::graph<stapl::UNDIRECTED,
                     stapl::NONMULTIEDGES, int> graf_int_tp;
typedef stapl::graph<stapl::DIRECTED,
                     stapl::NONMULTIEDGES,
                     stapl::properties::no_property,
                     stapl::properties::no_property > graf_null_tp;

typedef stapl::graph_view<graf_int_tp> graf_int_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // apply algorithm to construct graph
  string filename = string("");
#if 0
  graf_int_vw_tp gr_vw= stapl::graph_reader<graf_int_vw_tp>
                               (filename, stapl::read_adj_list_line() );
#endif

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_gr_rd.txt");

#if 0
  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ));
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ));
#endif

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
