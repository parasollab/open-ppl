// gralg/rebalance_diffusive.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/rebalance_diffusive.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>

#include "gralghelp.hpp"
using namespace std;

typedef stapl::dynamic_graph<stapl::UNDIRECTED,
                             stapl::MULTIEDGES, int> graf_int_tp;
typedef stapl::dynamic_graph<stapl::DIRECTED,
                             stapl::MULTIEDGES,
                             stapl::properties::no_property,
                             stapl::properties::no_property> graf_null_tp;

typedef stapl::graph_view<graf_int_tp> graf_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct graph
  size_t nx = 0;
  size_t ny = 0;
  graf_vw_tp gr_vw = stapl::generators::make_mesh<graf_vw_tp>(nx, ny);

  // apply algorithm
  stapl::rebalance_diffusive<graf_vw_tp>(gr_vw);

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_reb_diff.txt");
  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ));
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ));

  return EXIT_SUCCESS;
}
