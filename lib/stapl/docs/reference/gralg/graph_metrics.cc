/* ****************************** NOT DONE ****************************** */
// gralg/graph_metrics.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/graph_metrics.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/torus.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>

#include "gralghelp.hpp"
using namespace std;

typedef stapl::graph<stapl::UNDIRECTED,
                     stapl::MULTIEDGES, int> graf_int_tp;
typedef stapl::graph<stapl::DIRECTED,
                     stapl::MULTIEDGES,
                     stapl::properties::no_property,
                     stapl::properties::no_property> graf_null_tp;

typedef stapl::graph_view<graf_int_tp> graf_int_vw_tp;

typedef stapl::graph_view<graf_null_tp> graf_null_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct graph
  size_t nx = 0;
  size_t ny = 0;
  graf_int_vw_tp g1_vw = stapl::generators::make_mesh<graf_int_vw_tp>(nx, ny);

  graf_null_vw_tp g2_vw = stapl::generators::make_torus<graf_null_vw_tp>(nx, ny);

#if 0
  // apply algorithm
  stapl::metrics_info mesh_met = stapl::graph_metrics<graf_un_vw_tp>
                                 (g1_vw, pmap);

  stapl::metrics_info torus_met = stapl::graph_metrics<graf_dir_vw_tp>
                                  (g2_vw, pmap);

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_gr_met.txt");

  stapl::do_once( msg_val<int>( zout, "Mesh internal ",
                  mesh_met.sum_internal_edges ) );
  stapl::do_once( msg_val<int>( zout, "Mesh cross ",
                  mesh_met.cross_internal_edges ) );

  stapl::do_once( msg_val<int>( zout, "Torus internal ",
                  torus_met.sum_internal_edges ) );
  stapl::do_once( msg_val<int>( zout, "Torus cross ",
                  torus_met.cross_internal_edges ) );
#endif

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
