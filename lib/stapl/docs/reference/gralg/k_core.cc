// gralg/k_core.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/k_core.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>

#include "gralghelp.hpp"
using namespace std;

typedef stapl::dynamic_graph<stapl::UNDIRECTED,
                             stapl::NONMULTIEDGES, int> graf_int_tp;
typedef stapl::dynamic_graph<stapl::DIRECTED,
                             stapl::NONMULTIEDGES,
                             stapl::properties::no_property,
                             stapl::properties::no_property> graf_null_tp;

typedef stapl::graph_view<graf_int_tp> graf_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct graph
  size_t nx = 0;
  size_t ny = 0;
  graf_vw_tp gr_vw = stapl::generators::make_mesh<graf_vw_tp>(nx, ny);

  // apply algorithm
  size_t k = 0;
  size_t core_sz = 0;
  size_t iters = stapl::k_core<graf_vw_tp>(gr_vw, core_sz, k);

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_kcore.txt");
  stapl::do_once( msg_val<int>( zout, "K_CORE ", iters ) );

  return EXIT_SUCCESS;
}
