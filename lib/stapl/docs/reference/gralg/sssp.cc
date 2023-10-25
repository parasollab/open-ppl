/* ****************************** NOT DONE ****************************** */
// gralg/sssp.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include <stapl/containers/graph/algorithms/sssp.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/torus.hpp>

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
  graf_vw_tp gr_vw = stapl::generators::make_torus<graf_vw_tp>(nx, ny);

  // apply algorithm
#if 0
  size_t sssp = stapl::sssp<graf_vw_tp>(gr_vw, 0, 0);
#endif

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_.txt");
  //stapl::do_once( msg_val<int>( zout, " ",  ) );

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
