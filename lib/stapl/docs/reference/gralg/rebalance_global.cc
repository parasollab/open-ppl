/* ****************************** NOT DONE ****************************** */
// gralg/rebalance_global.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/rebalance_global.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/YYY.hpp>

#include "gralghelp.hpp"
using namespace std;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct graph

  // apply algorithm
  stapl::rebalance_global(gr_vw_1);

  stapl::rebalance_global(gr_vw_2, weights);

  // display results

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
