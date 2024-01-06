/* ****************************** NOT DONE ****************************** */
// gralg/cc_stats.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/cc_stats.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/YYY.hpp>

#include "gralghelp.hpp"
using namespace std;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct graph

  // apply algorithm

  stapl::connected_components(gr_vw);
  stapl::cc_stats(gr_vw, i, output_vw);
  is_same_cc(gr_vw, 0, 0);

  // display results

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
