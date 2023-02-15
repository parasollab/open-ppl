/* ****************************** NOT DONE ****************************** */
// gralg/connected_components.cc
 
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
 
#include <stapl/containers/graph/algorithms/connected_components.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/YYY.hpp>
 
#include "gralghelp.hpp"
using namespace std;

typedef stapl::graph<stapl::UNDIRECTED, 
                     stapl::MULTIEDGES, int_tp> graf_int_tp;
typedef stapl::graph<stapl::DIRECTED, 
                     stapl::MULTIEDGES, 
                     stapl::properties::no_property,
                     stapl::properties::no_property> graf_null_tp;

typedef stapl::graph_view<graf_int_tp> graf_int_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct graph

  
  // apply algorithm
  size_t iters = stapl::connected_components(gr_vw);

  // display results
  
  stapl::cc_stats

  stapl:: is_same_cc

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
