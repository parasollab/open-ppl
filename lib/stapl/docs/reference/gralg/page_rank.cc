/* ****************************** NOT DONE ****************************** */
// gralg/page_rank.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/page_rank.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/torus.hpp>

#include "gralghelp.hpp"
using namespace std;

typedef stapl::graph<stapl::UNDIRECTED,
                     stapl::MULTIEDGES, int> graf_int_tp;
typedef stapl::graph<stapl::DIRECTED,
                     stapl::MULTIEDGES,
                     stapl::properties::no_property,
                     stapl::properties::no_property> graf_null_tp;

typedef stapl::graph_view<graf_int_tp> graf_int_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct graph
  size_t nx = 0;
  size_t ny = 0;
  graf_int_vw_tp gr_vw = stapl::generators::make_torus<graf_int_vw_tp>(nx, ny);

  // apply algorithm
#if 0
  stapl::page_rank<graf_int_vw_tp>(gr_vw, 0, 0.85);
#endif

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_pg_rank.txt");
  //stapl::do_once( msg_val<int>( zout, " ",  ) );

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
