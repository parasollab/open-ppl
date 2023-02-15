/* ****************************** NOT DONE ****************************** */
// gralg/create_level.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/create_level.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/torus.hpp>

#include "gralghelp.hpp"
using namespace std;

typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, int >
                             stgraf_dir_tp;
typedef stapl::graph_view<stgraf_dir_tp> graf_int_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct graph
  size_t nx = 0;
  size_t ny = 0;
  graf_int_vw_tp vw = stapl::generators::make_torus<graf_int_vw_tp>(nx, ny);

  // apply algorithm
#if 0
  graf_vw_tp x = stapl::create_level(vw
#endif

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_cr_lev.txt");
  //stapl::do_once( msg_val<int>( zout, " ",  ) );

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
