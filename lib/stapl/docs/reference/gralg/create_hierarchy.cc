/* ****************************** NOT DONE ****************************** */
// gralg/create_hierarchy.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/create_hierarchy.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/torus.hpp>

#include "gralghelp.hpp"
using namespace std;

typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                             int,int> stgraf_dir_tp;
typedef stapl::graph_view<stgraf_dir_tp> graf_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  size_t nx = 0;
  size_t ny = 0;
  // construct graph
  graf_vw_tp vw = stapl::generators::make_torus<graf_vw_tp>(nx, ny);

  // apply algorithm
#if 0
  std::vector<graf_vw_tp> hierarchy = stapl::create_hierarchy(
#endif

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_cr_hier.txt");
  //stapl::do_once( msg_val<int>( zout, " ",  ) );

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
