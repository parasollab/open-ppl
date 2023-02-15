/* ****************************** NOT DONE ****************************** */
// gralg/breadth_first_search.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/breadth_first_search.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/torus.hpp>

#include "gralghelp.hpp"
using namespace std;

typedef stapl::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES,
                     int,int> stgraf_un_tp;
typedef stapl::graph_view<stgraf_un_tp> stgraf_un_vw_tp;

typedef stapl::graph_view<stgraf_un_tp> graf_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct graph
  size_t nx = 0;
  size_t ny = 0;
  graf_vw_tp gr_vw = stapl::generators::make_torus<graf_vw_tp>(nx, ny);
 
  // apply algorithm : level-sync BFS
#if 0
  size_t bfs = stapl::breadth_first_search(gr_vw, 0, 0 );
#endif
 
  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_br_first.txt");
#if 0
  stapl::do_once( msg_val<int>( zout, "BFS ", bfs ) );
#endif

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
