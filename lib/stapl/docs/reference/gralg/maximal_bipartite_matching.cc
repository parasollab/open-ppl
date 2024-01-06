/* ****************************** NOT DONE ****************************** */
// gralg/maximal_bipartite_matching.cc

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/containers/graph/algorithms/maximal_bipartite_matching.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/torus.hpp>

#include "gralghelp.hpp"
using namespace std;

typedef stapl::graph<stapl::UNDIRECTED, 
                     stapl::NONMULTIEDGES, int > graf_int_tp;

typedef stapl::graph_view<graf_int_tp> graf_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct graph
  size_t nx = 0;
  size_t ny = 0;
  graf_vw_tp gr_vw = stapl::generators::make_torus<graf_vw_tp>(nx, ny);

  // apply algorithm
  size_t k = 0;
#if 0
  size_t sz = stapl::maximal_bipartite_matching<graf_vw_tp>(gr_vw, k);
#endif

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_.txt");
#if 0
  stapl::do_once( msg_val<int>( zout, "SIZE ", sz) );
#endif

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
