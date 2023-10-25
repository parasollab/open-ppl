// gralg/cut_conductance.cc

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/numeric.hpp>

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>

#include <stapl/containers/graph/algorithms/cut_conductance.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>

#include "gralghelp.hpp"
using namespace std;


typedef stapl::graph<stapl::UNDIRECTED,
                     stapl::NONMULTIEDGES, int> graf_int_tp;
typedef stapl::graph<stapl::DIRECTED,
                     stapl::NONMULTIEDGES,
                     stapl::properties::no_property,
                     stapl::properties::no_property> graf_null_tp;

typedef stapl::graph_view<graf_int_tp> graf_vw_tp;

struct clear_if
{
  typedef std::size_t result_type;

  template<typename T, typename U>
  result_type operator()(T x, U y)
  {
    if (x > y)
      return 0;
    else
      return x;
  }
};

// create a sequence of blocks of size (n * ratio) starting from 1:
//  [ 1, 1, 1, 2, 2, 2, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
template<typename View>
void fill_membership(View& vw, double member_ratio, size_t num_sets)
{
  const size_t size = vw.size();
  const size_t block = size*member_ratio;

  stapl::iota(vw, block);

  stapl::transform(vw, vw, boost::bind(stapl::divides<size_t>(), _1, block));
  stapl::transform(vw, vw, boost::bind(clear_if(), _1, num_sets));
}

stapl::exit_code stapl_main(int argc, char **argv) {

  double memb_ratio = 0.5;
  size_t num_sets = 0;

  // construct graph
  size_t nx = 0;
  size_t ny = 0;
  size_t n = nx * ny;
  graf_vw_tp gr_vw = stapl::generators::make_mesh<graf_vw_tp>(nx, ny);

  // setup map for membership
  typedef stapl::array<size_t> memb_stor_tp;
  typedef stapl::array_view<memb_stor_tp> memb_vw_tp;
  typedef stapl::graph_external_property_map<graf_vw_tp, size_t, memb_vw_tp>
                 memb_map_tp;

  memb_stor_tp memb_ct(n);
  memb_vw_tp memb_vw(memb_ct);
  fill_membership(memb_vw, memb_ratio, num_sets);
  memb_map_tp memb_map(gr_vw, memb_vw);

  // apply algorithm
  double cut = stapl::cut_conductance<graf_vw_tp>(gr_vw, memb_map, 0);

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_cut_cond.txt");
  stapl::do_once( msg_val<int>( zout, "CUT_COND ", cut ) );

  return EXIT_SUCCESS;
}
