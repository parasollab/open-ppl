/* ****************************** NOT DONE ****************************** */
// gralg/color_graph.cc
 
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
 
#include <stapl/containers/graph/algorithms/color_graph.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/YYY.hpp>
 
#include "gralghelp.hpp"
using namespace std;

class access_color
{
public:
  typedef size_t value_tp;
  template<typename VProperty>
  value_tp get(VProperty const& vp)
  {
    return vp.get_color();
  }
  template<typename VProperty>
  void put(VProperty vp, value_tp const& color)
  {
    return vp.set_color(color);
  }
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct graph
  size_t nx = 0;
  size_t ny = 0;
  graf_vw_tp gr_vw = stapl::generators::make_torus<graf_vw_tp>(nx, ny);

  // apply algorithm
  typedef graph_internal_property_map<gr_int_vw_tp,access_color> prop_map_tp;
  prop_map_tp prop_map(gr_vw);
  stapl::color_graph(vw, prop_map);
  bool is_valid = stapl::is_valid_graph_coloring(gr_vw, prop_map);

// domset1D<size_t> colors = stapl::get_graph_colors(gr_vw, prop_map);
  
  // display results

  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() );

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
