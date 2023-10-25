#include <iostream>
#include <fstream>
#include "ch6.hpp"

#include <stapl/containers/graph/algorithms/graph_io.hpp> // ## 1
#include <stapl/containers/graph/generators/torus.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/graph/generators/binary_tree.hpp>

typedef stapl::graph<
               stapl::UNDIRECTED,  // ## 2
               stapl::NONMULTIEDGES, // ## 3
               stapl::properties::no_property, // ## 4
               stapl::properties::no_property > graf_tp; // ## 5
typedef stapl::graph_view<graf_tp>              graf_vw_tp;

size_t ex_601a(size_t, size_t, string const &);
size_t ex_601b(size_t, size_t, string const &);
size_t ex_601c(size_t, size_t, string const &);


stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ofstream> zout;
  zout.open("ex_601.out");
  size_t x_dim, y_dim;

  x_dim = 7;
  y_dim = 11;
  stapl::do_once( msg( zout, "Example 601a" ) );
  int result = ex_601a(x_dim, y_dim, "ex_601a.out");
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  stapl::do_once( msg( zout, "Example 601b" ) );
  result = ex_601b(x_dim, y_dim, "ex_601b.out");
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  stapl::do_once( msg( zout, "Example 601c" ) );
  result = ex_601c(x_dim, y_dim, "ex_601c.out");
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  return EXIT_SUCCESS;
}


size_t ex_601a( size_t x_dim, size_t y_dim, string const &edgelog ) {

  graf_vw_tp gr_vw = stapl::generators::make_mesh<graf_vw_tp> // ## 6
                     (x_dim, y_dim);

  stapl::write_edge_list<graf_vw_tp>(gr_vw, edgelog); // ## 7

  return gr_vw.num_vertices(); // ## 8
}

size_t ex_601b( size_t x_dim, size_t y_dim, string const &edgelog ) {

  graf_vw_tp gr_vw = stapl::generators::make_binary_tree<graf_vw_tp> // ## 9
                     (x_dim*y_dim, false);

  stapl::write_edge_list<graf_vw_tp>(gr_vw, edgelog);

  return gr_vw.num_vertices();
}

size_t ex_601c( size_t x_dim, size_t y_dim, string const &edgelog ) {

  graf_vw_tp gr_vw = stapl::generators::make_torus<graf_vw_tp> // ## 10
                     (x_dim, y_dim);

  stapl::write_edge_list<graf_vw_tp>(gr_vw, edgelog);

  return gr_vw.num_vertices();
}
