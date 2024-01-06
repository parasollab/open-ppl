#include <iostream>
#include <fstream>
#include "ch6.hpp"
#include <assert.h>

typedef stapl::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES, // ## 1
               int, int>                        graf_int_tp;
typedef stapl::graph_view<graf_int_tp>          graf_int_vw_tp;

typedef stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, // ## 2
               int, int>                        dygraf_int_tp;
typedef stapl::graph_view<dygraf_int_tp>        dygraf_int_vw_tp;

typedef typename dygraf_int_vw_tp::vertex_descriptor vd_dygraf_int_tp;

typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
               int, int>                        dir_graf_int_tp;
typedef stapl::graph_view<dir_graf_int_tp>      dir_graf_int_vw_tp;

size_t ex_602a( string const &, stapl::stream<ofstream> &);
size_t ex_602b( string const &, stapl::stream<ofstream> &);
size_t ex_602c( string const &, stapl::stream<ofstream> &);


stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ofstream> zout;
  zout.open("ex_602.out");
  size_t result;

  stapl::do_once( msg( zout, "Example 602a" ) );
  result = ex_602a("u_v48_e108.txt", zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  stapl::do_once( msg( zout, "Example 602b" ) );
  result = ex_602b("d_v48_e216.txt", zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  stapl::do_once( msg( zout, "Example 602c" ) );
  result = ex_602c("d_v88_e1117.txt", zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  zout.close();
  return EXIT_SUCCESS;
}

struct ex_602_init_wf {
  typedef void result_type;
  template<typename Vertex>
  result_type operator()(Vertex vtx) {
    vtx.property() = 0; // ## 3
  }
};

size_t ex_602a( string const &filename, stapl::stream<ofstream> &zout ) {

  graf_int_vw_tp gr_vw = stapl::read_edge_list<graf_int_tp>(filename); // ## 4

  gr_vw[0].property() = 314159; // ## 5
  gr_vw[1].property() = 271828;

  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ) );
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ) );

  stapl::write_adj_list<graf_int_vw_tp>(gr_vw, string("ex_602a.out") ); // ## 6

  return gr_vw.num_vertices();
}

size_t ex_602b( string const &filename, stapl::stream<ofstream> &zout ) {

  dygraf_int_vw_tp gr_vw = stapl::graph_reader<dygraf_int_tp>
                           (filename, stapl::read_edge_list_line() ); // ## 7

  stapl::map_func( ex_602_init_wf(), gr_vw ); // ## 8
  gr_vw[0].property() = 314159;
  gr_vw[1].property() = 271828;

  dygraf_int_tp::vertex_descriptor new_vtx_id = gr_vw.add_vertex(161803); // ## 9
  stapl::rmi_fence();

  dygraf_int_tp::edge_descriptor ed = gr_vw.add_edge(0, new_vtx_id,
                                                     stapl::get_location_id()); // ## 10
  assert(ed.source() == 0 && ed.target() == new_vtx_id);

  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ) );
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ) );

  stapl::write_adj_list<dygraf_int_vw_tp>(gr_vw, string("ex_602b.out") );

  return gr_vw.num_vertices();
}

struct read_edge_list_with_props_line // ## 11
{
  /// Lines have properties for edges
  using has_edge_property = std::true_type;

  template<typename Aggr, typename Graph>
  size_t operator() (std::stringstream& ss, Aggr& aggr, Graph* g) const // ## 12
  {
    size_t src, tgt;
    typename Graph::edge_property prop;

    ss >> src; // ## 13
    ss >> tgt;
    ss >> prop;

    if (src >= g->size() || tgt >= g->size())
      return 1;
    else {
      g->add_edge_async(src, tgt, prop); // ## 14
      return 0;
    }
  }
};

size_t ex_602c( string const &filename, stapl::stream<ofstream> &zout ) {

  stapl::graph_sharder(filename,100); // ## 15

  dir_graf_int_vw_tp gr_vw = stapl::sharded_graph_reader<dir_graf_int_tp>
                             (filename, read_edge_list_with_props_line() ); // ## 16

  gr_vw[0].property() = 314159;
  gr_vw[1].property() = 271828;
  stapl::map_func( ex_602_init_wf(), gr_vw );

  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ) );
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ) );

  stapl::write_adj_list<dir_graf_int_vw_tp>(gr_vw, string("ex_602c.out") ); // ## 17

  return gr_vw.num_vertices();
}

