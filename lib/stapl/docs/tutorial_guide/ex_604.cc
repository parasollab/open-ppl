#include <iostream>
#include <fstream>
#include "ch6.hpp"

typedef stapl::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES,
               stapl::properties::bfs_property, 
               stapl::properties::no_property>              bfs_graf_tp; // ## 1
typedef stapl::graph_view<bfs_graf_tp>                      bfs_graf_vw_tp;

typedef size_t vtx_col_tp; // ## 2

typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES,
               stapl::properties::scc_property<vtx_col_tp>,
               stapl::properties::no_property>              scc_graf_tp; // ## 3
typedef stapl::graph_view<scc_graf_tp>                      scc_graf_vw_tp;

typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
               stapl::properties::topological_sort_property, 
               stapl::properties::no_property>              tops_graf_tp; // ## 4
typedef stapl::graph_view<tops_graf_tp>                     tops_graf_vw_tp;

typedef stapl::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
               int, int >                                   graf_int_tp; // ## 5
typedef stapl::graph_view<graf_int_tp>                      graf_int_vw_tp;


size_t ex_604a( string const &, stapl::stream<ofstream> &);
size_t ex_604b( string const &, stapl::stream<ofstream> &);
size_t ex_604c( string const &, stapl::stream<ofstream> &);
size_t ex_604d( string const &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ofstream> zout;
  zout.open("ex_604.out");
  size_t result;

  stapl::do_once( msg( zout, "Example 604a" ) );
  result = ex_604a("u_v48_e108.txt", zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  stapl::do_once( msg( zout, "Example 604b" ) );
  result = ex_604b("d_v50_e147.txt", zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  stapl::do_once( msg( zout, "Example 604c" ) );
  result = ex_604c("dag_v88_e547.txt", zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  stapl::do_once( msg( zout, "Example 604d" ) );
  result = ex_604d("u_v88_e634.txt", zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  zout.close();
  return EXIT_SUCCESS;
}

struct get_level_wf
{
  typedef void result_type;
  template<typename Vtx, typename Elem>
  result_type operator() (Vtx vtx, Elem elem)
  {
    elem = vtx.property().level();
  }
};

size_t ex_604a( string const &filename, stapl::stream<ofstream> &zout ) {

  bfs_graf_vw_tp gr_vw = stapl::read_edge_list<bfs_graf_tp>(filename);

  typedef typename bfs_graf_vw_tp::vertex_descriptor vtx_desc_tp;
  typedef stapl::array<vtx_desc_tp> vtx_desc_ary_tp; // ## 6
  typedef stapl::array_view<vtx_desc_ary_tp> vtx_desc_ary_vw_tp;

  size_t count= stapl::breadth_first_search(gr_vw, 0); // ## 7

  size_t size = gr_vw.size();
  vtx_desc_ary_tp levels_ct(size);
  vtx_desc_ary_vw_tp levels_vw(levels_ct);
  stapl::map_func( get_level_wf(), gr_vw, levels_vw ); // ## 8

  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ) );
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ) );
  stapl::serial_io( put_ndx_val_wf(zout), stapl::counting_view<int>(size),
                    levels_vw );

  stapl::write_edge_list<bfs_graf_vw_tp>(gr_vw, string("ex_604a.out") );

  return count;
}

struct get_color_wf
{
  typedef void result_type;
  template<typename Vtx, typename Elem>
  result_type operator() (Vtx vtx, Elem elem)
  {
    elem = vtx.property().get_cc();
  }
};

size_t ex_604b( string const &filename, stapl::stream<ofstream> &zout ) {

  scc_graf_vw_tp gr_vw = stapl::read_edge_list<scc_graf_tp>(filename);

  typedef stapl::array<vtx_col_tp> vtx_col_ary_tp; // ## 9
  typedef stapl::array_view<vtx_col_ary_tp> vtx_col_ary_vw_tp;

  stapl::pscc<scc_graf_vw_tp>(gr_vw); // ## 10

  size_t size = gr_vw.size();
  vtx_col_ary_tp vtx_colors_ct(size);
  vtx_col_ary_vw_tp vtx_colors_vw(vtx_colors_ct);
  stapl::map_func( get_color_wf(), gr_vw, vtx_colors_vw ); // ## 11

  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ) );
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ) );
  stapl::serial_io( put_ndx_val_wf(zout), stapl::counting_view<int>(size),
                      vtx_colors_vw );

  stapl::write_edge_list<scc_graf_vw_tp>(gr_vw, string("ex_604b.out") );

  return 0;
}

struct get_topol_rank_wf
{
  typedef int result_type;
  template<typename Vtx>
  result_type operator()(Vtx vtx)
  {
    return vtx.property().rank(); // ## 12
  }
};

size_t ex_604c( string const &filename, stapl::stream<ofstream> &zout ) {

  tops_graf_vw_tp gr_vw = stapl::read_edge_list<tops_graf_tp>(filename);

  stapl::topological_sort(gr_vw, 0); // ## 13
  int max_path_len = stapl::map_reduce( get_topol_rank_wf(),
                                        max_int_wf(), gr_vw ); // ## 14

  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ) );
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ) );
  stapl::do_once( msg_val<int>( zout, "MAX PATH LENGTH ", max_path_len ) );

  stapl::write_edge_list<tops_graf_vw_tp>(gr_vw, string("ex_604c.out") );

  return max_path_len;
}

struct sum_edge_props_wf
{
  typedef int result_type;
  template<typename Vtx>
  result_type operator()(Vtx vtx)
  {
    int sum = 0;
    typename Vtx::adj_edge_iterator aei; // ## 15
    for( aei = vtx.begin(); aei != vtx.end(); ++aei ) { // ## 16
      sum += (*aei).property(); // ## 17
    }
    return sum;
  }
};


struct read_edge_list_with_props_line
{
  /// Lines have properties for edges
  using has_edge_property = std::true_type;

  template<typename Aggr, typename Graph>
  size_t operator() (std::stringstream& ss, Aggr& aggr, Graph* g) const
  {
    size_t src, tgt;
    typename Graph::edge_property prop;

    ss >> src;
    ss >> tgt;
    ss >> prop;

    if (src >= g->size() || tgt >= g->size())
      return 1;
    else {
      g->add_edge_async(src, tgt, prop);
      return 0;
    }
  }
};

size_t ex_604d( string const &filename, stapl::stream<ofstream> &zout ) {

  graf_int_vw_tp gr_vw = stapl::graph_reader<graf_int_tp>(
      filename, read_edge_list_with_props_line() ); // ## 18

  int max_sum = stapl::map_reduce( sum_edge_props_wf(), max_int_wf(),
                                   gr_vw ); // ## 19

  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ) );
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ) );
  stapl::do_once( msg_val<int>( zout, "MAX_SUM ", max_sum ) );

  stapl::write_adj_list<graf_int_vw_tp>(gr_vw, string("ex_604d.out") );

  return max_sum;
}
