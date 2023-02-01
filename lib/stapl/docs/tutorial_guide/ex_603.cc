#include <iostream>
#include <fstream>
#include "ch6.hpp"

typedef stapl::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES,
               int, int>                        graf_int_tp; // ## 1
typedef stapl::graph_view<graf_int_tp>          graf_int_vw_tp;

typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
               int, int>                        d_graf_int_tp; // ## 2
typedef stapl::graph_view<d_graf_int_tp>        d_graf_int_vw_tp;

typedef stapl::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
               int, int>                        m_graf_int_tp; // ## 3
typedef stapl::graph_view<m_graf_int_tp>        m_graf_int_vw_tp;

typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES,
               int, int>                        dm_graf_int_tp; // ## 4
typedef stapl::graph_view<dm_graf_int_tp>       dm_graf_int_vw_tp;

size_t ex_603a(const char *, stapl::stream<ofstream> & );
size_t ex_603b(const char *, stapl::stream<ofstream> & );
size_t ex_603c(const char *, stapl::stream<ofstream> & );
size_t ex_603d(const char *, stapl::stream<ofstream> & );


stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ofstream> zout;
  zout.open("ex_603.out");
  int result = 0;

  stapl::do_once( msg( zout, "Example 603a" ) );
  result = ex_603a("u_v48_e108.txt", zout); // ## 5
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  stapl::do_once( msg( zout, "Example 603b" ) );
  result = ex_603b("d_v48_e210.txt", zout); // ## 6
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  stapl::do_once( msg( zout, "Example 603c" ) );
  result = ex_603c("u_v48_e108.txt", zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  stapl::do_once( msg( zout, "Example 603d" ) );
  result = ex_603d("d_v48_e216.txt", zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  zout.close();
  return EXIT_SUCCESS;
}

struct ex_603_dup_edges_wf {
private:
  stapl::stream<ofstream> m_zout;

public:
  typedef void result_type;

  ex_603_dup_edges_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  {}

  template <typename Elem1, typename Elem2, typename Elem3, typename Graph>
  result_type operator()(Elem1 src, Elem2 dest, Elem3 ndx, Graph graf) {
    if( 0 == ndx % 10 ) {
      typename Graph::edge_descriptor ed = graf.add_edge(dest,src); // ## 7
      if (numeric_limits<size_t>::max() == ed.id()) {
        stapl::do_once( msg( m_zout, "failed to add edge" ) );
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t ex_603a( const char *filename, stapl::stream<ofstream> &zout ) {

  graf_int_vw_tp gr_vw = stapl::read_edge_list<graf_int_tp>(filename); // ## 8

  size_t vtx_cnt=0, edge_cnt=0;
  ifstream xin;
  xin.open(filename);
  if( !xin.is_open() ) {
    stapl::do_once( msg( zout, "603a input open failed" ) );
    return EXIT_FAILURE;
  }
  xin >> vtx_cnt >> edge_cnt; // ## 9
  xin.close();

  stapl::do_once( msg_val<int>( zout, "vtx_cnt", vtx_cnt ) );
  stapl::do_once( msg_val<int>( zout, "edge_cnt", edge_cnt ) );

  vec_int_tp src_ct(edge_cnt), dest_ct(edge_cnt);
  vec_int_vw_tp src_vw(src_ct), dest_vw(dest_ct); // ## 10

  stapl::stream<ifstream> zin;
  zin.open(filename);
  zin >> vtx_cnt >> edge_cnt;
  stapl::serial_io(get_pair_wf(zin), src_vw, dest_vw ); // ## 11
  zin.close();

  stapl::map_func(ex_603_dup_edges_wf(zout), src_vw, dest_vw,  // ## 12
                  stapl::counting_view<int>(src_vw.size()),
                  stapl::make_repeat_view(gr_vw) );

  stapl::write_edge_list<graf_int_vw_tp>(gr_vw, "ex_603a.out");

  return vtx_cnt;
}


struct ex_603_rev_edges_wf {
private:
  stapl::stream<ofstream> m_zout;

public:
  typedef void result_type;

  ex_603_rev_edges_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  {}

  template <typename Elem1, typename Elem2, typename Graph>
  result_type operator()(Elem1 src, Elem2 dest, Graph graf) {
    if( src < 13 && dest < 13 ) {
      typename Graph::edge_descriptor ed = graf.add_edge(dest,src); // ## 13
      if (numeric_limits<size_t>::max() == ed.id()) {
        stapl::do_once( msg( m_zout, "failed to add edge" ) );
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t ex_603b( const char *filename, stapl::stream<ofstream> &zout ) {

  d_graf_int_vw_tp gr_vw = stapl::read_edge_list<d_graf_int_tp>(filename);

  size_t vtx_cnt=0, edge_cnt=0;
  ifstream xin;
  xin.open(filename);
  if( !xin.is_open() ) {
    stapl::do_once( msg( zout, "603b input open failed" ) );
    return EXIT_FAILURE;
  }
  xin >> vtx_cnt >> edge_cnt;
  xin.close();

  vec_int_tp src_ct(edge_cnt), dest_ct(edge_cnt);
  vec_int_vw_tp src_vw(src_ct), dest_vw(dest_ct);

  stapl::stream<ifstream> zin;
  zin.open(filename);
  zin >> vtx_cnt >> edge_cnt;
  stapl::serial_io(get_pair_wf(zin), src_vw, dest_vw );
  zin.close();

  stapl::do_once( msg_val<int>( zout, "vtx_cnt", vtx_cnt ) );
  stapl::do_once( msg_val<int>( zout, "edge_cnt", edge_cnt ) );

  stapl::map_func(ex_603_rev_edges_wf(zout), src_vw, dest_vw,  // ## 14
                  stapl::make_repeat_view(gr_vw) );

  stapl::write_edge_list<d_graf_int_vw_tp>(gr_vw, "ex_603b.out");

  return vtx_cnt;
}


struct ex_603_groups_wf {
private:
  stapl::stream<ofstream> m_zout;

public:
  typedef void result_type;

  ex_603_groups_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  {}

  template <typename Elem1, typename Elem2, typename Map, typename Graph>
  result_type operator()(Elem1 src, Elem2 dest, Map group, Graph graf) {
    if( group[src] == group[dest] ) {
      typename Graph::edge_descriptor ed = graf.add_edge(src,dest);
      if (numeric_limits<size_t>::max() == ed.id()) {
        stapl::do_once( msg( m_zout, "failed to add edge" ) );
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t ex_603c( const char *filename, stapl::stream<ofstream> &zout ) {

  m_graf_int_vw_tp gr_vw = stapl::read_edge_list<m_graf_int_tp>(filename);

  size_t vtx_cnt=0, edge_cnt=0;
  ifstream xin;
  xin.open(filename);
  if( !xin.is_open() ) {
    stapl::do_once( msg( zout, "603c input open failed" ) );
    return EXIT_FAILURE;
  }
  xin >> vtx_cnt >> edge_cnt;
  xin.close();

  vec_int_tp src_ct(edge_cnt), dest_ct(edge_cnt);
  vec_int_vw_tp src_vw(src_ct), dest_vw(dest_ct);

  stapl::stream<ifstream> zin;
  zin.open(filename);
  zin >> vtx_cnt >> edge_cnt;
  stapl::serial_io(get_pair_wf(zin), src_vw, dest_vw );
  zin.close();

  stapl::do_once( msg_val<int>( zout, "vtx_cnt", vtx_cnt ) );
  stapl::do_once( msg_val<int>( zout, "edge_cnt", edge_cnt ) );

  std::map<int,int> groups; // ## 15
  xin.open("group_map.txt");
  if( !xin.is_open() ) {
    stapl::do_once( msg( zout, "603c input failed" ) );
    return EXIT_FAILURE;
  }

  string buffer;
  int first, second;
  while ( getline(xin, buffer) )  { // ## 16
    istringstream(buffer) >> first >> second;
    groups.insert(make_pair(first,second)); // ## 17
  }
  xin.close();

  stapl::map_func(ex_603_groups_wf(zout), src_vw, dest_vw, // ## 18
                  stapl::make_repeat_view(groups),
                  stapl::make_repeat_view(gr_vw) );

  stapl::write_edge_list<m_graf_int_vw_tp>(gr_vw, "ex_603c.out");

  return vtx_cnt;
}

size_t ex_603d( const char *filename, stapl::stream<ofstream> &zout ) {

  dm_graf_int_vw_tp gr_vw = stapl::read_edge_list<dm_graf_int_tp>(filename);

  size_t vtx_cnt=0, edge_cnt=0;
  ifstream xin;
  xin.open(filename);
  if( !xin.is_open() ) {
    stapl::do_once( msg( zout, "603d input open failed" ) );
    return EXIT_FAILURE;
  }
  xin >> vtx_cnt >> edge_cnt;

  vec_int_tp src_ct(edge_cnt), dest_ct(edge_cnt);
  vec_int_vw_tp src_vw(src_ct), dest_vw(dest_ct);

  stapl::stream<ifstream> zin;
  zin.open(filename);
  zin >> vtx_cnt >> edge_cnt;
  stapl::serial_io(get_pair_wf(zin), src_vw, dest_vw );
  zin.close();

  stapl::do_once( msg_val<int>( zout, "vtx_cnt", vtx_cnt ) );
  stapl::do_once( msg_val<int>( zout, "edge_cnt", edge_cnt ) );

  stapl::map_func(ex_603_dup_edges_wf(zout), src_vw, dest_vw,  // ## 19
                  stapl::counting_view<int>(src_vw.size()),
                  stapl::make_repeat_view(gr_vw) );

  stapl::map_func(ex_603_rev_edges_wf(zout), src_vw, dest_vw,  // ## 20
                  stapl::make_repeat_view(gr_vw) );

  stapl::write_edge_list<dm_graf_int_vw_tp>(gr_vw, "ex_603d.out");

  return vtx_cnt;
}

