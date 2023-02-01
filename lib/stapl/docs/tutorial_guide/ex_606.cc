#include <iostream>
#include <fstream>
#include "ch6.hpp"

typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, int, int>
                                                       graf_int_tp;  // ## 1
typedef stapl::graph_view<graf_int_tp>                 graf_int_vw_tp;

typedef stapl::array<graf_int_tp>                      ary_graf_int_tp; // ## 2
typedef stapl::array_view<ary_graf_int_tp>             ary_graf_int_vw_tp;

typedef stapl::vector<graf_int_tp>                     vec_graf_int_tp; // ## 3
typedef stapl::vector_view<vec_graf_int_tp>            vec_graf_int_vw_tp;


size_t ex_606(size_t, const char *, stapl::stream<ofstream> & );


stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ofstream> zout;
  zout.open("ex_606.out");

  size_t model = 1;
  stapl::do_once( msg( zout, "Example 606" ) );
  int result = ex_606(model, "d_v48_e216.txt", zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  zout.close();
  return EXIT_SUCCESS;
}

struct ex_606_build_wf {
  typedef void result_type;
  template <typename Graph, typename Elem1, typename Elem2, typename Elem3>
  result_type operator()(Graph graf, Elem1 count, Elem2 src, Elem3 dest) {
    if( src < count && dest < count ) { // ## 4
      typename Graph::edge_descriptor ed;
#ifdef GFORGE_1307_FIXED
      graf.add_edge(src,dest);
#endif
    }
  }
};

struct ex_606_init_wf {
  typedef void result_type;
  template<typename Vertex>
  result_type operator()(Vertex vtx) {
    vtx.property() = 1;
  }
};

struct ex_606_fill_wf {
  typedef void result_type;
  template <typename Elem1, typename Elem2, typename View3, typename View4>
  result_type operator()(Elem1 graf, Elem2 count, View3 vw3, View4 vw4) {
    stapl::map_func( ex_606_init_wf(), graf ); // ## 5
    stapl::map_func( ex_606_build_wf(), stapl::make_repeat_view(graf), // ## 6
                     stapl::make_repeat_view(count), vw3, vw4 );
  }
};

struct ex_606_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 vw1) {
   return stapl::map_reduce(inner_graf_wf(), add_int_wf(), vw1 ); // ## 7
  }
};

struct print_vertex_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  print_vertex_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Vertex>
  result_type operator()(Vertex vtx)
  {
    m_zout << "  [" << vtx.descriptor() << "], #" << vtx.size() << "  { ";
    typename Vertex::adj_edge_iterator it;
    for ( it = vtx.begin(); it != vtx.end(); ++it)
      m_zout << (*it).target() << ", ";
    m_zout << "}" << endl;
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct ex_606_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  ex_606_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1, typename Element>
  result_type operator()(View1 const &vw1, Element elem ) {
    m_zout << "[" << elem << "] # " << vw1.size() << endl;
    stapl::serial_io(print_vertex_wf(m_zout), vw1); // ## 8
    m_zout << endl;
  }
  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

size_t ex_606( size_t model, const char *filename,
               stapl::stream<ofstream> &zout ) {

  size_t outer = 100 * model; // ## 9
  size_t inner = 100 * model; // ## 10

  ary_int_tp len(outer);
  ary_int_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner)); // ## 11

  ary_graf_int_tp a(len_vw);
  ary_graf_int_vw_tp a_vw(a);

  size_t vtx_cnt=0, edge_cnt=0;
  ifstream xin;
  xin.open(filename);
  if( !xin.is_open() ) {
    stapl::do_once( msg( zout, "606 input open failed" ) );
    return EXIT_FAILURE;
  }
  xin >> vtx_cnt >> edge_cnt;
  xin.close();

  vec_int_tp src_ct(edge_cnt), dest_ct(edge_cnt);
  vec_int_vw_tp src_vw(src_ct), dest_vw(dest_ct);

  stapl::stream<ifstream> zin;
  zin.open(filename);
  zin >> vtx_cnt >> edge_cnt;
  stapl::serial_io(get_pair_wf(zin), src_vw, dest_vw ); // ## 12
  zin.close();

  stapl::map_func(ex_606_fill_wf(), a_vw, len_vw, // ## 13
                  stapl::make_repeat_view(src_vw),
                  stapl::make_repeat_view(dest_vw) );

  int res = stapl::map_reduce(ex_606_process_wf(), add_int_wf(), a_vw );

  stapl::serial_io(ex_606_show_wf(zout), a_vw,
                   stapl::counting_view<int>(a_vw.size()) );

  return res;
}

