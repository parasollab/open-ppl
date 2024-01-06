#include <iostream>
#include <fstream>
#include "ch6.hpp"

typedef stapl::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
               ary_int_tp, int>                        graf_ary_int_tp; // ## 1
typedef stapl::graph_view<graf_ary_int_tp>             graf_ary_int_vw_tp;

size_t ex_607(size_t, stapl::stream<ofstream> & );

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ofstream> zout;
  zout.open("ex_607.out");

  size_t model = 1;
  stapl::do_once( msg( zout, "Example 607" ) );
  int result = ex_607(model, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

struct ex_607_set_vtx_wf {
  typedef void result_type;
  template <typename Vtx>
  result_type operator()(Vtx vtx) {
    stapl::generate( vtx.property(), stapl::random_sequence(1000)); // ## 2
  }
};

struct ex_607_add_edge_wf {
  typedef void result_type;
  template <typename Elem1, typename Elem2, typename Graph>
  result_type operator()(Elem1 src, Elem2 dest, Graph graf) {
    graf.add_edge_async(src,dest);
  }
};

struct ex_607_process_wf {
  typedef int result_type;
  template <typename Vtx>
  result_type operator()(Vtx vtx) {
    return vtx.property().size(); // ## 3
  }
};

struct ex_607_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  ex_607_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Vertex>
  result_type operator()(Vertex vtx) {
    m_zout << "[" << vtx.descriptor() << "]" << endl;

    m_zout << "  vertex array: [ ";
    for (auto&& e : vtx.property())
      m_zout << e << " ";
    m_zout << "]" << endl;

    m_zout << "  edge count:   " << vtx.size() << endl;
    m_zout << "  end vertices: { ";
    typename Vertex::adj_edge_iterator it;
    for ( it = vtx.begin(); it != vtx.end(); ++it)  {
      m_zout << (*it).target() << ", ";
    }
    m_zout << "}" << endl << endl;
  }

  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

size_t ex_607( size_t model, stapl::stream<ofstream> &zout ) {

  size_t outer = 100 * model; // ## 4
  size_t inner = 100 * model; // ## 5

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner)); // ## 6

  graf_ary_int_tp a(len_vw);
  graf_ary_int_vw_tp a_vw(a);

  size_t edge_cnt = 3 * outer; // ## 7
  vec_int_tp src_ct(edge_cnt), dest_ct(edge_cnt);
  vec_int_vw_tp src_vw(src_ct), dest_vw(dest_ct);

  stapl::generate( src_vw, stapl::random_sequence(outer-1) ); // ## 8
  stapl::generate( dest_vw, stapl::random_sequence(outer-1) );

  stapl::map_func( ex_607_add_edge_wf(),
                   src_vw, dest_vw, stapl::make_repeat_view(a_vw) ); // ## 9
  stapl::rmi_fence();

  stapl::serial_io(ex_607_show_wf(zout), a_vw );

  stapl::map_func( ex_607_set_vtx_wf(), a_vw); // ## 10

  return stapl::map_reduce(ex_607_process_wf(), add_int_wf(), a_vw ); // ## 11
}
