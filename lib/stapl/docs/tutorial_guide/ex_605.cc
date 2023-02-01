#include <iostream>
#include <fstream>
#include "ch6.hpp"

class my_vertex_property // ## 1
{
private:
  size_t m_size_val;
  double m_double_val;

public:
  my_vertex_property(size_t x = 0 , double y = 0.0)
    : m_size_val(x) , m_double_val(y)
  { }

  my_vertex_property(my_vertex_property const& other)
    : m_size_val(other.m_size_val) , m_double_val(other.m_double_val)
  { }

  size_t get_size_prop(void) const // ## 4
  { return m_size_val; }

  void set_size_prop(size_t const& x)
  { m_size_val = x; }

  double get_double_prop(void) const // ## 3
  { return m_double_val; }

  void set_double_prop(double const& x)
  { m_double_val = x; }

  void define_type(stapl::typer& t)
  { t.member(m_size_val);
    t.member(m_double_val);
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  my_vertex_property const& x ) {
    os << "(" << x.get_size_prop() << "," << x.get_double_prop() << ") ";
    return os;
  }
};

namespace stapl {

STAPL_PROXY_HEADER(my_vertex_property) // ## 4
{
  STAPL_PROXY_DEFINES(my_vertex_property) // ## 5
  STAPL_PROXY_METHOD_RETURN(get_size_prop, size_t) // ## 6
  STAPL_PROXY_METHOD(set_size_prop, size_t) // ## 7
  STAPL_PROXY_METHOD_RETURN(get_double_prop, double)
  STAPL_PROXY_METHOD(set_double_prop, double)

  friend std::ostream& operator<<(std::ostream& os,
                                  proxy<target_t, Accessor> const& x ) {
    os << "(" << x.get_size_prop() << "," << x.get_double_prop() << ") ";
    return os;
  }
};

}

typedef stapl::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
               my_vertex_property,
               stapl::properties::no_property>  graf_int_tp;
typedef stapl::graph_view<graf_int_tp>          graf_int_vw_tp;

typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
               my_vertex_property,
               int>                             digraf_int_tp;
typedef stapl::graph_view<digraf_int_tp>        digraf_int_vw_tp;


size_t ex_605a( string const &, stapl::stream<ofstream> &);
size_t ex_605b( string const &, stapl::stream<ofstream> &);


stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ofstream> zout;
  zout.open("ex_605.out");
  size_t result;

  stapl::do_once( msg( zout, "Example 605a" ) );
  result = ex_605a("u_v88_e634.txt", zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  stapl::do_once( msg( zout, "Example 605b" ) );
  result = ex_605b("d_v88_e1117.txt", zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );

  zout.close();
  return EXIT_SUCCESS;
}

struct ex_605a_set_wf {
  typedef void result_type;
  template<typename Vertex, typename Elem>
  result_type operator()(Vertex vtx, Elem elem) {
    vtx.property().set_size_prop(elem); // ## 8
  }
};

size_t ex_605a( string const &filename, stapl::stream<ofstream> &zout ) {

  graf_int_vw_tp gr_vw = stapl::read_edge_list<graf_int_tp>(filename);

  gr_vw[0].property().set_double_prop(3.14159); // ## 9
  gr_vw[1].property().set_double_prop(2.71838);

  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ) );
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ) );

  stapl::map_func( ex_605a_set_wf(), gr_vw,
                   stapl::counting_view<int>(gr_vw.size()) ); // ## 10

  stapl::write_adj_list<graf_int_vw_tp>(gr_vw, string("ex_605a.out") ); // ## 11

  return gr_vw.num_vertices();
}

struct ex_605b_reset_wf {
  typedef void result_type;
  template<typename Vertex, typename Elem>
  result_type operator()(Vertex vtx, Elem elem) {
    vtx.property().set_double_prop(elem*.1); // ## 12
    typename Vertex::adj_edge_iterator aei;
    for( aei = vtx.begin(); aei != vtx.end(); ++aei ) {
      (*aei).property() = 1; // ## 13
    }
  }
};

size_t ex_605b( string const &filename, stapl::stream<ofstream> &zout ) {

  digraf_int_vw_tp gr_vw = stapl::read_edge_list<digraf_int_tp>(filename);

  stapl::map_func( ex_605b_reset_wf(), gr_vw,
                   stapl::counting_view<int>(gr_vw.size()) ); // ## 14

  gr_vw[0].property().set_size_prop(314159); // ## 15
  gr_vw[1].property().set_size_prop(271838);

  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ) );
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ) );

  stapl::write_adj_list<digraf_int_vw_tp>(gr_vw, string("ex_605b.out") ); // ## 16

  return gr_vw.num_vertices();
}

