#include <iostream>
#include <fstream>
#include "ch5.hpp"

#include <stapl/array.hpp>
#include <stapl/views/overlap_view.hpp>

typedef stapl::array<int>              ary_int_tp;
typedef stapl::array_view<ary_int_tp>  ary_int_vw_tp;
typedef stapl::array<size_t>           ary_size_tp;
typedef stapl::array_view<ary_size_tp> ary_size_vw_tp;

typedef stapl::overlap_view<ary_int_vw_tp>  ovl_ary_int_vw_tp; // ## 1

size_t ex_505(size_t,
              stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_505.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 505" ) );
  int result = ex_505(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

class show_overlaps_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  show_overlaps_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename SegRef>
  result_type operator() (SegRef seg) {
    m_zout << "  [ ";
    stapl::serial_io(put_val_wf(m_zout), seg);  // ## 2
    m_zout << "]\n";
  }

  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

struct inner_prod_wf {
  typedef void result_type;
  template<typename SegRef1, typename SegRef2, typename Elem>
  result_type operator()(SegRef1 x, SegRef2 y, Elem z) {
    typename SegRef1::iterator iter_x = x.begin();
    typename SegRef2::iterator iter_y = y.begin();

    z = 0;
    while(iter_x != x.end())
      z += (*iter_x++) * (*iter_y++); // ## 3
  }
};



size_t ex_505(size_t sz,
              stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  ary_int_tp a_ct(sz), b_ct(sz);
  ary_int_vw_tp a_vw(a_ct), b_vw(b_ct);

  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));

  stapl::iota(b_vw, 0);

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(a_vw.size()) );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::do_once( msg( zout, "b:" ) );
  stapl::serial_io(show_log_ndx_wf(zout), b_vw,
                   stapl::counting_view<int>(b_vw.size()) );
  stapl::do_once( msg(zout, "\n" ) );

  int left = 1; // ## 4
  int right = 1;
  int core = 1;
  ovl_ary_int_vw_tp a_ovl_vw = make_overlap_view(a_vw,core,left,right); // ## 5
  ovl_ary_int_vw_tp b_ovl_vw = make_overlap_view(b_vw,core,left,right);


  stapl::do_once( msg( zout, "a_ovl:\n[" ) );
  stapl::serial_io(show_overlaps_wf(zout), a_ovl_vw);
  stapl::do_once( msg( zout, "]\n" ) );

  stapl::do_once( msg( zout, "b_ovl:\n[" ) );
  stapl::serial_io(show_overlaps_wf(zout), b_ovl_vw);
  stapl::do_once( msg( zout, "]\n" ) );

  ary_size_tp c_ct(a_ovl_vw.size());
  ary_size_vw_tp c_vw(c_ct);

  stapl::map_func( inner_prod_wf(), a_ovl_vw, b_ovl_vw, c_vw ); // ## 6

  stapl::do_once( msg( zout, "c:" ) );
  stapl::serial_io(show_log_ndx_wf(zout), c_vw,
                   stapl::counting_view<size_t>(c_vw.size()) );
  stapl::do_once( msg(zout, "\n" ) );

  size_t val = c_vw[c_vw.size()-1];
  stapl::rmi_fence();
  return val;
}
