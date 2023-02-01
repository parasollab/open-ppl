#include <iostream>
#include <fstream>
#include "ch4.hpp"

typedef stapl::distribution_spec<> dist_spec_tp;
typedef vector<dist_spec_tp> vec_dist_spec_tp;

typedef stapl::array<int,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> > ary_int_tp;

typedef stapl::vector<ary_int_tp,
                      stapl::view_based_partition<dist_spec_tp>,
                      stapl::view_based_mapper<dist_spec_tp> > vec_ary_int_tp;

typedef stapl::array<vec_ary_int_tp ,
  stapl::view_based_partition<dist_spec_tp>,
  stapl::view_based_mapper<dist_spec_tp> > ary_vec_ary_int_tp; // ## 1

typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;
typedef stapl::array_view<ary_vec_ary_int_tp> ary_vec_ary_int_vw_tp; // ## 2


size_t ex_403(size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_403.out");

  size_t model = 1;
  stapl::do_once( msg( zout, "Example 403" ) );

  zin.open("primes_100000.txt");
  int result = ex_403(model, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  zin.close();

  return EXIT_SUCCESS;
}

struct ex_403_inner_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    int rep = 42;
    int base = 0;
    typedef stapl::block_sequence<int> repeat_wf;
    stapl::generate(vw1, repeat_wf(base,rep));
  }
};

struct ex_403_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(ex_403_inner_fill_wf(), vw1);
  }
};

struct ex_403_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  ex_403_inner_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct ex_403_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  ex_403_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(ex_403_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct ex_403_process_inner_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const& vw2) {
    stapl::map_func(stapl::assign<ary_int_vw_tp::value_type>(),vw1,vw2);
  }
};

struct ex_403_process_outer_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const& vw2) {
    stapl::map_func(ex_403_process_inner_wf(),vw1,vw2);
  }
};

size_t ex_403( size_t model,
               stapl::stream<ifstream>& zin, stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  size_t otr_blk_size = outer / (2 * stapl::get_num_locations() );
  size_t blk_size = inner / (2 * stapl::get_num_locations() );
  vec_dist_spec_tp comp_spec(3);

  comp_spec[0] = stapl::block(outer, otr_blk_size ); // ## 3
  comp_spec[1] = stapl::block(inner, blk_size ); // ## 4
  comp_spec[2] = stapl::block(inner, blk_size ); // ## 5

  ary_vec_ary_int_tp a(comp_spec), b(comp_spec); 
  ary_vec_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(ex_403_outer_fill_wf(), a_vw );

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(ex_403_outer_show_wf(zout), a_vw );
  stapl::do_once( msg( zout, "" ) );

  stapl::map_func(ex_403_process_outer_wf(), a_vw, b_vw);

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(ex_403_outer_show_wf(zout), a_vw );
  stapl::do_once( msg( zout, "" ) );

  return outer * inner * inner;
}
