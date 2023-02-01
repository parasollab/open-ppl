#include <iostream>
#include <fstream>
#include "ch4.hpp"

typedef stapl::distribution_spec<> dist_spec_tp;
typedef vector<dist_spec_tp> vec_dist_spec_tp;

typedef stapl::vector<int,
                      stapl::view_based_partition<dist_spec_tp>,
                      stapl::view_based_mapper<dist_spec_tp> > vec_int_tp;

typedef stapl::array<vec_int_tp ,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> > ary_vec_int_tp;

typedef stapl::vector<ary_vec_int_tp ,
  stapl::view_based_partition<dist_spec_tp>,
  stapl::view_based_mapper<dist_spec_tp> > vec_ary_vec_int_tp; // ## 1

typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;
typedef stapl::vector_view<vec_ary_vec_int_tp> vec_ary_vec_int_vw_tp; // ## 2

size_t ex_404(size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_404.out");

  size_t model = 1;
  stapl::do_once( msg( zout, "Example 404" ) );

  zin.open("primes_100000.txt");
  int result = ex_404(model, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  zin.close();

  return EXIT_SUCCESS;
}

struct ex_404_inner_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::generate(vw1, stapl::random_sequence());
  }
};

struct ex_404_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(ex_404_inner_fill_wf(), vw1);
  }
};

struct ex_404_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  ex_404_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct ex_404_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  ex_404_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(ex_404_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct ex_404_process_inner_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const& vw2) {
    stapl::map_func(stapl::assign<vec_int_vw_tp::value_type>(),vw1,vw2);
  }
};

struct ex_404_process_outer_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const& vw2) {
    stapl::map_func(ex_404_process_inner_wf(),vw1,vw2);
  }
};

size_t ex_404( size_t model,
               stapl::stream<ifstream>& zin, stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  size_t blk_size = inner / (2 * stapl::get_num_locations() );
  vec_dist_spec_tp comp_spec(3);

  comp_spec[0] = stapl::cyclic(outer); // ## 3
  comp_spec[1] = stapl::balance(inner); // ## 4
  comp_spec[2] = stapl::block(inner, blk_size ); // ## 5

  vec_ary_vec_int_tp a(comp_spec), b(comp_spec); 
  vec_ary_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(ex_404_outer_fill_wf(), a_vw );

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(ex_404_outer_show_wf(zout), a_vw );
  stapl::do_once( msg( zout, "" ) );

  stapl::map_func(ex_404_process_outer_wf(), a_vw, b_vw);

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(ex_404_outer_show_wf(zout), a_vw );
  stapl::do_once( msg( zout, "" ) );

  return outer * inner * inner;
}
