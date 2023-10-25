#include <iostream>
#include <fstream>
#include "ch5.hpp"

#include <stapl/vector.hpp>
#include <stapl/views/filter_view.hpp>

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

int ex_506(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_506.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 506" ) );
  int result = ex_506(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

template<typename T> 
struct filt_even_wf // ## 1
{
   typedef T argument_type;
   typedef bool result_type;
   result_type operator() (const T& x) const
   {
     return (x%2) == 0;
   }
};

int ex_506(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  vec_int_tp a(sz), b(sz), c(sz);
  vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));

  int rep = 42;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(b_vw, repeat_wf(base,rep));

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(put_val_wf(zout), a_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::do_once( msg( zout, "b:" ) );
  stapl::serial_io(put_val_wf(zout), b_vw);
  stapl::do_once( msg(zout, "\n" ) );

  typedef stapl::filter_view<vec_int_vw_tp,filt_even_wf<int> > // ## 2
          filt_even_vec_int_vw_tp;

  filt_even_wf<int> filter_wf; // ## 3
  filt_even_vec_int_vw_tp a_flt_vw = 
                          filt_even_vec_int_vw_tp(a_vw, filter_wf); // ## 4
  filt_even_vec_int_vw_tp b_flt_vw = 
                          filt_even_vec_int_vw_tp(b_vw, filter_wf); 
  filt_even_vec_int_vw_tp c_flt_vw = 
                          filt_even_vec_int_vw_tp(c_vw, filter_wf); 

#ifdef GFORGE_1306_FIXED
  stapl::transform(a_flt_vw, c_vw, neg_int_wf() ); // ## 5

  int red = stapl::reduce( a_flt_vw, max_int_wf() ); // ## 6
#endif

  stapl::do_once( msg( zout, "c:" ) );
  stapl::serial_io(put_val_wf(zout), c_vw);
  stapl::do_once( msg(zout, "\n" ) );

  int val = c_vw[sz-1];
  stapl::rmi_fence();
#ifdef GFORGE_1306_FIXED
  return red + val;
#else
  return val;
#endif
}
