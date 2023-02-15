// view/filter.cc

#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/views/filter_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

template<typename T>
struct filt_even_wf
{
   typedef T argument_type;
   typedef bool result_type;
   result_type operator() (const T& x) const
   {
     return (x%2) == 0;
   }
};

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

typedef stapl::minus<val_tp>    neg_int_wf;
typedef stapl::max<val_tp>      max_int_wf;


stapl::exit_code stapl_main(int argc, char **argv) {

  size_t sz = 40;
  // construct container
  vec_int_tp a_ct(sz), b_ct(sz), c_ct(sz);

  // construct view over container
  vec_int_vw_tp a_vw(a_ct), b_vw(b_ct), c_vw(c_ct);

  // initialize container
  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));

  int rep = 42;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(b_vw, repeat_wf(base,rep));

  // process elements through view
  typedef stapl::filter_view<vec_int_vw_tp,filt_even_wf<int> >
          filt_even_vec_int_vw_tp;

  filt_even_wf<int> filter_wf;
  filt_even_vec_int_vw_tp a_flt_vw =
                          filt_even_vec_int_vw_tp(a_vw, filter_wf);
  filt_even_vec_int_vw_tp b_flt_vw =
                          filt_even_vec_int_vw_tp(b_vw, filter_wf);

  val_tp sum = stapl::reduce( a_flt_vw, max_int_wf() );

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_filtvw.txt");
  stapl::serial_io( put_val_wf(zout), a_flt_vw );
  stapl::do_once( msg_val<val_tp>( zout, "Sum ", sum ) );

  return EXIT_SUCCESS;
}
