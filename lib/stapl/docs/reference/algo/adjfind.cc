// algo/adjfind.cc

#include <stapl/runtime.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "algohelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

template <typename T>
struct eq_pred
{
  typedef T    argument_type;
  typedef bool result_type;
  result_type operator() (const T& x, const T& y) const
  {
    return x == y;
  }
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(40);
  vec_int_vw_tp v_vw(v_ct);

  // initialize container
  int base = 0;
  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(v_vw, repeat_wf(base,rep));

  // apply algorithm
  int ndx_p = -1;
  auto ref_p = stapl::adjacent_find( v_vw, eq_pred<val_tp>() );
  if( !stapl::is_null_reference(ref_p) ) {
    ndx_p = stapl::index_of(ref_p);
  }

  int ndx = -1;
  vec_int_vw_tp::reference ref = stapl::adjacent_find( v_vw );
  if( !stapl::is_null_reference(ref) ) {
    ndx = stapl::index_of(ref);
  }

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_adjfind.txt");
  stapl::do_once( msg_val<val_tp>( zout, "ndx_p ", ndx_p ) );
  stapl::do_once( msg_val<val_tp>( zout, "ndx ", ndx ) );
  stapl::serial_io( put_val_wf(zout), v_vw );

  return EXIT_SUCCESS;
}
