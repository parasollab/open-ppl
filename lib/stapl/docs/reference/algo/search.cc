// algo/search.cc

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
  vec_int_tp v_ct(40), w_ct(10);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct);

  // initialize container
  stapl::iota(v_vw, 0);
  stapl::iota(w_vw, 10);

  // apply algorithm
  int ndx_p = -1;
  auto ref_p = stapl::search( v_vw, w_vw, eq_pred<val_tp>() );
  if( !stapl::is_null_reference(ref_p) ) {
    ndx_p = stapl::index_of(ref_p);
  }

  int ndx = -1;
  vec_int_vw_tp::reference ref = stapl::search( v_vw, w_vw );
  if( !stapl::is_null_reference(ref) ) {
    ndx = stapl::index_of(ref);
  }

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_search.txt");

  stapl::do_once( msg_val<val_tp>( zout, "ndx_p ", ndx_p ) );
  stapl::do_once( msg_val<val_tp>( zout, "ndx ", ndx ) );

  return EXIT_SUCCESS;
}
