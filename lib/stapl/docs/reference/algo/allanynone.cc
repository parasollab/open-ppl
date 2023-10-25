// algo/allanynone.cc

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
struct odd_pred
{
  typedef T    argument_type;
  typedef bool result_type;
  result_type operator() (const T& x) const
  {
    return 1 == (x % 2);
  }
};

template <typename T>
struct even_pred
{
  typedef T    argument_type;
  typedef bool result_type;
  result_type operator() (const T& x) const
  {
    return 0 == (x % 2);
  }
};

template <typename T>
struct neg_pred
{
  typedef T    argument_type;
  typedef bool result_type;
  result_type operator() (const T& x) const
  {
    return x < 0;
  }
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(40), w_ct(4), u_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct), u_vw(u_ct);

  // initialize container
  stapl::iota(v_vw, 0);

  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(v_vw, repeat_wf(base,rep));

  bool all_test, none_test, any_test;
  // apply algorithm
  all_test = stapl::all_of(v_vw, even_pred<val_tp>() );
  none_test = stapl::none_of(w_vw, neg_pred<val_tp>() );
  any_test = stapl::any_of(u_vw, odd_pred<val_tp>() );

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_allanynone.txt");
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg_val<val_tp>( zout, "all_test ", all_test ) );
  stapl::serial_io( put_val_wf(zout), w_vw );
  stapl::do_once( msg_val<val_tp>( zout, "none_test ", none_test ) );
  stapl::serial_io( put_val_wf(zout), u_vw );
  stapl::do_once( msg_val<val_tp>( zout, "any_test ", any_test ) );

  return EXIT_SUCCESS;
}
