// algo/count.cc

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
struct even_pred
{
  typedef T    argument_type;
  typedef bool result_type;
  result_type operator() (const T& x) const
  {
    return 0 == (x % 2);
  }
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(40), w_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct);

  // initialize containers
  int base = 10;
  int step = 1;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(v_vw, repeat_wf(base,rep));

  // apply algorithm
  auto v_cnt = stapl::count_if(v_vw, even_pred<val_tp>() );
  auto w_cnt = stapl::count(w_vw, v_vw[v_vw.size()/2]);

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_count.txt");
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg_val<val_tp>( zout, "v_cnt ", v_cnt ) );
  stapl::serial_io( put_val_wf(zout), w_vw );
  stapl::do_once( msg_val<val_tp>( zout, "w_cnt ", w_cnt ) );

  return EXIT_SUCCESS;
}
