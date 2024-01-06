// cont/set.cc

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/containers/unordered_set/unordered_set.hpp>
#include <stapl/containers/set/set.hpp>
#include <stapl/views/set_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::set<val_tp> set_val_tp;
typedef stapl::unordered_set<val_tp> unord_set_val_tp;
typedef stapl::set_view<set_val_tp> set_int_vw_tp;
typedef stapl::set_view<unord_set_val_tp> unord_set_int_vw_tp;

int prime[20] = { 2,  3,  5,  7, 11, 13, 17, 19, 23, 29,
                 31, 37, 41, 43, 47, 53, 59, 61, 67, 71 };

int fibo[20] = { 1, 2, 3, 4, 8, 13, 21, 34, 55, 89,
               144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946 };

typedef stapl::array<int> ary_val_tp;
typedef stapl::array_view<ary_val_tp> ary_int_vw_tp;

struct init_set_wf {
  typedef void result_type;
  template<typename Key, typename Set>
  result_type operator() (Key key, Set &set) {
    set.insert(key);
  }
};


stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container
  ary_val_tp key_ct(20+20);
  ary_int_vw_tp key_vw(key_ct);

  // initialize key container with sequential loop
  stapl::do_once([&]() {
    size_t i = 0;
    for ( ; i<20; i++ ) {
      key_vw[i] = fibo[i];
    }
    for ( ; i<20+20; i++ ) {
      size_t j = (i < 20) ? i : i % 20;
      key_vw[i] = prime[j];
    }
  });
  stapl::rmi_fence();

  // construct container
  set_val_tp s_ct;

  // construct view over container
  set_int_vw_tp t_vw(s_ct);

  // initialize container in parallel
  stapl::map_func( init_set_wf(), 
                   key_vw, stapl::make_repeat_view(t_vw) );
  set_int_vw_tp s_vw(s_ct);

  // process elements in parallel
  val_tp sum = stapl::map_reduce( stapl::identity<val_tp>(),
                                  stapl::plus<val_tp>(), s_vw );

  // print size
  stapl::stream<ofstream> zout;
  zout.open("refman_setvw.txt");
  stapl::do_once( msg_val<val_tp>( zout, "Size ", s_vw.size() ) );

  // print elements
  stapl::serial_io( put_val_wf(zout), s_vw );
  stapl::do_once( msg_val<val_tp>( zout, "Sum ", sum ) );

  return EXIT_SUCCESS;
}
