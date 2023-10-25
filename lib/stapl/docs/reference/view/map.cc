// cont/map.cc

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/containers/unordered_map/unordered_map.hpp>
#include <stapl/containers/map/map.hpp>
#include <stapl/views/map_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::map<val_tp,val_tp> map_val_tp;
typedef stapl::unordered_map<val_tp,val_tp> unord_map_val_tp;
typedef stapl::map_view<map_val_tp> map_int_vw_tp;
typedef stapl::map_view<unord_map_val_tp> unord_map_int_vw_tp;
typedef stapl::indexed_domain<val_tp> ndx_dom_tp;

template <typename T>
struct map_val_wf {
  typedef T result_type;
  template<typename Pair>
  result_type operator() (Pair const &pair) {
    return pair.second;
  }
};

typedef stapl::plus<int> add_int_wf;

typedef stapl::array<int> ary_val_tp;
typedef stapl::array_view<ary_val_tp> ary_int_vw_tp;

struct init_map_wf {
  typedef void result_type;
  template<typename Key, typename Val, typename Map>
  result_type operator() (Key key, Val val, Map &map) {
    map[key] = val;
  }
};


int prime[20] = { 2,  3,  5,  7, 11, 13, 17, 19, 23, 29,
                 31, 37, 41, 43, 47, 53, 59, 61, 67, 71 };

int fibo[20] = { 1, 2, 3, 4, 8, 13, 21, 34, 55, 89,
               144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946 };


stapl::exit_code stapl_main(int argc, char **argv) {

  // construct key and value containers
  ary_val_tp key_ct(20), val_ct(20);
  ary_int_vw_tp key_vw(key_ct), val_vw(val_ct);

  // initialize key and value containers with sequential loop
  stapl::do_once([&]() {
    for ( size_t i=0; i<20; i++ ) {
      key_vw[i] = prime[i];
      val_vw[i] = fibo[i%20];
    }
  });
  stapl::rmi_fence();

  // construct container
  map_val_tp m_ct(ndx_dom_tp(0,16777216));

  // construct view over container
  map_int_vw_tp m_vw(m_ct);

  // initialize container in parallel
  stapl::map_func( init_map_wf(), 
                   key_vw, val_vw, stapl::make_repeat_view(m_vw) );

  // process elements in parallel
  val_tp sum = stapl::map_reduce( map_val_wf<val_tp>(),
                                  stapl::plus<val_tp>(), m_vw );

  // print size
  stapl::stream<ofstream> zout;
  zout.open("refman_mapvw.txt");
  stapl::do_once( msg_val<val_tp>( zout, "Size ", m_vw.size() ) );

  // print elements
  stapl::serial_io( put_map_val_wf(zout), m_vw );
  stapl::do_once( msg_val<val_tp>( zout, "Sum ", sum ) );

  return EXIT_SUCCESS;
}

