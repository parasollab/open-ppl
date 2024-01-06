#include <iostream>
#include <fstream>

#include <stapl/utility/do_once.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/views/map_view.hpp>
#include <stapl/containers/map/map.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/runtime.hpp>
#include <stapl/stream.hpp>
#include <stapl/skeletons/serial.hpp>
#include <stapl/stream.hpp>

using namespace std;
#include "ch2_msg.hpp"
#include "ch2_put.hpp"

typedef stapl::map<int,int> map_int_tp;
typedef stapl::map_view<map_int_tp> map_int_vw_tp;
typedef stapl::indexed_domain<int> ndx_dom_tp;

typedef stapl::identity<int> id_int_wf;
typedef stapl::negate<int> neg_int_wf;
typedef stapl::minus<int> sub_int_wf;
typedef stapl::max<int> max_int_wf;
typedef stapl::min<int> min_int_wf;
typedef stapl::plus<int> add_int_wf;

extern int prime_nums[], rand_nums[];

struct map_int_val_wf {
  typedef int result_type;
  template<typename Pair>
  result_type operator()(Pair const &pair) {
    return pair.second; // ## 1
  }
};

struct get_map_key_val_wf {
private:
  stapl::stream<ifstream> zin;
public:
  typedef void result_type;

  get_map_key_val_wf(stapl::stream<ifstream> const& in)
    : zin(in)
  { }

  template <typename Ref>
  result_type operator()(Ref pair) {
    zin >> pair.second; // ## 2
  }

  void define_type(stapl::typer& t) {
    t.member(zin);
  }
};

class ex_211a_insert_wf {
private:
  size_t size;
public:
  ex_211a_insert_wf(size_t s)
    : size(s)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a){
    for ( size_t i = 0; i < size; i++ ) {
      a[ prime_nums[i] ] = rand_nums[i]; // ## 3
    }
  }
  void define_type(stapl::typer& t) {
    t.member(size);
  }
};

class ex_211b_insert_wf {
private:
  size_t size;
public:
  ex_211b_insert_wf(size_t s)
    : size(s)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a){
    for ( size_t i = 0; i < size; i++ ) {
      a[ prime_nums[i] - 1 ] = prime_nums[i];
    }
  }
  void define_type(stapl::typer& t) {
    t.member(size);
  }
};

int ex_211(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  zin.open("lexord_1000.txt");
  stapl::stream<ofstream> zout;
  zout.open("ex_211.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 211" ) );

  int result = ex_211(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

int ex_211(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  ndx_dom_tp a_dom(0, prime_nums[sz-1]);
  map_int_tp a(a_dom);
  map_int_vw_tp a_vw(a);

  stapl::do_once( ex_211a_insert_wf(sz), a_vw );

  int elem = prime_nums[sz/2];

  int found = a_vw.count(elem); // ## 4
  stapl::do_once( msg_val<int>( zout, "Value ", elem ) );
  stapl::do_once( msg_val<int>( zout, "occurs ", found ) );

  stapl::do_once( msg( zout, "a 1:") );
  stapl::serial_io(put_map_val_wf(zout), a_vw);
  stapl::do_once( msg(zout, "\n" ) );

  stapl::serial_io(get_map_key_val_wf(zin), a_vw); // ## 5
  stapl::do_once( msg( zout, "a 2:") );
  stapl::serial_io(put_map_val_wf(zout), a_vw);
  stapl::do_once( msg(zout, "\n" ) );

  stapl::do_once( ex_211b_insert_wf(sz), a_vw );

  int tot = stapl::map_reduce( map_int_val_wf(), add_int_wf(), a_vw); //  ## 6

  stapl::do_once( msg( zout, "a 3:") );
  stapl::serial_io(put_map_val_wf(zout), a_vw);
  stapl::do_once( msg(zout, "\n" ) );

  return tot;
}
