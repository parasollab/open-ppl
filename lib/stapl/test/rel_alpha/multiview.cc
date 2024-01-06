/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cstdio>
#include <cstdlib>

#include <iostream>

#include <stapl/utility/tuple.hpp>
#include <stapl/domains/indexed.hpp>

#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/containers/map/map.hpp>
#include <stapl/views/map_view.hpp>

#include <stapl/views/strided_view.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/views/overlap_view.hpp>
#include <stapl/views/filter_view.hpp>
#include <stapl/views/reverse_view.hpp>
#include <stapl/views/transform_view.hpp>
#include <stapl/views/segmented_view.hpp>
#include <stapl/views/zip_view.hpp>

#include <stapl/skeletons/serial.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include <stapl/utility/do_once.hpp>

#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include "json_parser.hpp"

using namespace std;
#include "testutil.hpp"
#include "rel_alpha_data.h"
#include "rel_alpha_util.hpp"

// this bug prevents scan from being applied to any composed view
//#define BUG_SCAN_COMPOSE

// these bugs prevent composing a list view over an array, vector, or map
// or composing a vector or array view over a list
// code is turned on, so statements under these controls need
// to have "_vw1" to "_vw2" in order to demonstrate the problem
#define BUG_ELEM_LIST
#define BUG_RED_LIST
#define BUG_SCAN_LIST

// this bug prevents composing a map view with a larger domain
// over a map view with more restricted domain
//#define MAP_DOMAIN_BUG

// turn on these statements once the bugs are fixed
//#define CLEANUP

/*=========================================================================*/

typedef stapl::indexed_domain<size_t> ndx_dom_tp;

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view< vec_int_tp > vec_int_vw_tp;

typedef stapl::array<int> ary_int_tp;
typedef stapl::array_view< ary_int_tp > ary_int_vw_tp;

typedef stapl::list<int> list_int_tp;
typedef stapl::list_view<list_int_tp> list_int_vw_tp;

typedef stapl::map<size_t,size_t> map_int_tp;
typedef stapl::map_view<map_int_tp> map_int_vw_tp;

typedef stapl::splitter_partition<vec_int_vw_tp::domain_type>
        seg_vec_splitter;
typedef stapl::segmented_view<vec_int_vw_tp, seg_vec_splitter>
        seg_vec_vw_tp;
typedef stapl::splitter_partition<ary_int_vw_tp::domain_type>
        seg_ary_splitter;
typedef stapl::segmented_view<ary_int_vw_tp, seg_ary_splitter>
        seg_ary_vw_tp;

/*=========================================================================*/

template <typename T>
struct filter1
{
  typedef T   argument_type;
  typedef int result_type;

  bool operator() (const T& x) const
  {
    return x < 2000;
  }
};

template <typename T>
struct filter2
{
  typedef T    first_argument_type;
  typedef T    second_argument_type;
  typedef bool result_type;

  bool operator() (const T& x, const T& y) const
  {
    return 0 == (x % y);
  }
};

struct seg_flags_wf {
  typedef int result_type;
  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 key, Ref2 seg_flag) {
    int prev = key[0];
    int count = 0;
    int limit = key.size();
    for ( int i=1; i < limit; i++ ) {
      if ( prev != key[i] ) {
        prev = key[i];
        seg_flag[i-1] = 1;
        count++;
      }
    }
    seg_flag[limit-1] = 1;
    count++;
    return count;
  }
};

struct seg_counts_wf {
  typedef int result_type;
  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 seg_flag, Ref2 seg_cnt) {
    int count = 1;
    size_t j = 0;
    int limit = seg_flag.size();
    for ( int i=0; i<limit; i++ ) {
      if ( seg_flag[i] == 1 ) {
        seg_cnt[j++] = count;
        count = 1;
      } else {
        count++;
      }
    }
    return count;
  }
};

/*==========================================================================*/


typedef stapl::strided_view<vec_int_vw_tp>::type str_vec_int_vw_tp;
typedef stapl::view_impl::reverse_view<vec_int_vw_tp> rev_vec_int_vw_tp;
typedef stapl::filter_view<vec_int_vw_tp,filter1<int> > filt_vec_int_vw_tp;
typedef stapl::view_impl::overlap_view_builder<vec_int_vw_tp>::view_type
  ovl_vec_int_vw_tp;

typedef stapl::zip_view<ary_int_vw_tp, ary_int_vw_tp,
                        ary_int_vw_tp > zip3_int_vw_tp;

typedef stapl::list_view<ary_int_vw_tp> list_ary_int_vw_tp;
typedef stapl::list_view<vec_int_vw_tp> list_vec_int_vw_tp;
typedef stapl::list_view<map_int_vw_tp> list_map_int_vw_tp;

typedef stapl::array_view<list_int_vw_tp> ary_list_int_vw_tp;
typedef stapl::array_view<vec_int_vw_tp> ary_vec_int_vw_tp;
typedef stapl::array_view<map_int_vw_tp> ary_map_int_vw_tp;

typedef stapl::vector_view<list_int_vw_tp> vec_list_int_vw_tp;
typedef stapl::vector_view<ary_int_vw_tp> vec_ary_int_vw_tp;
typedef stapl::vector_view<map_int_vw_tp> vec_map_int_vw_tp;

typedef stapl::array_view<ary_int_vw_tp> ary_ary_int_vw_tp;
typedef stapl::vector_view<vec_int_vw_tp> vec_vec_int_vw_tp;
typedef stapl::map_view<map_int_vw_tp> map_map_int_vw_tp;

/*=========================================================================*/

void open_zin(int model, int test, stapl::stream<ifstream>& zin) {
  switch ( model ) {
  case 1:
    switch( test ) {
    default:
      zin.open("data/tiny_primes.zin");
      break;
    }
    break;
  case 100:
    switch( test ) {
    default:
      zin.open("data/small_primes.zin");
      break;
   }
    break;
  case 10000:
    switch( test ) {
    default:
      zin.open("data/medium_primes.zin");
      break;
    }
    break;
  case 1000000:
    switch( test ) {
    default:
      zin.open("data/big_primes.zin");
      break;
    }
    break;
  case 100000000:
    switch( test ) {
    default:
      zin.open("data/huge_primes.zin");
      break;
    }
    break;
  }
}

/*=========================================================================*/

size_t mv_101( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_102( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_103( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_104( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_105( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_106( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_107( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_108( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_109( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_110( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );

size_t mv_111( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_112( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_113( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );

size_t mv_114( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_115( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_116( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );

size_t mv_117( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_118( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );

size_t mv_121( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_122( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_123( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_124( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_125( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_126( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_127( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_128( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_129( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_130( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_130( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t mv_132( size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );

/*=========================================================================*/

bool opt_list = false, opt_noref = false;
bool opt_out = false, opt_quiet = false;
int opt_test = -1;
char *opt_data = 0;
bool opt_stapl= false;
bool opt_manual= false;

stapl::exit_code stapl_main(int argc, char **argv) {

  char *temp = 0;
  for ( int argi = 1; argi < argc; ) {
    char * opt = argv[argi++];
    if ('-' == opt[0] ) {
      switch ( opt[1] ) {
      case 'h':
        cerr << "HELP\n";
        break;
      case 'd':
        opt_data = argv[argi++];
        break;
      case 'l':
        opt_list = true;
        break;
      case 'n':
        opt_noref = true;
        break;
      case 'o':
        opt_out = true;
        break;
      case 'q':
        opt_quiet = true;
        break;
      case 't':
        temp = argv[argi++];
        opt_test = atoi(temp);
        break;
      case 'v':
        temp = argv[argi++];
        opt_stapl = (0 == strcmp("stapl", temp) );
        opt_manual = (0 == strcmp("manual", temp) );
        break;
      }
    } else {
      cerr << "unknown command line argument " << opt << endl;
    }
  }
  if (!opt_stapl && !opt_manual ) {
    // default behavior
    opt_stapl = true;
    opt_manual = true;
  }

  if (opt_list ) {
    // Boost.PropertyTree will only serialize a node with unnamed subkeys
    // as an array.  Therefore we've got to build extra trees in order to
    // create the unnamed subkeys needed to form proper JSON output.
    typedef boost::property_tree::ptree ptree;
    ptree manual_tree;
    ptree stapl_tree;
    manual_tree.push_back(std::make_pair("name", ptree("stapl")));
    stapl_tree.push_back(std::make_pair("name", ptree("manual")));
    ptree version_array;
    version_array.push_back(std::make_pair("", manual_tree));
    version_array.push_back(std::make_pair("", stapl_tree));

    ptree test_array;
    for ( int i = 1; i<25; i++ ) {
      ptree test;
      char buf[8];
      sprintf(buf,"t%03d",i);
      test.push_back(std::make_pair(std::string(buf), version_array));
      test_array.push_back(std::make_pair("", test));
    }

    // The array of tests is the property of a key named "tests".
    ptree test_tree;
    test_tree.push_back(std::make_pair("tests", test_array));
    write_json(std::cout, test_tree, false);
    return EXIT_SUCCESS;
  }

  int model = -1;
  switch ( opt_data[0] ) {
  case 't':
    model = 1;
    break;
  case 's':
    model = 100;
    break;
  case 'm':
    model = 10000;
    break;
  case 'b':
    model = 10000000;
    break;
  case 'h':
    model = 100000000;
    break;
  default:
    cerr << "opt_data " << opt_data << endl;
    break;
  }
  if (model == -1) {
    std::cerr << "usage: exe -data tiny/small/medium/big/huge\n";
    exit(1);
  }

  int first_test = 1;
  int last_test = 36;
  if (opt_test != -1) {
    first_test = opt_test;
    last_test = opt_test;
  }

  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;

  bool ok = true;
  first_test = 101; last_test = 112;
  for (int test=first_test; test<=last_test; test++ ) {
    if (stapl::get_location_id() == 0 ) {
        std::cout << "Multiview " << test << endl;
    }
    set_random_seed();
    int result = 0;
    switch ( test) {

    case 101:
      zout.open("mv101.zout");
      result = mv_101(model, zin, zout);
      break;
    case 102:
      zout.open("mv102.zout");
      result = mv_102(model, zin, zout);
      break;
    case 103:
      zout.open("mv103.zout");
      result = mv_103(model, zin, zout);
      break;
    case 104:
      zout.open("mv104.zout");
      result = mv_104(model, zin, zout);
      break;
    case 105:
      zout.open("mv105.zout");
      result = mv_105(model, zin, zout);
      break;

    case 106:
      zout.open("mv106.zout");
      result = mv_106(model, zin, zout);
      break;
    case 107:
      zout.open("mv107.zout");
      result = mv_107(model, zin, zout);
      break;
    case 108:
      zout.open("mv108.zout");
      result = mv_108(model, zin, zout);
      break;
    case 109:
      zout.open("mv109.zout");
      result = mv_109(model, zin, zout);
      break;
    case 110:
      zout.open("mv110.zout");
      result = mv_110(model, zin, zout);
      break;

    case 111:
      zout.open("mv111.zout");
      result = mv_111(model, zin, zout);
      break;
    case 112:
      zout.open("mv112.zout");
      result = mv_112(model, zin, zout);
      break;
    case 113:
      zout.open("mv113.zout");
      result = mv_113(model, zin, zout);
      break;

    case 114:
      zout.open("mv114.zout");
      result = mv_114(model, zin, zout);
      break;
    case 115:
      zout.open("mv115.zout");
      result = mv_115(model, zin, zout);
      break;
    case 116:
      zout.open("mv116.zout");
      result = mv_116(model, zin, zout);
      break;

    case 117:
      zout.open("mv117.zout");
      result = mv_117(model, zin, zout);
      break;
    case 118:
      zout.open("mv118.zout");
      result = mv_118(model, zin, zout);
      break;

    case 121:
      zout.open("mv121.zout");
      ok = mv_121(model, zin, zout);
      break;
    case 122:
      zout.open("mv122.zout");
      ok = mv_122(model, zin, zout);
      break;
    case 123:
      zout.open("mv123.zout");
      ok = mv_123(model, zin, zout);
      break;
    case 124:
      zout.open("mv124.zout");
      ok = mv_124(model, zin, zout);
      break;
    case 125:
      zout.open("mv125.zout");
      ok = mv_125(model, zin, zout);
      break;
    case 126:
      zout.open("mv126.zout");
      ok = mv_126(model, zin, zout);
      break;
    case 127:
      zout.open("mv127.zout");
      ok = mv_127(model, zin, zout);
      break;
    case 128:
      zout.open("mv128.zout");
      ok = mv_128(model, zin, zout);
      break;
    case 129:
      zout.open("mv129.zout");
      ok = mv_129(model, zin, zout);
      break;
    case 130:
      zout.open("mv130.zout");
      ok = mv_130(model, zin, zout);
      break;
    case 131:
      zout.open("mv131.zout");
      ok = mv_130(model, zin, zout);
      break;
    case 132:
      zout.open("mv132.zout");
      ok = mv_132(model, zin, zout);
      break;
    }
    zout.close();
    if (stapl::get_location_id() == 0 ) {
        std::cout << endl << "Multiview: " << result << endl;
    }
  }
  return EXIT_SUCCESS;
}

/*==========================================================================*/

///////////////////////////////////////////////////////////////////////////
// vector : vector view : strided view
// generated input into base view
// basic algorithms on strided view
///////////////////////////////////////////////////////////////////////////

struct mv_101_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

size_t mv_101( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(y);

  int stride = 2;
  str_vec_int_vw_tp a_str_vw = stapl::make_strided_view(a_vw, stride, 0);
  str_vec_int_vw_tp b_str_vw = stapl::make_strided_view(b_vw, stride, 0);
  str_vec_int_vw_tp x_str_vw = stapl::make_strided_view(x_vw, stride, 0);
  str_vec_int_vw_tp y_str_vw = stapl::make_strided_view(y_vw, stride, 0);

  // initialize base views
  int base = 10;
  int step = 4;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step) );

  stapl::iota( b_vw, 0 );

  // basic algorithms
  stapl::map_func( mv_101_elem_wf(), a_str_vw, b_str_vw, x_str_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::scan( b_str_vw, y_str_vw, add_int_wf() );
#endif
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), a_str_vw );

  size_t cksum = stapl::reduce( a_str_vw, xor_un_wf() );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_str_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_str_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV101","STAPL",time1);

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// vector : vector view : reverse view
// generated input into base view
// basic algorithms on reverse view
///////////////////////////////////////////////////////////////////////////

struct mv_102_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

size_t mv_102( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(y);

  rev_vec_int_vw_tp a_rev_vw = rev_vec_int_vw_tp(a_vw);
  rev_vec_int_vw_tp b_rev_vw = rev_vec_int_vw_tp(b_vw);
  rev_vec_int_vw_tp x_rev_vw = rev_vec_int_vw_tp(x_vw);
  rev_vec_int_vw_tp y_rev_vw = rev_vec_int_vw_tp(y_vw);

  // initialize base views
  stapl::iota( a_vw, 0 );

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( b_vw, repeat_wf(base,rep) );

  // basic algorithms
  stapl::map_func( mv_102_elem_wf(), a_rev_vw, b_rev_vw, x_rev_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::scan( b_rev_vw, y_rev_vw, min_int_wf() );
#endif
  int map_red = stapl::map_reduce( abs_int_wf(), min_int_wf(), a_rev_vw );

  size_t cksum = stapl::reduce( a_rev_vw, xor_un_wf() );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_rev_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_rev_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV102","STAPL",time1);

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// vector : vector view : filter view
// generated input into base view
// basic algorithms on filter view
///////////////////////////////////////////////////////////////////////////

struct mv_103_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

struct mv_103_fill_wf {
  typedef void result_type;
  template <typename Elem, typename Count>
  result_type operator()(Elem elem, Count ndx)
  {
    elem = prime_nums[ rand_nums[ndx%data_cnt] ];
  }
};

size_t mv_103( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(y);

  filter1<int> filter_wf;
  filt_vec_int_vw_tp a_flt_vw = filt_vec_int_vw_tp (a_vw, filter_wf);
  filt_vec_int_vw_tp b_flt_vw = filt_vec_int_vw_tp (b_vw, filter_wf);
  filt_vec_int_vw_tp x_flt_vw = filt_vec_int_vw_tp (x_vw, filter_wf);
  filt_vec_int_vw_tp y_flt_vw = filt_vec_int_vw_tp (y_vw, filter_wf);

  // initialize base views
  int base = 10;
  int step = 4;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step) );

  stapl::map_func( mv_103_fill_wf(), b_vw,
                   stapl::counting_view<int>(a_vw.size()) );

  // basic algorithms
  stapl::map_func( mv_103_elem_wf(), a_flt_vw, b_flt_vw, x_flt_vw );
#ifdef BUG_SCAN_COMPOSE
 stapl::scan( b_flt_vw, y_flt_vw, add_int_wf() );
#endif
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), a_flt_vw );

  size_t cksum = stapl::reduce( a_flt_vw, xor_un_wf() );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_flt_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_flt_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV103","STAPL",time1);

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// vector : vector view : segmented view
// generated input into base view
// basic algorithms on segmented view
///////////////////////////////////////////////////////////////////////////

struct mv_104_fill_wf {
  typedef void result_type;
  template <typename Elem, typename Count>
  result_type operator()(Elem elem, Count ndx)
  {
    elem = prime_nums[ rand_nums[ndx%data_cnt] ];
  }
};

struct mv_104_seg_elem_wf {
  typedef void result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 x, View2 y, View3 z)
  {
    stapl::transform( x, y, z, mul_int_wf() );
  }
};

struct mv_104_seg_scan_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 x, View2 y)
  {
    stapl::scan( min_int_wf(), x, y );
  }
};

struct mv_104_seg_add_red_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 x)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), x );
  }
};

struct mv_104_seg_xor_red_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 x)
  {
    return stapl::map_reduce( id_un_wf(), xor_un_wf(), x );
  }
};

size_t mv_104( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  size_t size= 1000 * model;
  size_t limit = 0;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(y);

  // initialize base views
  stapl::generate( a_vw, stapl::random_sequence() );

  stapl::map_func( mv_104_fill_wf(), b_vw,
                   stapl::counting_view<int>(b_vw.size()) );


  vec_int_tp key(limit), fac(limit), rpt(limit);
  vec_int_vw_tp key_vw(key), fac_vw(fac), rpt_vw(rpt);

  stapl::serial_io(get_triple_wf(zin), key_vw, fac_vw, rpt_vw );

  vec_int_tp seg_flag(size);
  vec_int_vw_tp seg_flag_vw(seg_flag);
  int count = stapl::do_once( seg_flags_wf(), key_vw, seg_flag_vw );

  vec_int_tp seg_cnt(count);
  vec_int_vw_tp seg_cnt_vw(seg_cnt);
  count = stapl::do_once( seg_counts_wf(), seg_flag_vw, seg_cnt_vw );

  std::vector<size_t> stdq(seg_cnt.size()-1);
  for ( size_t i= 0; i<seg_cnt_vw.size(); i++ ) {
    stdq[i] = seg_cnt[i];
  }
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());

  seg_vec_vw_tp fac_seg_vw( fac_vw,
                 seg_vec_splitter(fac_vw.domain(), stdq,true));
  seg_vec_vw_tp rpt_seg_vw( rpt_vw,
                 seg_vec_splitter(rpt_vw.domain(), stdq,true));

  seg_vec_vw_tp x_seg_vw( x_vw,
                 seg_vec_splitter(fac_vw.domain(), stdq,true));
  seg_vec_vw_tp y_seg_vw( y_vw,
                 seg_vec_splitter(rpt_vw.domain(), stdq,true));

  // basic algorithms
  stapl::map_func( mv_104_seg_elem_wf(), fac_seg_vw, rpt_seg_vw, x_seg_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::map_func( mv_104_seg_scan_wf(), fac_seg_vw, y_seg_vw );
#endif
  int map_red = stapl::map_reduce( mv_104_seg_add_red_wf(), add_int_wf(),
                                   fac_seg_vw );

  size_t cksum = stapl::map_reduce( mv_104_seg_xor_red_wf(),
                                    xor_un_wf(), fac_seg_vw );

  // output

  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV104","STAPL",time1);

}

///////////////////////////////////////////////////////////////////////////
// vector : vector view : overlap view
// generated input into base view
// basic algorithms on overlap view
///////////////////////////////////////////////////////////////////////////

struct mv_105_ovl_elem_wf {
  typedef void result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 x, View2 y, View3 z)
  {
    stapl::transform( x, y, z, mul_int_wf() );
  }
};

struct mv_105_ovl_scan_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 x, View2 y)
  {
    stapl::scan( max_int_wf(), x, y );
  }
};

struct mv_105_ovl_add_red_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 x)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), x );
  }
};

struct mv_105_ovl_xor_red_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 x)
  {
    return stapl::map_reduce( id_un_wf(), xor_un_wf(), x );
  }
};

size_t mv_105( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(y);

  int core = 2;
  int left = 1;
  int right = 1;
  ovl_vec_int_vw_tp a_ovl_vw = make_overlap_view(a_vw,core,left,right);
  ovl_vec_int_vw_tp b_ovl_vw = make_overlap_view(b_vw,core,left,right);
  ovl_vec_int_vw_tp x_ovl_vw = make_overlap_view(x_vw,core,left,right);
  ovl_vec_int_vw_tp y_ovl_vw = make_overlap_view(y_vw,core,left,right);

  // initialize base views
  stapl::generate( a_vw, stapl::random_sequence() );

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( b_vw, repeat_wf(base,rep) );

  // basic algorithms
  stapl::map_func( mv_105_ovl_elem_wf(), a_ovl_vw, b_ovl_vw, x_ovl_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::map_func( mv_105_ovl_scan_wf(), a_ovl_vw, y_ovl_vw );
#endif

  int map_red = stapl::map_reduce( mv_105_ovl_add_red_wf(),
                                   add_int_wf(), a_ovl_vw);

  size_t cksum = stapl::map_reduce( mv_105_ovl_xor_red_wf(),
                                    xor_un_wf(), a_ovl_vw );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV105","STAPL",time1);

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// vector : vector view : strided view
// file input base view
// basic algorithms on strided view
///////////////////////////////////////////////////////////////////////////

struct mv_106_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

size_t mv_106( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  open_zin(model,106,zin);

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(y);

  int stride = 2;
  str_vec_int_vw_tp a_str_vw = stapl::make_strided_view(a_vw, stride, 0);
  str_vec_int_vw_tp b_str_vw = stapl::make_strided_view(b_vw, stride, 0);
  str_vec_int_vw_tp x_str_vw = stapl::make_strided_view(x_vw, stride, 0);
  str_vec_int_vw_tp y_str_vw = stapl::make_strided_view(y_vw, stride, 0);

  // input base views
  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::serial_io(get_val_wf(zin), b_vw);

  // basic algorithms
  stapl::map_func( mv_106_elem_wf(), a_str_vw, b_str_vw, x_str_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::scan( b_str_vw, y_str_vw, add_int_wf() );
#endif
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), a_str_vw );

  size_t cksum = stapl::reduce( a_str_vw, xor_un_wf() );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_str_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_str_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV106","STAPL",time1);
  zin.close();

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// vector : vector view : reverse view
// file input base view
// basic algorithms on reverse view
///////////////////////////////////////////////////////////////////////////

struct mv_107_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

size_t mv_107( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  open_zin(model,107,zin);

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(x);

  rev_vec_int_vw_tp a_rev_vw = rev_vec_int_vw_tp(a_vw);
  rev_vec_int_vw_tp b_rev_vw = rev_vec_int_vw_tp(b_vw);
  rev_vec_int_vw_tp x_rev_vw = rev_vec_int_vw_tp(x_vw);
  rev_vec_int_vw_tp y_rev_vw = rev_vec_int_vw_tp(y_vw);

  // input base views
  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::serial_io(get_val_wf(zin), b_vw);

  // basic algorithms
  stapl::map_func( mv_107_elem_wf(), a_rev_vw, b_rev_vw, x_rev_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::scan( b_rev_vw, y_rev_vw, min_int_wf() );
#endif
  int map_red = stapl::map_reduce( abs_int_wf(), min_int_wf(), a_rev_vw );

  size_t cksum = stapl::reduce( a_rev_vw, xor_un_wf() );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_rev_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_rev_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV107","STAPL",time1);
  zin.close();

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// vector : vector view : filter view
// file input base view
// basic algorithms on filter view
///////////////////////////////////////////////////////////////////////////

struct mv_108_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

struct mv_108_fill_wf {
  typedef void result_type;
  template <typename Elem, typename Count>
  result_type operator()(Elem elem, Count ndx)
  {
    elem = prime_nums[ rand_nums[ndx%data_cnt] ];
  }
};

size_t mv_108( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  open_zin(model,108,zin);

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(y);

  filter1<int> filter_wf;
  filt_vec_int_vw_tp a_flt_vw = filt_vec_int_vw_tp (a_vw, filter_wf);
  filt_vec_int_vw_tp b_flt_vw = filt_vec_int_vw_tp (b_vw, filter_wf);
  filt_vec_int_vw_tp x_flt_vw = filt_vec_int_vw_tp (x_vw, filter_wf);
  filt_vec_int_vw_tp y_flt_vw = filt_vec_int_vw_tp (y_vw, filter_wf);

  // input base views
  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::serial_io(get_val_wf(zin), b_vw);

  // basic algorithms
  stapl::map_func( mv_108_elem_wf(), a_flt_vw, b_flt_vw, x_flt_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::scan( b_flt_vw, y_flt_vw, add_int_wf() );
#endif
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), a_flt_vw );

  size_t cksum = stapl::reduce( a_flt_vw, xor_un_wf() );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_flt_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_flt_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV108","STAPL",time1);
  zin.close();

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// vector : vector view : segmented view
// file input into base view
// basic algorithms on segmented view
///////////////////////////////////////////////////////////////////////////

struct mv_109_fill_wf {
  typedef void result_type;
  template <typename Elem, typename Count>
  result_type operator()(Elem elem, Count ndx)
  {
    elem = prime_nums[ rand_nums[ndx%data_cnt] ];
  }
};

struct mv_109_seg_elem_wf {
  typedef void result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 x, View2 y, View3 z)
  {
    stapl::transform( x, y, z, mul_int_wf() );
  }
};

struct mv_109_seg_scan_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 x, View2 y)
  {
    stapl::scan( min_int_wf(), x, y );
  }
};

struct mv_109_seg_add_red_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 x)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), x );
  }
};

struct mv_109_seg_xor_red_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 x)
  {
    return stapl::map_reduce( id_un_wf(), xor_un_wf(), x );
  }
};

size_t mv_109( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  size_t size= 1000 * model;
  size_t limit = 0;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp x(size), y(size);
  vec_int_vw_tp x_vw(x), y_vw(y);

  vec_int_tp key(limit), fac(limit), rpt(limit);
  vec_int_vw_tp key_vw(key), fac_vw(fac), rpt_vw(rpt);

  // input base views
  stapl::serial_io(get_triple_wf(zin), key_vw, fac_vw, rpt_vw );

  vec_int_tp seg_flag(size);
  vec_int_vw_tp seg_flag_vw(seg_flag);
  int count = stapl::do_once( seg_flags_wf(), key_vw, seg_flag_vw );

  vec_int_tp seg_cnt(count);
  vec_int_vw_tp seg_cnt_vw(seg_cnt);
  count = stapl::do_once( seg_counts_wf(), seg_flag_vw, seg_cnt_vw );

  std::vector<size_t> stdq(seg_cnt.size()-1);
  for ( size_t i= 0; i<seg_cnt_vw.size(); i++ ) {
    stdq[i] = seg_cnt[i];
  }
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());

  seg_vec_vw_tp fac_seg_vw( fac_vw,
                 seg_vec_splitter(fac_vw.domain(), stdq,true));
  seg_vec_vw_tp rpt_seg_vw( rpt_vw,
                 seg_vec_splitter(rpt_vw.domain(), stdq,true));

  seg_vec_vw_tp x_seg_vw( x_vw,
                 seg_vec_splitter(fac_vw.domain(), stdq,true));
  seg_vec_vw_tp y_seg_vw( y_vw,
                 seg_vec_splitter(rpt_vw.domain(), stdq,true));

  // basic algorithms
  stapl::map_func( mv_109_seg_elem_wf(), fac_seg_vw, rpt_seg_vw, x_seg_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::map_func( mv_109_seg_scan_wf(), fac_seg_vw, y_seg_vw );
#endif
  int map_red = stapl::map_reduce( mv_109_seg_add_red_wf(), add_int_wf(),
                                   fac_seg_vw );

  size_t cksum = stapl::map_reduce( mv_109_seg_xor_red_wf(),
                                    xor_un_wf(), fac_seg_vw );

  // output

  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV109","STAPL",time1);
}


///////////////////////////////////////////////////////////////////////////
// vector : vector view : overlap view
// file input into base view
// basic algorithms on overlap view
///////////////////////////////////////////////////////////////////////////

struct mv_110_ovl_elem_wf {
  typedef void result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 x, View2 y, View3 z)
  {
    stapl::transform( x, y, z, mul_int_wf() );
  }
};

struct mv_110_ovl_scan_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 x, View2 y)
  {
    stapl::scan( max_int_wf(), x, y );
  }
};

struct mv_110_ovl_add_red_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 x)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), x );
  }
};

struct mv_110_ovl_xor_red_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 x)
  {
    return stapl::map_reduce( id_un_wf(), xor_un_wf(), x );
  }
};

size_t mv_110( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(y);

  int core = 2;
  int left = 1;
  int right = 1;
  ovl_vec_int_vw_tp a_ovl_vw = make_overlap_view(a_vw,core,left,right);
  ovl_vec_int_vw_tp b_ovl_vw = make_overlap_view(b_vw,core,left,right);
  ovl_vec_int_vw_tp x_ovl_vw = make_overlap_view(x_vw,core,left,right);
  ovl_vec_int_vw_tp y_ovl_vw = make_overlap_view(y_vw,core,left,right);

  // input base views
  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::serial_io(get_val_wf(zin), b_vw);

  // basic algorithms
  stapl::map_func( mv_110_ovl_elem_wf(), a_ovl_vw, b_ovl_vw, x_ovl_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::map_func( mv_110_ovl_scan_wf(), a_ovl_vw, y_ovl_vw );
#endif

  int map_red = stapl::map_reduce( mv_110_ovl_add_red_wf(),
                                   add_int_wf(), a_ovl_vw);

  size_t cksum = stapl::map_reduce( mv_110_ovl_xor_red_wf(),
                                    xor_un_wf(), a_ovl_vw );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV110","STAPL",time1);
}

///////////////////////////////////////////////////////////////////////////
// vector : vector view : strided view
// generated input into derived view
// basic algorithms on strided view
///////////////////////////////////////////////////////////////////////////

struct mv_111_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

size_t mv_111( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(y);

  int stride = 2;
  str_vec_int_vw_tp a_str_vw = stapl::make_strided_view(a_vw, stride, 0);
  str_vec_int_vw_tp b_str_vw = stapl::make_strided_view(b_vw, stride, 0);
  str_vec_int_vw_tp x_str_vw = stapl::make_strided_view(x_vw, stride, 0);
  str_vec_int_vw_tp y_str_vw = stapl::make_strided_view(y_vw, stride, 0);

  // initialize base views
  int base = 10;
  int step = 4;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_str_vw, step_wf(base,step) );

  stapl::iota( b_str_vw, 0 );

  // basic algorithms
  stapl::map_func( mv_111_elem_wf(), a_str_vw, b_str_vw, x_str_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::scan( b_str_vw, y_str_vw, add_int_wf() );
#endif
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), a_str_vw );

  size_t cksum = stapl::reduce( a_str_vw, xor_un_wf() );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_str_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_str_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV111","STAPL",time1);

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// vector : vector view : reverse view
// generated input into derived view
// basic algorithms on reverse view
///////////////////////////////////////////////////////////////////////////

struct mv_112_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

size_t mv_112( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(y);

  rev_vec_int_vw_tp a_rev_vw = rev_vec_int_vw_tp(a_vw);
  rev_vec_int_vw_tp b_rev_vw = rev_vec_int_vw_tp(b_vw);
  rev_vec_int_vw_tp x_rev_vw = rev_vec_int_vw_tp(x_vw);
  rev_vec_int_vw_tp y_rev_vw = rev_vec_int_vw_tp(y_vw);

  // initialize base views
  stapl::iota( a_rev_vw, 0 );

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( b_rev_vw, repeat_wf(base,rep) );

  // basic algorithms
  stapl::map_func( mv_112_elem_wf(), a_rev_vw, b_rev_vw, x_rev_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::scan( b_rev_vw, y_rev_vw, min_int_wf() );
#endif
  int map_red = stapl::map_reduce( abs_int_wf(), min_int_wf(), a_rev_vw );

  size_t cksum = stapl::reduce( a_rev_vw, xor_un_wf() );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_rev_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_rev_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV112","STAPL",time1);

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// vector : vector view : filter view
// generated input into derived view
// basic algorithms on filter view
///////////////////////////////////////////////////////////////////////////

struct mv_113_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

struct mv_113_fill_wf {
  typedef void result_type;
  template <typename Elem, typename Count>
  result_type operator()(Elem elem, Count ndx)
  {
    elem = prime_nums[ rand_nums[ndx%data_cnt] ];
  }
};

size_t mv_113( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(y);

  filter1<int> filter_wf;
  filt_vec_int_vw_tp a_flt_vw = filt_vec_int_vw_tp (a_vw, filter_wf);
  filt_vec_int_vw_tp b_flt_vw = filt_vec_int_vw_tp (b_vw, filter_wf);
  filt_vec_int_vw_tp x_flt_vw = filt_vec_int_vw_tp (x_vw, filter_wf);
  filt_vec_int_vw_tp y_flt_vw = filt_vec_int_vw_tp (y_vw, filter_wf);

  // initialize base views
  int base = 10;
  int step = 4;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_flt_vw, step_wf(base,step) );

  stapl::map_func( mv_113_fill_wf(), b_flt_vw,
                   stapl::counting_view<int>(b_vw.size()) );

  // basic algorithms
  stapl::map_func( mv_113_elem_wf(), a_flt_vw, b_flt_vw, x_flt_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::scan( b_flt_vw, y_flt_vw, add_int_wf() );
#endif
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), a_flt_vw );

  size_t cksum = stapl::reduce( a_flt_vw, xor_un_wf() );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_flt_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_flt_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV113","STAPL",time1);

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// THIS IS A PLACEHOLDER
// vector : vector view : segmented view
// generated input into segmented view has no meaning:
// how to input into partitioned view when the partitions
// are driven by data in the file?
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
// THIS IS A PLACEHOLDER
// vector : vector view : overlap view
// generated input into overlap view has no meaning
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
// vector : vector view : strided view
// file input into derived view
// basic algorithms on strided view
///////////////////////////////////////////////////////////////////////////

struct mv_114_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

size_t mv_114( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  open_zin(model,114,zin);

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(y);

  int stride = 2;
  str_vec_int_vw_tp a_str_vw = stapl::make_strided_view(a_vw, stride, 0);
  str_vec_int_vw_tp b_str_vw = stapl::make_strided_view(b_vw, stride, 0);
  str_vec_int_vw_tp x_str_vw = stapl::make_strided_view(x_vw, stride, 0);
  str_vec_int_vw_tp y_str_vw = stapl::make_strided_view(y_vw, stride, 0);

  // input base views
  stapl::serial_io(get_val_wf(zin), a_str_vw);
  stapl::serial_io(get_val_wf(zin), b_str_vw);

  // basic algorithms
  stapl::map_func( mv_114_elem_wf(), a_str_vw, b_str_vw, x_str_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::scan( b_str_vw, y_str_vw, add_int_wf() );
#endif
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), a_str_vw );

  size_t cksum = stapl::reduce( a_str_vw, xor_un_wf() );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_str_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_str_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV114","STAPL",time1);
  zin.close();

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// vector : vector view : reverse view
// file input into derived view
// basic algorithms on reverse view
///////////////////////////////////////////////////////////////////////////

struct mv_115_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

size_t mv_115( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  open_zin(model,115,zin);

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(x);

  rev_vec_int_vw_tp a_rev_vw = rev_vec_int_vw_tp(a_vw);
  rev_vec_int_vw_tp b_rev_vw = rev_vec_int_vw_tp(b_vw);
  rev_vec_int_vw_tp x_rev_vw = rev_vec_int_vw_tp(x_vw);
  rev_vec_int_vw_tp y_rev_vw = rev_vec_int_vw_tp(y_vw);

  // input base views
  stapl::serial_io(get_val_wf(zin), a_rev_vw);
  stapl::serial_io(get_val_wf(zin), b_rev_vw);

  // basic algorithms
  stapl::map_func( mv_115_elem_wf(), a_rev_vw, b_rev_vw, x_rev_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::scan( b_rev_vw, y_rev_vw, min_int_wf() );
#endif
  int map_red = stapl::map_reduce( abs_int_wf(), min_int_wf(), a_rev_vw );

  size_t cksum = stapl::reduce( a_rev_vw, xor_un_wf() );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_rev_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_rev_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV115","STAPL",time1);
  zin.close();

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// vector : vector view : filter view
// file input into derived view
// basic algorithms on filter view
///////////////////////////////////////////////////////////////////////////

struct mv_116_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

struct mv_116_fill_wf {
  typedef void result_type;
  template <typename Elem, typename Count>
  result_type operator()(Elem elem, Count ndx)
  {
    elem = prime_nums[ rand_nums[ndx%data_cnt] ];
  }
};

size_t mv_116( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  open_zin(model,116,zin);

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), x(size), y(size);
  vec_int_vw_tp a_vw(a), b_vw(b), x_vw(x), y_vw(y);

  filter1<int> filter_wf;
  filt_vec_int_vw_tp a_flt_vw = filt_vec_int_vw_tp (a_vw, filter_wf);
  filt_vec_int_vw_tp b_flt_vw = filt_vec_int_vw_tp (b_vw, filter_wf);
  filt_vec_int_vw_tp x_flt_vw = filt_vec_int_vw_tp (x_vw, filter_wf);
  filt_vec_int_vw_tp y_flt_vw = filt_vec_int_vw_tp (y_vw, filter_wf);

  // input base views
  stapl::serial_io(get_val_wf(zin), a_flt_vw);
  stapl::serial_io(get_val_wf(zin), b_flt_vw);

  // basic algorithms
  stapl::map_func( mv_116_elem_wf(), a_flt_vw, b_flt_vw, x_flt_vw );
#ifdef BUG_SCAN_COMPOSE
  stapl::scan( b_flt_vw, y_flt_vw, add_int_wf() );
#endif
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), a_flt_vw );

  size_t cksum = stapl::reduce( a_flt_vw, xor_un_wf() );

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), x_flt_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), y_flt_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV116","STAPL",time1);
  zin.close();

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// THIS IS A PLACEHOLDER
// vector : vector view : segmented view
// file input into segmented view has no meaning:
// how to input into partitioned view when the partitions
// are driven by data in the file?
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
// THIS IS A PLACEHOLDER
// vector : vector view : overlap view
// file input into overlap view has no meaning
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
// array : array_view : zip_view
// generated input into base view
// basic algorithms on zip view
///////////////////////////////////////////////////////////////////////////


struct mv_117_triple_sum_wf {
  typedef void result_type;
  template <typename Left, typename Result>
  result_type operator()(Left left, Result result)
  {
    tuple<int,int,int> tup = left;
    result = get<0>(tup) + get<1>(tup) + get<2>(tup);
  }
};

struct mv_117_triple_max_wf {
  typedef int result_type;
  template <typename Arg>
  result_type operator()(Arg arg)
  {
    tuple<int,int,int> tup = arg;
    return std::max(std::max(get<0>(tup), get<1>(tup)), get<2>(tup));
  }
};

size_t mv_117( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_int_tp a(size), b(size), c(size), d(size);
  ary_int_vw_tp a_vw(a), b_vw(b), c_vw(c), d_vw(d);

  zip3_int_vw_tp p_zip_vw( a_vw, b_vw, c_vw );
  zip3_int_vw_tp q_zip_vw( b_vw, c_vw, d_vw );

  ary_int_tp x(size), y(size), z(size);
  ary_int_vw_tp x_vw(x), y_vw(y), z_vw(z);

  // initialize base views
  int base = 10;
  int step = 4;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step) );

  stapl::iota( b_vw, 0 );

  stapl::generate( c_vw, stapl::random_sequence() );

  int rep = 42;
  base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( d_vw, repeat_wf(base,rep) );

  // basic algorithms
  stapl::map_func( mv_117_triple_sum_wf(), p_zip_vw, x_vw );

#ifdef BUG_SCAN_COMPOSE
  stapl::scan( b_str_vw, y_str_vw, add_int_wf() );
#endif
  int map_red = stapl::map_reduce( mv_117_triple_max_wf(), add_int_wf(),
                                   p_zip_vw );

  size_t cksum = 0;

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_triple_wf(zout), p_zip_vw,
                  stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_triple_wf(zout), q_zip_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV117","STAPL",time1);

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// array : array_view : zip_view
// file input into base view
// basic algorithms on zip view
///////////////////////////////////////////////////////////////////////////

struct mv_118_triple_sum_wf {
  typedef void result_type;
  template <typename Left, typename Result>
  result_type operator()(Left left, Result result)
  {
    tuple<int,int,int> tup = left;
    result = get<0>(tup) + get<1>(tup) + get<2>(tup);
  }
};

struct mv_118_triple_max_wf {
  typedef int result_type;
  template <typename Arg>
  result_type operator()(Arg arg)
  {
    tuple<int,int,int> tup = arg;
    return std::max(std::max(get<0>(tup), get<1>(tup)), get<2>(tup));
  }
};

size_t mv_118( size_t model,
             stapl::stream<ifstream>& zin,
             stapl::stream<ofstream>& zout ) {

  open_zin(model,118,zin);

  size_t size= 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_int_tp a(size), b(size), c(size);
  ary_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  zip3_int_vw_tp p_zip_vw( a_vw, b_vw, c_vw );
  zip3_int_vw_tp q_zip_vw( c_vw, b_vw, a_vw );

  ary_int_tp x(size), y(size), z(size);
  ary_int_vw_tp x_vw(x), y_vw(y), z_vw(z);

  // input base views
  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::serial_io(get_val_wf(zin), b_vw);
  stapl::serial_io(get_val_wf(zin), c_vw);

  // basic algorithms
  stapl::map_func( mv_118_triple_sum_wf(), p_zip_vw, x_vw );

#ifdef BUG_SCAN_COMPOSE
  stapl::scan( b_str_vw, y_str_vw, add_int_wf() );
#endif
  int map_red = stapl::map_reduce( mv_118_triple_max_wf(),
                                   add_int_wf(), p_zip_vw );

  size_t cksum = 0;

  // output
  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_triple_wf(zout), p_zip_vw,
                  stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_triple_wf(zout), q_zip_vw,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV118","STAPL",time1);

  return cksum;
}

/*=========================================================================*/

///////////////////////////////////////////////////////////////////////////
// list_view over array
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

struct mv_121_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

void mv_121_build( size_t model, ary_int_vw_tp a_vw,
                   ary_int_vw_tp b_vw, ary_int_vw_tp c_vw ) {

  int size = model * 1000;

  int init = 10;
  int step = 4;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(init,step) );

  stapl::iota( b_vw, 0 );

  stapl::generate( c_vw, stapl::random_sequence() );
}

size_t mv_121( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_int_tp a(size), b(size), c(size);
  ary_int_vw_tp a_vw1(a), b_vw1(b), c_vw1(c);

  mv_121_build( model, a, b, c );

  list_ary_int_vw_tp a_vw2 = list_ary_int_vw_tp(a_vw1);
  list_ary_int_vw_tp b_vw2 = list_ary_int_vw_tp(b_vw1);
  list_ary_int_vw_tp c_vw2 = list_ary_int_vw_tp(c_vw1);

// change "_vw1" to "_vw2" in these 4 statements to see the bugs
#ifdef BUG_ELEM_LIST
  stapl::map_func( mv_121_elem_wf(), a_vw1, b_vw1, c_vw1 );
#endif
#ifdef BUG_SCAN_LIST
  stapl::scan( b_vw1, c_vw1, add_int_wf() );
#endif
#ifdef BUG_RED_LIST
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), c_vw1 );
  size_t cksum = stapl::reduce( a_vw1, xor_un_wf() );
#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV121","STAPL",time1);

#ifdef CLEANUP
  return cksum;
#endif
}

///////////////////////////////////////////////////////////////////////////
// list_view over vector
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

struct mv_122_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

void mv_122_build( size_t model, vec_int_vw_tp a_vw,
                   vec_int_vw_tp b_vw, vec_int_vw_tp c_vw ) {

  int size = model * 1000;

  stapl::iota( a_vw, 0 );

  stapl::generate( b_vw, stapl::random_sequence() );

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( c_vw, repeat_wf(base,rep) );

}

size_t mv_122( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), c(size);
  vec_int_vw_tp a_vw1(a), b_vw1(b), c_vw1(c);

  mv_122_build( model, a, b, c );

  list_vec_int_vw_tp a_vw2 = list_vec_int_vw_tp(a_vw1);
  list_vec_int_vw_tp b_vw2 = list_vec_int_vw_tp(b_vw1);
  list_vec_int_vw_tp c_vw2 = list_vec_int_vw_tp(c_vw1);

// change "_vw1" to "_vw2" in these 4 statements to see the bugs
#ifdef BUG_ELEM_LIST
  stapl::map_func( mv_122_elem_wf(), a_vw1, b_vw1, c_vw1 );
#endif
#ifdef BUG_SCAN_LIST
  stapl::scan( b_vw1, c_vw1, add_int_wf() );
#endif
#ifdef BUG_RED_LIST
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), c_vw1 );
  size_t cksum = stapl::reduce( a_vw1, xor_un_wf() );
#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV122","STAPL",time1);

#ifdef CLEANUP
  return cksum;
#endif
}

///////////////////////////////////////////////////////////////////////////
// list_view over map
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

struct mv_123_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result, typename Target>
  result_type operator()(Left left_pair, Right right_pair,
                         Result result_pair, Target target_map)
  {
    typename Left::second_reference left = left_pair.second;
    typename Right::second_reference right = right_pair.second;
    target_map.insert( result_pair.first, left + right );
  }
};

class mv_123_build_wf {
private:
  size_t m_size;
  size_t m_offset;
public:
  mv_123_build_wf(size_t s, size_t o)
    : m_size(s), m_offset(o)
  { }
  typedef void result_type;
  template<typename Data>
  result_type operator()(Data &a) {
    if ( m_offset == 0 ) {
      for ( size_t i = 0; i < m_size; i++ ) {
        a[ prime_nums[i] ] = rand_nums[i];
      }
    } else {
      for ( size_t i = m_offset; i < m_size; i++ ) {
        a[ prime_nums[i-m_offset] ] = rand_nums[i];
      }
      for ( size_t i = 0; i < m_offset; i++ ) {
        a[ prime_nums[i+m_offset] ] = rand_nums[i];
      }
    }
  }
  void define_type(stapl::typer& t) {
    t.member(m_size);
    t.member(m_offset);
  }
};

size_t mv_123( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ndx_dom_tp dom(0, size-1);

  map_int_tp a(dom), b(dom), c(dom), d(dom);
  map_int_vw_tp a_vw1(a), b_vw1(b), c_vw1(c), d_vw1(d);

  stapl::do_once( mv_123_build_wf(size,fibo20[0]), a_vw1 );
  stapl::do_once( mv_123_build_wf(size,fibo20[2]), b_vw1 );
  stapl::do_once( mv_123_build_wf(size,fibo20[4]), c_vw1 );

  list_map_int_vw_tp a_vw2 = list_map_int_vw_tp(a_vw1);
  list_map_int_vw_tp b_vw2 = list_map_int_vw_tp(b_vw1);
  list_map_int_vw_tp c_vw2 = list_map_int_vw_tp(c_vw1);
  list_map_int_vw_tp d_vw2 = list_map_int_vw_tp(d_vw1);

// change "_vw1" to "_vw2" in these 4 statements to see the bugs
#ifdef BUG_ELEM_LIST
  stapl::map_func( mv_123_elem_wf(), a_vw1, b_vw1, c_vw1,
                   stapl::make_repeat_view(c_vw1) );
#endif
#ifdef BUG_SCAN_LIST
  // need to change work function to use pair.first,second
  //stapl::scan( b_vw1, d_vw1, add_int_wf() );
#endif
#ifdef BUG_RED_LIST
  int map_red = stapl::map_reduce( inner_map_elem_wf(), max_int_wf(), c_vw1 );
  size_t cksum = stapl::map_reduce( inner_map_elem_wf(), xor_un_wf(), a_vw1 );
#endif

  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_map_wf(zout), c_vw1,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_map_wf(zout), d_vw1,
                    stapl::counting_view<int>(size) );

#ifdef CLEANUP
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );
#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV123","STAPL",time1);

#ifdef CLEANUP
  return cksum;
#endif

}

///////////////////////////////////////////////////////////////////////////
// array_view over list
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

struct mv_124_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

void mv_124_build( size_t model, list_int_vw_tp a_vw,
                   list_int_vw_tp b_vw, list_int_vw_tp c_vw ) {

  stapl::generate( a_vw, stapl::random_sequence() );

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( b_vw, repeat_wf(base,rep) );

  int init = 10;
  int step = 4;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(c_vw, step_wf(init,step) );
}

size_t mv_124( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  list_int_tp a, b, c;
  list_int_vw_tp a_vw1(a), b_vw1(b), c_vw1(c);

  mv_124_build( model, a, b, c );

  ary_list_int_vw_tp a_vw2 = ary_list_int_vw_tp(a_vw1);
  ary_list_int_vw_tp b_vw2 = ary_list_int_vw_tp(b_vw1);
  ary_list_int_vw_tp c_vw2 = ary_list_int_vw_tp(c_vw1);

// change "_vw1" to "_vw2" in these 4 statements to see the bugs
#ifdef BUG_ELEM_LIST
  stapl::map_func( mv_124_elem_wf(), a_vw1, b_vw1, c_vw1 );
#endif
#ifdef BUG_SCAN_LIST
  stapl::scan( b_vw1, c_vw1, add_int_wf() );
#endif
#ifdef BUG_RED_LIST
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), c_vw1 );
  size_t cksum = stapl::reduce( a_vw1, xor_un_wf() );
#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV124","STAPL",time1);

#ifdef CLEANUP
  return cksum;
#endif
}

///////////////////////////////////////////////////////////////////////////
// array_view over vector
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

struct mv_125_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

void mv_125_build( size_t model, vec_int_vw_tp a_vw,
                   vec_int_vw_tp b_vw, vec_int_vw_tp c_vw ) {

  int size = model * 1000;

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( a_vw, repeat_wf(base,rep) );

  int init = 10;
  int step = 4;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(b_vw, step_wf(init,step) );

  stapl::iota( c_vw, 0 );

}

size_t mv_125( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), c(size), d(size);
  vec_int_vw_tp a_vw1(a), b_vw1(b), c_vw1(c), d_vw1(d);

  mv_125_build( model, a, b, c );

  ary_vec_int_vw_tp a_vw2 = ary_vec_int_vw_tp(a_vw1);
  ary_vec_int_vw_tp b_vw2 = ary_vec_int_vw_tp(b_vw1);
  ary_vec_int_vw_tp c_vw2 = ary_vec_int_vw_tp(c_vw1);

  stapl::map_func( mv_125_elem_wf(), a_vw2, b_vw2, c_vw1 );

  stapl::scan( b_vw2, d_vw1, add_int_wf() );
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), c_vw2 );
  size_t cksum = stapl::reduce( a_vw2, xor_un_wf() );

  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), c_vw1,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), d_vw1,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV125","STAPL",time1);

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// array_view over map
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

struct mv_126_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result, typename Target>
  result_type operator()(Left left_pair, Right right_pair,
                         Result result_pair, Target target_map)
  {
    typename Left::second_reference left = left_pair.second;
    typename Right::second_reference right = right_pair.second;
    target_map.insert( result_pair.first, left + right );
  }
};

class mv_126_build_wf {
private:
  size_t m_size;
  size_t m_offset;
public:
  mv_126_build_wf(size_t s, size_t o)
    : m_size(s), m_offset(o)
  { }
  typedef void result_type;
  template<typename Data>
  result_type operator()(Data &a) {
    if ( m_offset == 0 ) {
      for ( size_t i = 0; i < m_size; i++ ) {
        a[ prime_nums[i] ] = rand_nums[i];
      }
    } else {
      for ( size_t i = m_offset; i < m_size; i++ ) {
        a[ prime_nums[i-m_offset] ] = rand_nums[i];
      }
      for ( size_t i = 0; i < m_offset; i++ ) {
        a[ prime_nums[i+m_offset] ] = rand_nums[i];
      }
    }
  }
  void define_type(stapl::typer& t) {
    t.member(m_size);
    t.member(m_offset);
  }
};

size_t mv_126( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ndx_dom_tp dom(0, size-1);

  map_int_tp a(dom), b(dom), c(dom), d(dom);
  map_int_vw_tp a_vw1(a), b_vw1(b), c_vw1(c), d_vw1(d);

  stapl::do_once( mv_126_build_wf(size,fibo20[1]), a_vw1 );
  stapl::do_once( mv_126_build_wf(size,fibo20[3]), b_vw1 );
  stapl::do_once( mv_126_build_wf(size,fibo20[5]), c_vw1 );

  ary_map_int_vw_tp a_vw2 = ary_map_int_vw_tp(a_vw1);
  ary_map_int_vw_tp b_vw2 = ary_map_int_vw_tp(b_vw1);
  ary_map_int_vw_tp c_vw2 = ary_map_int_vw_tp(c_vw1);
  ary_map_int_vw_tp d_vw2 = ary_map_int_vw_tp(d_vw1);

// change "_vw1" to "_vw2" in these 4 statements to see the bugs
#ifdef BUG_ELEM_LIST
  stapl::map_func( mv_126_elem_wf(), a_vw1, b_vw1, c_vw1,
                   stapl::make_repeat_view(c_vw1) );
#endif
#ifdef BUG_SCAN_LIST
  // need to change work function to use pair.first,second
  //stapl::scan( b_vw1, d_vw1, add_int_wf() );
#endif
#ifdef BUG_RED_LIST
  int map_red = stapl::map_reduce( inner_map_elem_wf(), max_int_wf(), c_vw1 );
  size_t cksum = stapl::map_reduce( inner_map_elem_wf(), xor_un_wf(), a_vw1 );
#endif

  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_map_wf(zout), c_vw1,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_map_wf(zout), d_vw1,
                    stapl::counting_view<int>(size) );

#ifdef CLEANUP
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );
#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV126","STAPL",time1);

#ifdef CLEANUP
  return cksum;
#endif
}

///////////////////////////////////////////////////////////////////////////
// vector_view over list
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

struct mv_127_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

void mv_127_build( size_t model, list_int_vw_tp a_vw,
                   list_int_vw_tp b_vw, list_int_vw_tp c_vw ) {

  int size = model * 1000;

  int init = 10;
  int step = 4;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(init,step) );

  stapl::iota( b_vw, 0 );

  stapl::generate( c_vw, stapl::random_sequence() );

}

size_t mv_127( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  list_int_tp a, b, c, d;
  list_int_vw_tp a_vw1(a), b_vw1(b), c_vw1(c), d_vw1(d);

  mv_127_build( model, a, b, c );

  vec_list_int_vw_tp a_vw2 = vec_list_int_vw_tp(a_vw1);
  vec_list_int_vw_tp b_vw2 = vec_list_int_vw_tp(b_vw1);
  vec_list_int_vw_tp c_vw2 = vec_list_int_vw_tp(c_vw1);
  vec_list_int_vw_tp d_vw2 = vec_list_int_vw_tp(c_vw1);

// change "_vw1" to "_vw2" in these 4 statements to see the bugs
#ifdef BUG_ELEM_LIST
  stapl::map_func( mv_127_elem_wf(), a_vw1, b_vw1, c_vw1 );
#endif
#ifdef BUG_SCAN_LIST
  stapl::scan( b_vw1, d_vw1, add_int_wf() );
#endif
#ifdef BUG_RED_LIST
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), c_vw1 );
  size_t cksum = stapl::reduce( a_vw1, xor_un_wf() );
#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV127","STAPL",time1);

#ifdef CLEANUP
  return cksum;
#endif
}

///////////////////////////////////////////////////////////////////////////
// vector_view over array
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

struct mv_128_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

void mv_128_build( size_t model, ary_int_vw_tp a_vw,
                    ary_int_vw_tp b_vw, ary_int_vw_tp c_vw ) {

  int size = model * 1000;

  stapl::iota( a_vw, 0 );

  stapl::generate( b_vw, stapl::random_sequence() );

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( c_vw, repeat_wf(base,rep) );

}

size_t mv_128( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_int_tp a(size), b(size), c(size), d(size);
  ary_int_vw_tp a_vw1(a), b_vw1(b), c_vw1(c), d_vw1(d);

  mv_128_build( model, a, b, c );

  vec_ary_int_vw_tp a_vw2 = vec_ary_int_vw_tp(a_vw1);
  vec_ary_int_vw_tp b_vw2 = vec_ary_int_vw_tp(b_vw1);
  vec_ary_int_vw_tp c_vw2 = vec_ary_int_vw_tp(c_vw1);
  vec_ary_int_vw_tp d_vw2 = vec_ary_int_vw_tp(d_vw1);

  stapl::map_func( mv_128_elem_wf(), a_vw2, b_vw2, c_vw1 );
  stapl::scan( b_vw2, c_vw1, add_int_wf() );
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), c_vw2 );
  size_t cksum = stapl::reduce( a_vw2, xor_un_wf() );

  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), c_vw1,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), d_vw1,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV128","STAPL",time1);

  return cksum;
}

///////////////////////////////////////////////////////////////////////////
// vector_view over map
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

struct mv_129_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result, typename Target>
  result_type operator()(Left left_pair, Right right_pair,
                         Result result_pair, Target target_map)
  {
    typename Left::second_reference left = left_pair.second;
    typename Right::second_reference right = right_pair.second;
    target_map.insert( result_pair.first, left + right );
  }
};

class mv_129_build_wf {
private:
  size_t m_size;
  size_t m_offset;
public:
  mv_129_build_wf(size_t s, size_t o)
    : m_size(s), m_offset(o)
  { }
  typedef void result_type;
  template<typename Data>
  result_type operator()(Data &a) {
    if ( m_offset == 0 ) {
      for ( size_t i = 0; i < m_size; i++ ) {
        a[ prime_nums[i] ] = rand_nums[i];
      }
    } else {
      for ( size_t i = m_offset; i < m_size; i++ ) {
        a[ prime_nums[i-m_offset] ] = rand_nums[i];
      }
      for ( size_t i = 0; i < m_offset; i++ ) {
        a[ prime_nums[i+m_offset] ] = rand_nums[i];
      }
    }
  }
  void define_type(stapl::typer& t) {
    t.member(m_size);
    t.member(m_offset);
  }
};

size_t mv_129( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ndx_dom_tp dom(0, size-1);

  map_int_tp a(dom), b(dom), c(dom), d(dom);
  map_int_vw_tp a_vw1(a), b_vw1(b), c_vw1(c), d_vw1(d);

  stapl::do_once( mv_129_build_wf(size,fibo20[6]), a_vw1 );
  stapl::do_once( mv_129_build_wf(size,fibo20[8]), b_vw1 );
  stapl::do_once( mv_129_build_wf(size,fibo20[10]), c_vw1 );

  vec_map_int_vw_tp a_vw2 = vec_map_int_vw_tp(a_vw1);
  vec_map_int_vw_tp b_vw2 = vec_map_int_vw_tp(b_vw1);
  vec_map_int_vw_tp c_vw2 = vec_map_int_vw_tp(c_vw1);
  vec_map_int_vw_tp d_vw2 = vec_map_int_vw_tp(d_vw1);

// change "_vw1" to "_vw2" in these 4 statements to see the bugs
#ifdef BUG_ELEM_LIST
  stapl::map_func( mv_129_elem_wf(), a_vw1, b_vw1, c_vw1,
                   stapl::make_repeat_view(c_vw1) );
#endif
#ifdef BUG_SCAN_LIST
  // need to change work function to use pair.first,second
  //stapl::scan( b_vw1, d_vw1, add_int_wf() );
#endif
#ifdef BUG_RED_LIST
  int map_red = stapl::map_reduce( inner_map_elem_wf(), max_int_wf(), c_vw1 );
  size_t cksum = stapl::map_reduce( inner_map_elem_wf(), xor_un_wf(), a_vw1 );
#endif

  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_map_wf(zout), c_vw1,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_map_wf(zout), d_vw1,
                    stapl::counting_view<int>(size) );

#ifdef CLEANUP
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );
#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV129","STAPL",time1);

#ifdef CLEANUP
  return cksum;
#endif
}

///////////////////////////////////////////////////////////////////////////
// array_view over array_view
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

struct mv_130_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

void mv_130_build( size_t model, ary_int_vw_tp a_vw,
                   ary_int_vw_tp b_vw, ary_int_vw_tp c_vw ) {

  int size = model * 1000;

  stapl::generate( a_vw, stapl::random_sequence() );

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( b_vw, repeat_wf(base,rep) );

  int init = 10;
  int step = 4;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(c_vw, step_wf(init,step) );

}

size_t mv_130( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_int_tp a(size), b(size), c(size), d(size);
  ary_int_vw_tp a_vw1(a), b_vw1(b), c_vw1(c), d_vw1(d);

  mv_130_build( model, a, b, c );

  ndx_dom_tp twice_dom(0, size*2);

  ary_ary_int_vw_tp a_vw2 = ary_ary_int_vw_tp(a_vw1, twice_dom);
  ary_ary_int_vw_tp b_vw2 = ary_ary_int_vw_tp(b_vw1, twice_dom);
  ary_ary_int_vw_tp c_vw2 = ary_ary_int_vw_tp(c_vw1, twice_dom);
  ary_ary_int_vw_tp d_vw2 = ary_ary_int_vw_tp(d_vw1, twice_dom);

  stapl::map_func( mv_130_elem_wf(), a_vw2, b_vw2, c_vw2 );
  stapl::scan( b_vw2, c_vw1, add_int_wf() );
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), c_vw2 );
  size_t cksum = stapl::reduce( a_vw2, xor_un_wf() );

  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), c_vw1,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), d_vw1,
                    stapl::counting_view<int>(size) );
#ifdef CLEANUP
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );
#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV130","STAPL",time1);

#ifdef CLEANUP
  return cksum;
#endif
}

///////////////////////////////////////////////////////////////////////////
// vector_view over vector
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

struct mv_131_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left left, Right right, Result result)
  {
    result = left + right;
  }
};

void mv_131_build( size_t model, vec_int_vw_tp a_vw,
                   vec_int_vw_tp b_vw, vec_int_vw_tp c_vw ) {

  int size = model * 1000;

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( a_vw, repeat_wf(base,rep) );

  int init = 10;
  int step = 4;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(b_vw, step_wf(init,step) );

  stapl::iota( c_vw, 0 );

}

size_t mv_131( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  vec_int_tp a(size), b(size), c(size), d(size);
  vec_int_vw_tp a_vw1(a), b_vw1(b), c_vw1(c), d_vw1(d);

  mv_131_build( model, a, b, c );

  ndx_dom_tp twice_dom(0, size*2);

  vec_vec_int_vw_tp a_vw2 = vec_vec_int_vw_tp(a_vw1, twice_dom);
  vec_vec_int_vw_tp b_vw2 = vec_vec_int_vw_tp(b_vw1, twice_dom);
  vec_vec_int_vw_tp c_vw2 = vec_vec_int_vw_tp(c_vw1, twice_dom);
  vec_vec_int_vw_tp d_vw2 = vec_vec_int_vw_tp(d_vw1, twice_dom);

  stapl::map_func( mv_131_elem_wf(), a_vw2, b_vw2, c_vw2 );
  stapl::scan( b_vw2, c_vw1, add_int_wf() );
  int map_red = stapl::map_reduce( neg_int_wf(), max_int_wf(), c_vw2 );
  size_t cksum = stapl::reduce( a_vw2, xor_un_wf() );

  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_ndx_wf(zout), c_vw1,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_ndx_wf(zout), d_vw1,
                    stapl::counting_view<int>(size) );
#ifdef CLEANUP
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );
#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV131","STAPL",time1);

#ifdef CLEANUP
  return cksum;
#endif
}

///////////////////////////////////////////////////////////////////////////
// map_view over map
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

struct mv_132_elem_wf {
  typedef void result_type;
  template <typename Left, typename Right, typename Result, typename Target>
  result_type operator()(Left left_pair, Right right_pair,
                         Result result_pair, Target target_map)
  {
    typename Left::second_reference left = left_pair.second;
    typename Right::second_reference right = right_pair.second;
    target_map.insert( result_pair.first, left + right );
  }
};

class mv_132_build_wf {
private:
  size_t m_size;
  size_t m_offset;
public:
  mv_132_build_wf(size_t s, size_t o)
    : m_size(s), m_offset(o)
  { }
  typedef void result_type;
  template<typename Data>
  result_type operator()(Data &a) {
    if ( m_offset == 0 ) {
      for ( size_t i = 0; i < m_size; i++ ) {
        a[ prime_nums[i] ] = rand_nums[i];
      }
    } else {
      for ( size_t i = m_offset; i < m_size; i++ ) {
        a[ prime_nums[i-m_offset] ] = rand_nums[i];
      }
      for ( size_t i = 0; i < m_offset; i++ ) {
        a[ prime_nums[i+m_offset] ] = rand_nums[i];
      }
    }
  }
  void define_type(stapl::typer& t) {
    t.member(m_size);
    t.member(m_offset);
  }
};

size_t mv_132( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ndx_dom_tp dom(0, size-1);

  map_int_tp a(dom), b(dom), c(dom), d(dom);
  map_int_vw_tp a_vw1(a), b_vw1(b), c_vw1(c), d_vw1(d);

  stapl::do_once( mv_132_build_wf(size,fibo20[7]), a_vw1 );
  stapl::do_once( mv_132_build_wf(size,fibo20[9]), b_vw1 );
  stapl::do_once( mv_132_build_wf(size,fibo20[11]), c_vw1 );

  ndx_dom_tp twice_dom(0, size*2);

#ifdef MAP_DOMAIN_BUG
  map_map_int_vw_tp a_vw2 = map_map_int_vw_tp(a_vw1, twice_dom);
  map_map_int_vw_tp b_vw2 = map_map_int_vw_tp(b_vw1, twice_dom);
  map_map_int_vw_tp c_vw2 = map_map_int_vw_tp(c_vw1, twice_dom);
  map_map_int_vw_tp d_vw2 = map_map_int_vw_tp(d_vw1, twice_dom);

  stapl::map_func( mv_132_elem_wf(), a_vw1, b_vw1, c_vw1,
                   stapl::make_repeat_view(c_vw1) );
  //stapl::scan( b_vw1, d_vw1, add_int_wf() );
  int map_red = stapl::map_reduce( inner_map_elem_wf(), max_int_wf(), c_vw1 );
  size_t cksum = stapl::map_reduce( inner_map_elem_wf(), xor_un_wf(), a_vw1 );
#endif

  stapl::do_once( msg( "MAP_FUNC" ) );
  stapl::serial_io( show_log_map_wf(zout), c_vw1,
                    stapl::counting_view<int>(size) );
  stapl::do_once( msg( "PRE_SCAN" ) );
  stapl::serial_io( show_log_map_wf(zout), d_vw1,
                    stapl::counting_view<int>(size) );

#ifdef CLEANUP
  stapl::do_once( msg_val<int>( "MAP_RED", map_red ) );
#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
  show_time("MV132","STAPL",time1);

#ifdef CLEANUP
  return cksum;
#endif
}
