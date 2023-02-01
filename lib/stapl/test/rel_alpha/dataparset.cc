/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cstdlib>
#include <cmath>
#include <string>
#include <stapl/runtime.hpp>
#include <iostream>

#include <stapl/set.hpp>
#include <stapl/views/set_view.hpp>
#include <stapl/views/counting_view.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/utility/do_once.hpp>

#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include "json_parser.hpp"

#include "testutil.hpp"
#include "rel_alpha_data.h"

#define MAP_FUNC 1
using namespace std;

//////////////////////////////////////////////////////////////////////

typedef stapl::indexed_domain<int>      ndx_dom_tp;

typedef stapl::set<int>                 set_int_tp;
typedef stapl::set_view<set_int_tp>     set_int_vw_tp;

//////////////////////////////////////////////////////////////////////

void init_set( set_int_vw_tp &, set_int_vw_tp &,
               set_int_vw_tp &, set_int_vw_tp &,
               set_int_vw_tp &, set_int_vw_tp &, int );

bool test_006( set_int_vw_tp &, set_int_vw_tp &,
               set_int_vw_tp &, set_int_vw_tp &,
               set_int_vw_tp &, set_int_vw_tp &, size_t );

bool test_007( set_int_vw_tp &, set_int_vw_tp &,
               set_int_vw_tp &, set_int_vw_tp &,
               set_int_vw_tp &, set_int_vw_tp &, size_t );

bool test_008( set_int_vw_tp &, set_int_vw_tp &,
               set_int_vw_tp &, set_int_vw_tp &,
               set_int_vw_tp &, set_int_vw_tp &, size_t );

bool test_009( set_int_vw_tp &, set_int_vw_tp &,
               set_int_vw_tp &, set_int_vw_tp &,
               set_int_vw_tp &, set_int_vw_tp &, size_t );

#ifdef COMPLEMENT
bool test_010( set_int_vw_tp &, set_int_vw_tp &,
               set_int_vw_tp &, set_int_vw_tp &,
               set_int_vw_tp &, set_int_vw_tp &, size_t );
#endif

//////////////////////////////////////////////////////////////////////

int dom_lmt = -1;

//////////////////////////////////////////////////////////////////////
// DATA PARALLEL TESTS - sets
//////////////////////////////////////////////////////////////////////

bool opt_list = false, opt_noref = false, opt_out = false, opt_quiet = false;
int opt_test = -1;
char *opt_data = 0;
bool opt_stapl = false;
bool opt_manual = false;

stapl::exit_code stapl_main(int argc, char **argv) {

  char *temp = 0;
  for ( int argi = 1; argi < argc; ) {
    char * opt = argv[argi++];
    if ( '-' == opt[0] ) {
      switch( opt[1] ) {
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
  if ( !opt_stapl && !opt_manual ) {
    // default behavior
    opt_stapl = true;
    opt_manual = true;
  }

  if ( opt_list ) {
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
    for ( int i = 1; i<50; i++ ) {
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
  switch( opt_data[0] ) {
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

  int first_test = 6;
  int last_test = 6;
  if ( opt_test != -1 ) {
    first_test = opt_test;
    last_test = opt_test;
  }

  size_t size = 1000 * model;
  dom_lmt = size * 1000;

  ndx_dom_tp dom(0, dom_lmt - 1);

  set_int_tp u(dom), v(dom), w(dom);
  set_int_tp x(dom), y(dom), z(dom);

  set_int_vw_tp u_vw(u), v_vw(v), w_vw(w);
  set_int_vw_tp x_vw(x), y_vw(y), z_vw(z);

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  init_set( u_vw, v_vw, w_vw, x_vw, y_vw, z_vw, model);

  stapl::rmi_fence();
  ctr.stop();
  double time_p = ctr.value();
#define DEBUG 1
#ifdef DEBUG
  stapl::do_once([&](){
    cerr << "dataparset init_set time " << time_p << endl;
  });
#endif

  int fail_count = 0;
  bool ok = true;
  for (int test = first_test; test <= last_test; test++ ) {
    switch (test) {
#ifdef DEBUG
  stapl::do_once([&](){
    cerr << "dataparset Test " << test << endl;
  });
#endif
    case 6:
      ok = test_006( u_vw,v_vw,w_vw,x_vw,y_vw,z_vw, size);
      break;
    case 7:
      ok = test_007( u_vw,v_vw,w_vw,x_vw,y_vw,z_vw, size);
      break;
    case 8:
      ok = test_008( u_vw,v_vw,w_vw,x_vw,y_vw,z_vw, size);
      break;
    case 9:
      ok = test_009( u_vw,v_vw,w_vw,x_vw,y_vw,z_vw, size);
      break;
#ifdef COMPLEMENT
    case 10:
      ok = test_010( u_vw,v_vw,w_vw,x_vw,y_vw,z_vw, size);
      break;
#endif
    }
    fail_count += ok == false;
  }

  stapl::do_once([&](){
    cerr << "Failures " << fail_count << endl;
  });

  return EXIT_SUCCESS;
}

//////////////////////////////////////////////////////////////////////
// initialize integer sets
//////////////////////////////////////////////////////////////////////


struct dpset_fill_wf {
  typedef void result_type;
  template <typename Count, typename View1>
  result_type operator()(Count ndx, View1 vw1)
  {
    int key = prime_nums[ rand_nums[ndx%data_cnt] ];
    vw1.insert(key);
  }
};

void init_set( set_int_vw_tp & u_vw, set_int_vw_tp & v_vw,
               set_int_vw_tp & w_vw, set_int_vw_tp & x_vw,
               set_int_vw_tp & y_vw, set_int_vw_tp & z_vw, int model ) {

  int count = model * 1000;

#ifdef OLD_STUFF
  for (int i = 0; i < model; i++) {
    for (int j = 0; j < 1000; j++) {
      key = (i*1000) + prime1000[rand1000_01[j%1000]]; // integer in
      assert( key < dom_lmt );
      u.insert(key);

      key = (i*1000) + prime1000[rand1000_02[j%500]]; // integer in
      assert( key < dom_lmt );
      v.insert(key);

      key = (i*1000+j) + prime1000[rand1000_03[j%100]]; // integer in
      assert( key < dom_lmt );
      w.insert(key);
    }
  }
#else
  stapl::map_func( dpset_fill_wf(), stapl::counting_view<int>(count),
                   stapl::make_repeat_view<set_int_vw_tp>(u_vw) );
  stapl::map_func( dpset_fill_wf(), stapl::counting_view<int>(count),
                   stapl::make_repeat_view<set_int_vw_tp>(v_vw) );
  stapl::map_func( dpset_fill_wf(), stapl::counting_view<int>(count),
                   stapl::make_repeat_view<set_int_vw_tp>(w_vw) );
#endif

}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// set membership
//////////////////////////////////////////////////////////////////////

bool test_006( set_int_vw_tp& u_vw, set_int_vw_tp& v_vw,
               set_int_vw_tp& w_vw, set_int_vw_tp& x_vw,
               set_int_vw_tp& y_vw, set_int_vw_tp& z_vw,
               size_t size ) {

  stapl::counter<stapl::default_timer> ctr;

  // reference

  bool ref_member_found = true;
  bool ref_non_member_not_found = true;

  ctr.start();

  // algorithm

#if 0
  int elem = 101; // forced insertion of this value in init function
#else
  int elem = prime_nums[ rand_nums[101%data_cnt] ];
#endif

  bool alg_member_found = false;
  bool alg_non_member_not_found = false;

  if ( u_vw.end() != u_vw.find(elem) ) { // member
    alg_member_found = true;
  }

  elem = -1;
  if ( u_vw.end() == u_vw.find(elem) ) { // non member
    alg_non_member_not_found = true;
  }

  stapl::rmi_fence();
  ctr.stop();
  double time_p = ctr.value();
  ctr.reset();

  bool pass = ref_member_found &&
              ref_member_found == alg_member_found &&
              ref_non_member_not_found &&
              ref_non_member_not_found == alg_non_member_not_found;

  stapl::do_once([&](){
    cerr << "dataparset test 006: " <<
            ref_member_found << " " << ref_non_member_not_found << " " <<
            alg_member_found << " " << alg_non_member_not_found << endl;
  });

  return check_scalar<bool>( "006", "STAPL", "manual",
                             0.0, time_p, 0, 0, pass, true );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// set union
//////////////////////////////////////////////////////////////////////

struct set_copy_wf
{
  typedef void result_type;
  template <typename Elem, typename View1>
  void operator()(Elem u_elem, View1 z_set) {
// GH: assert happens here
    z_set.insert(u_elem);
  }
};

struct set_union_wf
{
  typedef int result_type;
  template <typename Elem, typename View>
  result_type operator()(Elem v_elem, View z_set) {
    // is right not in result?
    if ( z_set.end() == z_set.find(v_elem) ) {
      z_set.insert(v_elem);
      return 1;
    } else {
      return 0;
    }
  }
};

bool test_007( set_int_vw_tp& u_vw, set_int_vw_tp& v_vw,
               set_int_vw_tp& w_vw, set_int_vw_tp& x_vw,
               set_int_vw_tp& y_vw, set_int_vw_tp& z_vw,
               size_t size ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int old_sz = u_vw.size();

#ifdef OLD_STUFF
  int inserts = 0;
  // start with all elements of left
  set_int_tp::iterator u_iter;
  for ( u_iter = u.begin(); u_iter != u.end(); u_iter++ ) {
    int u_elem = (*u_iter);
    z.insert(u_elem);
  }
  set_int_tp::iterator v_iter;
  for ( v_iter = v.begin(); v_iter != v.end(); v_iter++ ) {
    int v_elem = (*v_iter);
    // is right not in result?
    if ( z.end() == z.find(v_elem) ) {
      z.insert(v_elem);
      inserts++;
    }
  }
#else
#ifdef DEBUG
stapl::do_once([&](){
  cerr << "test_007 a" << endl;
});
#endif
  // copy all elements of left argument
  stapl::map_func( set_copy_wf(), u_vw, stapl::make_repeat_view(z_vw) );
#ifdef DEBUG
stapl::do_once([&](){
  cerr << "test_007 b" << endl;
});
#endif
  // insert elements of right argument not in result
  int inserts = stapl::map_reduce( set_union_wf(), stapl::plus<int>(),
                                   v_vw, stapl::make_repeat_view(z_vw) );
#endif

  int new_sz = z_vw.size();

  stapl::rmi_fence();
  ctr.stop();
  double time_p = ctr.value();
  ctr.reset();

  bool pass = old_sz == new_sz - inserts;

  return check_scalar<bool>( "007", "STAPL", "manual",
                             0.0, time_p, 0, 0, pass, true );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// set difference
//////////////////////////////////////////////////////////////////////

struct set_difference_wf
{
  typedef int result_type;
  template <typename Elem, typename View2, typename View3>
  result_type operator()(Elem u_elem, View2 v_set, View3 z_set) {
    // is left not in right?
    if ( v_set.end() == v_set.find(u_elem) ) {
      z_set.insert( u_elem );
      return 1;
    } else {
      return 0;
    }
  }
};

bool test_008( set_int_vw_tp& u_vw, set_int_vw_tp& v_vw,
               set_int_vw_tp& w_vw, set_int_vw_tp& x_vw,
               set_int_vw_tp& y_vw, set_int_vw_tp& z_vw,
               size_t size ) {

#ifdef DEBUG
stapl::do_once([&](){
  cerr << "test_008 " << endl;
});
#endif

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int old_sz = u_vw.size();

#ifdef OLD_STUFF
  int inserts = 0;
  set_int_tp::iterator u_iter;
  for ( u_iter = u.begin(); u_iter != u.end(); u_iter++ ) {
    int u_elem = (*u_iter);
    // is left not in right?
    if ( v.end() == v.find(u_elem) ) {
      z.insert( u_elem );
      inserts++;
    }
  }
#else
  int inserts = stapl::map_reduce( set_difference_wf(), stapl::plus<int>(),
                                   u_vw, stapl::make_repeat_view(v_vw),
                                   stapl::make_repeat_view(z_vw) );
#endif

  int new_sz = z_vw.size();

  stapl::rmi_fence();
  ctr.stop();
  double time_p = ctr.value();
  ctr.reset();

  bool pass = old_sz == new_sz + inserts;

  return check_scalar<bool>( "008", "STAPL", "manual",
                             0.0, time_p, 0, 0, pass, true );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// set intersection
//////////////////////////////////////////////////////////////////////

struct set_intersection_wf
{
  typedef int result_type;
  template <typename View1, typename Elem, typename View3>
  result_type operator()(View1 u_set, Elem v_elem, View3 z_set) {
    // right is in left?
    if ( z_set.end() != u_set.find(v_elem) ) {
      z_set.insert( v_elem );
      return 1;
    } else {
      return 0;
    }
  }
};

bool test_009( set_int_vw_tp& u_vw, set_int_vw_tp& v_vw,
               set_int_vw_tp& w_vw, set_int_vw_tp& x_vw,
               set_int_vw_tp& y_vw, set_int_vw_tp& z_vw,
               size_t size ) {

#ifdef DEBUG
stapl::do_once([&](){
  cerr << "test_009" << endl;
});
#endif

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int old_sz = u_vw.size();

#ifdef OLD_STUFF
  int inserts = 0;
  set_int_tp::iterator v_iter;
  for ( v_iter = v.begin(); v_iter != v.end(); v_iter++ ) {
    int v_elem = (*v_iter);
    // right is in left?
    if ( z.end() != u.find(v_elem) ) {
      z.insert( v_elem );
      inserts++;
    }
  }
#else
  int inserts = stapl::map_reduce( set_intersection_wf(), stapl::plus<int>(),
                                   stapl::make_repeat_view(u_vw), v_vw,
                                   stapl::make_repeat_view(z_vw) );
#endif

  int new_sz = z_vw.size();

  stapl::rmi_fence();
  ctr.stop();
  double time_p = ctr.value();
  ctr.reset();

  bool pass = old_sz == new_sz + inserts;

  return check_scalar<bool>( "009", "STAPL", "manual",
                             0.0, time_p, 0, 0, pass, true );
}

#ifdef COMPLEMENT
//////////////////////////////////////////////////////////////////////
// FEATURES:
// set complement - difference with universe
//////////////////////////////////////////////////////////////////////

bool test_010( set_int_tp &u, set_int_tp &v, set_int_tp &w,
               set_int_tp &x, set_int_tp &y, set_int_tp &z,
               set_int_vw_tp& u_vw, set_int_vw_tp& v_vw,
               set_int_vw_tp& w_vw, set_int_vw_tp& x_vw,
               set_int_vw_tp& y_vw, set_int_vw_tp& z_vw,
               size_t size ) {

#if 0
  int max_val = stapl::max_value(left_view);
  stapl::map<int,int> universe;
#endif

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // algorithm

  ctr.stop();
  double 0.0 = ctr.value();

  ctr.start();

  // reference

  stapl::rmi_fence();
  ctr.stop();
  double time_p = ctr.value();
  ctr.reset();

  bool pass = z_vw.size() == z_vw.size(); // FIXME

  return check_scalar<bool>( "010", "STAPL", "manual",
                             0.0, time_p, 0, 0, pass, true );
}
#endif
