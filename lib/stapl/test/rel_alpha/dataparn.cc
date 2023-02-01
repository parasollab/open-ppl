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
#include <iostream>

#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/segmented_view.hpp>

#include <stapl/skeletons/serial.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/sorting.hpp>
#include <stapl/algorithms/functional.hpp>

#include <stapl/runtime.hpp>

#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include "json_parser.hpp"

using namespace std;
#include "testutil.hpp"
#include "rel_alpha_data.h"

#define PARTN_BUG 1
#undef NESTED
#undef REV_COPY

using namespace std;

template<typename T>
struct test_pair
{
  char const* name;
  T test;
};

//////////////////////////////////////////////////////////////////////

typedef stapl::negate<int> neg_int_wf;
typedef stapl::identity<int> id_int_wf;
typedef stapl::plus<int> add_int_wf;
typedef stapl::minus<int> sub_int_wf;
typedef stapl::multiplies<int> mul_int_wf;
typedef stapl::min<int> min_int_wf;
typedef stapl::max<int> max_int_wf;
typedef stapl::logical_or<int> or_int_wf;
typedef stapl::logical_and<int> and_int_wf;
typedef stapl::equal_to<int> eq_int_wf;
typedef stapl::not_equal_to<int> ne_int_wf;

template <typename Functor>
struct self_assign
{
  typedef void result_type;

  Functor m_func;

  self_assign(Functor const& f)
    : m_func(f)
  {}

  void define_type(stapl::typer& t)
  {
    t.member(m_func);
  }

  template <typename Ref>
  void operator()(Ref val)
  {
    val = m_func(val);
  }
};

template <typename Functor>
self_assign<Functor> make_self_assign(Functor const& f)
{
  return self_assign<Functor>(f);
}

struct roll_wf
{
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 length, View2 limit) const
  {
    length = 1 + (rand() % limit);
  }
};

struct copy_flat2nest_wf
{
  typedef void result_type;

  template <typename SegRef, typename View1>
  void operator()(SegRef seg, View1 vw)
  {
    int j = 0;
    typename SegRef::iterator iter;
    for ( iter = seg.begin(); iter != seg.end(); ++iter ) {
      vw[j++] = *iter;
    }
  }
};

struct red_scan_debug_wf
{
  typedef void result_type;

  template <typename SegRef, typename View1>
  void operator()(SegRef seg, View1 vw)
  {
    int j = 0;
    typename SegRef::iterator iter;
    for ( iter = seg.begin(); iter != seg.end(); ++iter ) {
      cerr << j << " ";
      j++;
    }
    cerr << endl;
  }
};

//////////////////////////////////////////////////////////////////////

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector< vec_int_tp > vec_vec_int_tp;

typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;
typedef stapl::vector_view<vec_vec_int_tp> vec_vec_int_vw_tp;

typedef vec_int_vw_tp::domain_type dom_tp;

typedef stapl::vector<size_t> vec_sz_tp;
typedef stapl::vector_view<vec_sz_tp> vec_sz_vw_tp;

typedef stapl::splitter_partition<vec_int_vw_tp::domain_type> seg_splitter;
typedef stapl::segmented_view<vec_int_vw_tp, seg_splitter> seg_vw_tp;

//////////////////////////////////////////////////////////////////////

bool test_001(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_002(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_003(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_004(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_005(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_006(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_007(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_008(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_009(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_010(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_011(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_012(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_013(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_014(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_015(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_016(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_017(
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_int_tp &, vec_int_tp &, vec_int_tp &, vec_int_tp &,
     vec_sz_tp &, vec_sz_tp &,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&, vec_int_vw_tp&,
     vec_sz_vw_tp&, vec_sz_vw_tp& );

bool test_101( size_t );
bool test_102( size_t );
bool test_201( size_t );
bool test_202( size_t );
bool test_203( size_t );
bool test_204( size_t );
bool test_205( size_t );
bool test_206( size_t );
bool test_207( size_t );
bool test_301( size_t );
bool test_302( size_t );

void init_int( vec_int_tp &, vec_int_tp &,
               vec_int_tp &, vec_int_tp &,
               vec_int_tp &, vec_int_tp &,
               vec_int_tp &, vec_int_tp &,
               vec_sz_tp &, vec_sz_tp &,
               vec_sz_vw_tp&, vec_sz_vw_tp&, int);

//////////////////////////////////////////////////////////////////////
// DATA PARALLEL TESTS - 1 DIMENSIONAL ARRAY
//////////////////////////////////////////////////////////////////////

bool opt_list = false, opt_noref = false;
bool opt_out = false, opt_quiet = false;
int opt_test = -1;
char *opt_data = 0;
bool opt_stapl= false;
bool opt_manual= false;

stapl::exit_code stapl_main(int argc, char **argv)
{
  char *temp = 0;
  for ( int argi = 1; argi < argc; ) {
    char * opt = argv[argi++];
    if ('-' == opt[0] ) {
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

  int first_test = 1;
  int last_test = 17;
  if (opt_test != -1 ) {
    first_test = opt_test;
    last_test = opt_test;
  }

  int size = 1000 * model;

  vec_int_tp a(size), b(size), c(size), d(size);
  vec_int_tp e(size), f(size), g(size), h(size);
  vec_sz_tp p(size), q(size);

  vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c), d_vw(d);
  vec_int_vw_tp e_vw(e), f_vw(f), g_vw(g), h_vw(h);
  vec_sz_vw_tp p_vw(p), q_vw(q);

  init_int( a, b, c, d, e, f, g, h, p, q, p_vw, q_vw, model);

  int fail_count = 0;
  bool ok = true;

  for (int test = first_test; test <= last_test; test++ ) {
#ifdef DEBUG
if( 0==stapl::get_location_id() ) {
  cerr << "TEST " << test << endl;
}
#endif
    switch (test) {
    case 1:
      ok = test_001(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 2:
      ok = test_002(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 3:
      ok = test_003(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 4:
      ok = test_004(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 5:
      ok = test_005(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 6:
      ok = test_006(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 7:
      ok = test_007(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 8:
      ok = test_008(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 9:
      ok = test_009(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 10:
      ok = test_010(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 11:
      ok = test_011(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 12:
      ok = test_012(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 13:
      ok = test_013(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 14:
      ok = test_014(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 15:
      ok = test_015(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 16:
      ok = test_016(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;
    case 17:
      ok = test_017(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw);
      break;

    case 18:
      ok = test_101( model );
      break;
    case 19:
      ok = test_102( model );
      break;
    case 20:
      ok = test_201( model );
      break;
    case 21:
      ok = test_202( model );
      break;
    case 22:
      ok = test_203( model );
      break;
    case 23:
      ok = test_204( model );
      break;
    case 24:
      ok = test_205( model );
      break;
    case 25:
      ok = test_206( model );
      break;
    case 26:
      ok = test_207( model );
      break;
#if 0
    case 27:
      ok = test_301( model );
      break;
    case 28:
      ok = test_302( model );
      break;
#endif
    }
  }
  return EXIT_SUCCESS;
}

//////////////////////////////////////////////////////////////////////
// initialize integer data
//////////////////////////////////////////////////////////////////////

void init_int( vec_int_tp & a, vec_int_tp & b,
               vec_int_tp & c, vec_int_tp & d,
               vec_int_tp & e, vec_int_tp & f,
               vec_int_tp & g, vec_int_tp & h,
               vec_sz_tp & p, vec_sz_tp & q,
               vec_sz_vw_tp &p_vw, vec_sz_vw_tp & q_vw, int model)
{
  int fibo[10] = { /* 1, 2, 3, 5, 8, */
                   13, 21, 34, 55, 89,
                   144, 233, 377, 610, 987,
                   /* 1597, 2584, 4181, 6765, 10946 */ };

  for (int i = 0; i < model; i++) {
    int n = i * 1000;
    for (int j = 0; j < 1000; j++) {
      a.set_element( n+j, prime1000[rand1000_01[j]] ); // int in
      b.set_element( n+j, prime1000[rand1000_02[j]] ); // int in
      c.set_element( n+j, 0 ); // bool in
      d.set_element( n+j, 1 ); // bool in
      e.set_element( n+j, prime1000[rand1000_05[j]] % 100 ); // dup int in
      f.set_element( n+j, 0); // temp/output
      g.set_element( n+j, 0); // temp/output
      h.set_element( n+j, 0); // temp/output
      p.set_element( n+j, 0); // segments
      q.set_element( n+j, 0); // segments
    }
  }

  // overwrite selected elements of boolean vectors
  for (int i = 0; i < model; i++) {
    int n = i * 1000;
    for (int j = 0; j < 1000; j++) {
      int k = j % 10;
      c.set_element( n + fibo[k], 1 );
      d.set_element( n + fibo[k], 0 );
    }
  }

#if 0
  set_random_seed();

  int next = 0;
  for (int i = 0; i < model; i++) {
    int span = 0;
    for (int j = 0; j < 100 && span < 1000; j++ ) {
      int k = 1 + rand() % 20;
      p[next++] = k;
      span += k;
    }
    if (span < 1000 ) {
      p[next++] = 1000 - span;
    }
  }

  int temp = next-1;
  int last = p.size()-1;
  while ( temp < last ) {
    p.erase(last);
    q.erase(last);
    last--;
  }

  // value of splitters
  (void) stapl::partial_sum(p_vw, q_vw);
  q.erase(last--);
#endif

#ifdef DEBUG
  cerr << "size " << p.size() << endl;
  for ( int i=0; i<p.size(); i++ ) {
    cerr << p[i] << " ";
    if ( 0 == (i+1) % 10 ) {
      cerr << endl;
    }
  }
  cerr << endl;

  cerr << "size " << q.size() << endl;
  for ( int i=0; i<q.size(); i++ ) {
    cerr << q[i] << " ";
    if ( 0 == (i+1) % 10 ) {
      cerr << endl;
    }
  }
  cerr << endl;
#endif
}


/*=========================================================================
 * COMPUTATIONS ON UNNESTED SEQUENCES
 *=========================================================================*/

//////////////////////////////////////////////////////////////////////
// FEATURES:
// NESL: dist(a,l)    # distribute value a to sequence of length l
//////////////////////////////////////////////////////////////////////

bool test_001(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  int x = 17, y = 42, z = 99;

  stapl::fill_n(g_vw, x, y);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  for (int i = 0; i < y; i++) {
    h[i] = x;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "001", "STAPL", "manual",
                                      time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// NESL: iseq(s, d, e) # integers starting at s, ending at e, by d
//////////////////////////////////////////////////////////////////////

bool test_002(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;

  int size = a.size();
  int start = 10;
  int delta = 5;
  int end = size / 2;
  int count = ( 1 + end - start ) / delta;

  // algorithm

  ctr.start();

  dom_tp g_dom = g_vw.domain();
  dom_tp z_dom( 0, count-1, g_dom);
  vec_int_vw_tp z_vw(g_vw.container(), z_dom);

  stapl::iota(z_vw, 0);
  stapl::for_each(z_vw,
         make_self_assign( bind(stapl::plus<int>(),_1,start/delta)) );
  stapl::for_each(z_vw,
         make_self_assign( bind(stapl::multiplies<int>(),_1,delta)) );

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  int i = start;
  for (int j = 0; j < count; j++ ) {
    h[j] = i;
    i += delta;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "002", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: transform unary
// NESL:                 # apply function to sequence
//////////////////////////////////////////////////////////////////////

bool test_003(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  stapl::transform(a_vw, g_vw, neg_int_wf());

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  int size = a.size();
  for (int i = 0; i < size; i++) {
    h[i] = -a[i];
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "003", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: transform binary
// NESL:              # apply function to sequence
//////////////////////////////////////////////////////////////////////

bool test_004(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  stapl::transform(a_vw,b_vw, g_vw, min_int_wf());

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  int size = a.size();
  for (int i = 0; i < size; i++) {
    h[i] = std::min(a[i], b[i]);
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "004", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// NESL: count(a)     # count number of true values in a
//////////////////////////////////////////////////////////////////////

bool test_005(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  vec_int_vw_tp::iterator::difference_type alg_int;
  alg_int = stapl::count_if(c_vw,
                              bind(stapl::equal_to<int>(),_1,1) );
  int alg_cnt = (int)alg_int;

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  int size = c.size();
  int ref_cnt = 0;
  for (int i = 0; i < size; i++) {
    if (c[i] == 1 ) {
      ref_cnt++;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_scalar<int>( "005", "STAPL", "manual",
                            time1, time2, 0, 0, alg_cnt, ref_cnt );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: copy, domains, set_element
// NESL: a ++ b       # append sequences a and b
//////////////////////////////////////////////////////////////////////

bool test_006(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  return known_fail( "006", "STAPL", "manual",
                     "infinite loop, test problem", 0 );

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  int a_size = a.size();
  int b_size = a.size();

  vec_int_tp z(a_size+b_size);
  vec_int_vw_tp z_vw(z);
  dom_tp z_dom = z_vw.domain();

  dom_tp a_dom = a_vw.domain();
  dom_tp b_dom = b_vw.domain();

  dom_tp x_dom( 0, a_size-1, z_dom);
  dom_tp y_dom( a_size, a_size+b_size, z_dom);

  vec_int_vw_tp x_vw(z_vw.container(), x_dom);
  vec_int_vw_tp y_vw(z_vw.container(), y_dom);

  stapl::copy(a_vw, x_vw);
  stapl::copy(b_vw, y_vw);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  vec_int_tp w(a_size+b_size);
  vec_int_vw_tp w_vw(w);

  int i = 0;
  for (; i != a_size; ++i) {
    w.set_element(i, a[i] );
  }
  for (int j = 0 ; j != b_size; ++i, ++j) {
    w.set_element(i, b[j] );
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "006", "STAPL", "manual",
                                          time1, time2, 0, 0, z, w );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// NESL: take(a,n)    # take first n elements of sequence a
//////////////////////////////////////////////////////////////////////

bool test_007(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 97;
  int size = a.size();

  // algorithm

  // input domains and views
  dom_tp b_dom = b_vw.domain();
  dom_tp bpre_dom( 0, n-1, b_dom);
  vec_int_vw_tp bpre_vw(b_vw.container(), bpre_dom);

  // output domains and views
  dom_tp g_dom = g_vw.domain();
  dom_tp gpre_dom( 0, n-1, g_dom);
  dom_tp gpost_dom( n, size-1, g_dom);
  vec_int_vw_tp gpre_vw(g_vw.container(), gpre_dom);
  vec_int_vw_tp gpost_vw(g_vw.container(), gpost_dom);

  stapl::copy(bpre_vw, gpre_vw);
  stapl::fill_n(gpost_vw, -12345, size-n);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  ctr.start();

  // reference

  int j = 0;
  for (int i = 0; i <=n-1; i++) {
    h[j++] = b[i];
  }
  for (int i = n; i <=size-1; i++) {
    h[j++] = -12345;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "007", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// NESL: take(a,n)    # take last n elements of sequence a
//////////////////////////////////////////////////////////////////////

bool test_008(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 97;
  int size = a.size();

  // algorithm

  // input domains and views
  dom_tp b_dom = b_vw.domain();
  dom_tp bpost_dom( size-n, size-1, b_dom);
  vec_int_vw_tp bpost_vw(b_vw.container(), bpost_dom);

  // output domains and views
  dom_tp g_dom = g_vw.domain();
  dom_tp gpre_dom( 0, n-1, g_dom);
  dom_tp gpost_dom( n, size-1, g_dom);
  vec_int_vw_tp gpre_vw(g_vw.container(), gpre_dom);
  vec_int_vw_tp gpost_vw(g_vw.container(), gpost_dom);

  int bcount = 1 + (bpost_dom.last() - bpost_dom.first());
  int gcount = 1 + (gpre_dom.last() - gpre_dom.first());
  assert( bcount == gcount );
  assert( bcount == n );

  stapl::copy(bpost_vw, gpre_vw);
  stapl::fill_n(gpost_vw, -12345, size-n);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  ctr.start();

  // reference

  int j = 0;
  for (int i = size-n; i < size; i++) {
    h[j++] = b[i];
  }
  for (int i = n; i < size; i++) {
    h[j++] = -12345;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "008", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// NESL: drop(a,n)    # drop first n elements of sequence a
//////////////////////////////////////////////////////////////////////

bool test_009(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;

  int n = 97;
  int size = a.size();

  // algorithm

  ctr.start();

  // input domains and views
  dom_tp b_dom = b_vw.domain();
  dom_tp bpre_dom( 0, n-1, b_dom); // not used
  dom_tp bpost_dom( n, size-1, b_dom);
  vec_int_vw_tp bpost_vw(b_vw.container(), bpost_dom);

  // output domains and views
  dom_tp g_dom = g_vw.domain();
  dom_tp gpre_dom( 0, size-(n+1), g_dom);
  dom_tp gpost_dom( size-n, size-1, g_dom);

  vec_int_vw_tp gpre_vw(g_vw.container(), gpre_dom);
  vec_int_vw_tp gpost_vw(g_vw.container(), gpost_dom);

  int bcount = 1 + (bpost_dom.last() - bpost_dom.first());
  int gcount = 1 + (gpre_dom.last() - gpre_dom.first());
  assert( bcount == gcount );
  assert( bcount == size - n );

  stapl::copy(bpost_vw, gpre_vw);
  stapl::fill(gpost_vw, -12345);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  int j = 0;
  for (int i = n; i < size; i++) {
    h[j++] = b[i];
  }
  for (int i = 0; i <n; i++) {
    h[j++] = -12345;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "009", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// NESL: drop(a,n)    # drop last n elements of sequence a
//////////////////////////////////////////////////////////////////////

bool test_010(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  int n = 97;
  int size = a.size();

  ctr.start();

  // input domains and views
  dom_tp b_dom = b_vw.domain();
  dom_tp bpre_dom( 0, size-(n+1), b_dom);
  dom_tp bpost_dom( size-n, size-1, b_dom); // not used
  vec_int_vw_tp bpre_vw(b_vw.container(), bpre_dom);

  // output domains and views
  dom_tp g_dom = g_vw.domain();
  dom_tp gpre_dom( 0, size-(n+1), g_dom);
  dom_tp gpost_dom( size-n, size-1, g_dom);
  vec_int_vw_tp gpre_vw(g_vw.container(), gpre_dom);
  vec_int_vw_tp gpost_vw(g_vw.container(), gpost_dom);

  int bcount = 1 + bpre_dom.last() - bpre_dom.first();
  int gcount = 1 + gpre_dom.last() - gpre_dom.first();
  assert( bcount == gcount );
  assert( gcount == size - n );

  stapl::copy(bpre_vw, gpre_vw);
  stapl::fill(gpost_vw, -12345);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  int j = 0;
  for (int i = 0; i < size-n; i++) {
    h[j++] = b[i];
  }
  for (int i = 0; i <n; i++) {
    h[j++] = -12345;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "010", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

struct reverse_parts
{
  typedef void result_type;

  template <typename SegRef, typename IndexRef>
  void operator()(SegRef p, IndexRef i)
  {
    int total = 0; // remove

    typename SegRef::iterator bw = p.end();
    --bw;
    for (typename SegRef::iterator fw = p.begin(); fw != p.end(); ++fw) {
      int temp = *fw;
      *fw = *bw;
      *bw = temp;
    }
  }
};

//////////////////////////////////////////////////////////////////////
// FEATURES:
// NESL: reverse(a)  # reverse elements in a sequence
//////////////////////////////////////////////////////////////////////

bool test_011(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  return known_fail( "011", "STAPL", "manual",
                     "stapl::reverse_copy() aborts at compile time", 0 );

  stapl::counter<stapl::default_timer> ctr;

  int n = 97;
  int size = a.size();

  // algorithm

  ctr.start();

#ifdef REV_COPY
  stapl::reverse_copy(a_vw, b_vw);
#endif

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  int j = size - 1;
  for (int i = 0; i < size; i++) {
    h[j] = c[i];
    --j;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "011", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

struct rotate_parts
{
  typedef void result_type;

  template <typename SegRef, typename IndexRef>
  void operator()(SegRef p, IndexRef i)
  {
    for (typename SegRef::iterator it = p.begin(); it != p.end(); ++it) {
      if (it == p.begin()) {
      } else {
      }
    }
  }
};

//////////////////////////////////////////////////////////////////////
// FEATURES:
// NESL: rotate(a,n)  # rotate sequence a by n positions
//////////////////////////////////////////////////////////////////////

bool test_012(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;

  int n = 97;
  int size = a.size();

  // algorithm

  ctr.start();

  stapl::rotate_copy(b_vw, g_vw, n);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  int j = 0;
  for (int i = n; i < size; i++) {
    h[j++] = b[i];
  }
  for (int i = 0; i <n; i++) {
    h[j++] = b[i];
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "012", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// ## BASE ##
// FEATURES:
// NESL: length_from_flags(a)   # compute segment lengths
//////////////////////////////////////////////////////////////////////

bool test_013(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  return known_fail( "013", "STAPL", "manual",
                     "runtime error ", 0 );

  stapl::counter<stapl::default_timer> ctr;

  int size = c.size();

  // algorithm

  ctr.start();

  int j = 0;
  for (int i = 0; i < size; i++) {
    if (c[i] == 1 ) {
      f[j++] = i;
    }
  }
  int count = j;
  dom_tp p_dom = f_vw.domain();
  dom_tp z_dom( 0, count-1, p_dom);
  vec_int_vw_tp z_vw(f_vw.container(), z_dom);

  stapl::adjacent_difference(z_vw, g_vw);
  for (int i = 1; i < count; i++) {
    g[i-1] = g[i];
  }
  for (int i=count-1; i<size; i++ ) {
    g[i] = 0;
  }

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  int prev = -1;
  j = -1;
  for (int i = 0; i < size; i++) {
    if (c[i] == 1 ) {
      if (j == -1 ) {
        prev = i;
        j = 0;
      } else {
        h[j++] = i - prev;
        prev = i;
      }
    }
  }
  for (int i=count-1; i<size; i++ ) {
    h[i] = 0;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "013", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// NESL: remove_dups(s) # remove duplicates from sequence
//////////////////////////////////////////////////////////////////////

bool test_014(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  stapl::copy(e_vw, f_vw);
  stapl::unique_copy(f_vw, g_vw);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  stapl::copy(e_vw, f_vw);

  int size = e.size();
  int k = 0;
  for (int i = 1; i < size; i++ ) {
    if (f[i] == f[i-1] ) {
      k++;
    }
  }

  int j = 0;
  h[j++] = f[0];
  for (int i = 1; i < size; i++) {
    if (f[i] != f[i-1] ) {
      h[j++] = f[i];
    } else {
      h[size-k] = f[i];
      k--;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "014", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: copy, sort, sample_sort
// NESL: sort(a)        # sort sequence
//////////////////////////////////////////////////////////////////////

bool test_015(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  stapl::copy(a_vw, g_vw);
  stapl::sample_sort(g_vw);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  stapl::copy(a_vw, h_vw);
  stapl::sort(h_vw);

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "015", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: find
// NESL:  find(elm, seq)
//////////////////////////////////////////////////////////////////////

bool test_016(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;

  int size = a.size();

  // algorithm

  ctr.start();

  int x = a[size/2];
  vec_int_vw_tp::reference alg_ref = stapl::find(a_vw, x);
  if (is_null_reference(alg_ref) ) {
    return false;
  }
  int alg_int = index_of(alg_ref);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  int ref_int = -1;
  for (int i = 0; i < size; i++ ) {
    if ( a[i] == x ) {
      ref_int = i;
      break;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_scalar<int>( "016", "STAPL", "manual",
                            time1, time2, 0, 0, alg_int, ref_int );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: equal
// APL: eql(a,b)
//////////////////////////////////////////////////////////////////////

bool test_017(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  stapl::counter<stapl::default_timer> ctr;

  int size = a.size();

  // algorithm

  ctr.start();

  bool alg_bool = stapl::equal(a_vw, b_vw);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  bool ref_bool = true;
  for (int i = 0; i < size; i++) {
    if (a[i] != b[i] ) {
      ref_bool = false;
      break;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_scalar<bool>( "017", "STAPL", "manual",
                             time1, time2, 0, 0, alg_bool, ref_bool );
}

/*=========================================================================
 * ASSEMBLE AND DISASSEMBLE NESTED SEQUENCES
 *=========================================================================*/

//////////////////////////////////////////////////////////////////////
// ## BASE ##
// FEATURES:
// NESL: partition(a,l) # partition sequence a into nested sequence
//////////////////////////////////////////////////////////////////////

struct test_101_copy_wf
{
  typedef void result_type;

  template <typename SegRef, typename View1>
  void operator()(SegRef seg, View1 vw)
  {
    int j = 0;
    typename SegRef::iterator iter;
    for ( iter = seg.begin(); iter != seg.end(); ++iter ) {
      vw[j++] = *iter;
    }
  }
};

bool test_101( size_t model ) {

  // build data structures

if( 0 == stapl::get_location_id() ) {
  cerr << "Test 101 BEGIN " << endl << endl;
}
  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();

  vec_sz_tp p(size), q(size);
  vec_sz_vw_tp p_vw(p), q_vw(q);

  stapl::map_func(roll_wf(), p_vw, stapl::make_repeat_view(limit));
  stapl::rmi_fence();

  vec_vec_int_tp aa(p_vw), bb(p_vw);
  vec_vec_int_vw_tp aa_vw(aa), bb_vw(bb);

  int len = 0;
  for ( size_t i= 0; i<p_vw.size(); i++ ) {
    len += p_vw[i];
  }

  vec_int_tp a(len), b(len);
  vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::iota( a_vw, 0 );
  stapl::rmi_fence();

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  // create a segmented view over the source

  // subtract 1 from the size because splitter_partition adds an element
#ifdef PARTN_BUG
  // the wrong way: STAPL defect forces this implementation
  std::vector<size_t> stdq(p_vw.size()-1);
  for ( size_t i= 0; i<p_vw.size()-1; i++ ) {
    stdq[i] = p.get_element(i);
  }
  stapl::rmi_fence();

  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());

if( 0 == stapl::get_location_id() ) {
  cerr << "Test 101 trace 4 " << endl << endl;
  stapl::rmi_fence();
  return false;
}

  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), stdq, true) ); // FAIL

#else
  // the right way
  //seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), p, true) );
#endif
  stapl::rmi_fence();

  // copy from the segmented view of the source to the nested sequence

  stapl::map_func(test_101_copy_wf(), seg_vw, aa_vw );
  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  // no other way to do this at the moment

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

#if 0
  return check_container<vec_int_tp>( "101", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
#else
  stapl::rmi_fence();
  if ( stapl::get_location_id() == 0 ) {
    cerr << "Test 101 RUN" << endl;
  }
  stapl::rmi_fence();
  return true;
#endif
}

//////////////////////////////////////////////////////////////////////
// ## BASE ##
// FEATURES:
// NESL: flatten(a)   # flatten nested sequence a
//////////////////////////////////////////////////////////////////////

struct test_102_copy_wf
{
  typedef void result_type;

  template <typename View1, typename SegRef>
  void operator()(View1 vw, SegRef seg)
  {
    int j = 0;
    typename SegRef::iterator iter;
    for ( iter = seg.begin(); iter != seg.end(); ++iter ) {
      *iter = vw[j++];
    }
  }
};

bool test_102( size_t model ) {

  // build data structures

if( 0 == stapl::get_location_id() ) {
  cerr << "Test 102 BEGIN " << endl << endl;
}
  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();

  vec_sz_tp p(size), q(size);
  vec_sz_vw_tp p_vw(p), q_vw(q);
if( 0 == stapl::get_location_id() ) {
  cerr << "Test 102 TRACE 0: " << p_vw.size() << endl;
  return false;
}

  stapl::map_func(roll_wf(), p_vw, stapl::make_repeat_view(limit));
  stapl::rmi_fence();

  vec_vec_int_tp aa(p_vw), bb(p_vw);
  vec_vec_int_vw_tp aa_vw(aa), bb_vw(bb);

if( 0 == stapl::get_location_id() ) {
  cerr << "Test 102 TRACE 1 " << endl << endl;
  stapl::rmi_fence();
  return false;
}

  int len = 0;
  for ( size_t i= 0; i<p_vw.size(); i++ ) {
    len += p_vw[i];
  }

if( 0 == stapl::get_location_id() ) {
  cerr << "Test 102 TRACE 2 " << endl << endl;
  stapl::rmi_fence();
  return false;
}
  vec_int_tp a(len), b(len);
  vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::iota( a_vw, 0 );
  stapl::rmi_fence();

if( 0 == stapl::get_location_id() ) {
  cerr << "Test 102 TRACE 3 " << endl << endl;
  stapl::rmi_fence();
  return false;
}
  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  // create a segmented view over the source

  // subtract 1 from the size because splitter_partition adds an element
#ifdef PARTN_BUG
  // the wrong way: STAPL defect forces this implementation
  std::vector<size_t> stdq(p.size()-1);
  for (size_t i = 0; i<p.size()-1; i++ ) {
    stdq[i] = p.get_element(i);
  }
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), stdq, true) );
#else
  // the right way
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), p, true) );
#endif
  stapl::rmi_fence();

  // copy from the segmented view of the source to the nested sequence

  assert( seg_vw.size() == aa_vw.size() );
  stapl::map_func( test_102_copy_wf(), aa_vw, seg_vw );

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#if 0
  // reference

  ctr.start();

  // no other way to do this at the moment

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

cerr << "Test 102 DONE" << endl;

#if 0
  stapl::rmi_fence();
  return check_container<vec_int_tp>( "102", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
#else
  stapl::rmi_fence();
  if ( stapl::get_location_id() == 0 ) {
    cerr << "Test 102 RUN" << endl;
  }
  return true;
#endif
}

/*=========================================================================
 * REDUCTIONS AND SCANS OF NESTED SEQUENCES
 *=========================================================================*/

//////////////////////////////////////////////////////////////////////
// FEATURES: accumulate
// NESL: sum(a)       # return sum of sequence a
//////////////////////////////////////////////////////////////////////

struct test_201_process_wf {
  typedef int result_type;

  template <typename View1>
  int operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), vw1);
  }
};

struct test_201_check_wf {
  typedef int result_type;

  template <typename View1>
  int operator()(View1 const &vw1)
  {
    int id_val = stapl::identity_value<stapl::plus<int>, int>::value();
    int red = id_val;
    for (typename View1::size_type elem = 0; elem < vw1.size(); elem++) {
      red += vw1[elem];
    }
    return red;
  }
};

bool test_201( size_t model ) {

  // build the data structures

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();

  vec_sz_tp p(size), q(size);
  vec_sz_vw_tp p_vw(p), q_vw(q);

  stapl::map_func(roll_wf(), p_vw, stapl::make_repeat_view(limit));
  stapl::rmi_fence();

  vec_vec_int_tp aa(p_vw), bb(p_vw), cc(p_vw);
  vec_vec_int_vw_tp aa_vw(aa), bb_vw(bb), cc_vw(cc);

  int len = 0;
  for ( size_t i= 0; i<p_vw.size(); i++ ) {
    len += p_vw[i];
  }

  vec_int_tp a(len), b(len);
  vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::iota( a_vw, 0 );

  stapl::counter<stapl::default_timer> ctr;

  // create a segmented view over the source

  // subtract 1 from the size because splitter_partition adds an element
#ifdef PARTN_BUG
  // the wrong way: STAPL defect forces this implementation
  std::vector<size_t> stdq(p.size()-1);
  for (size_t i = 0; i<p.size()-1; i++ ) {
    stdq[i] = p.get_element(i);
  }
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), stdq, true) );
#else
  // the right way
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), p, true) );
#endif

  // copy from the segmented view of the source to the nested sequence

  assert( seg_vw.size() == aa_vw.size() );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, aa_vw );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, cc_vw );

  // algorithm

  ctr.start();

  stapl::map_func(test_201_process_wf(), aa_vw );

  stapl::rmi_fence();

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  stapl::serial(test_201_check_wf(), cc_vw );

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

#if 0
  return check_container<vec_int_tp>( "201", "STAPL", "manual",
                                      time1, time2, 0, 0, a, c );
#else
  if ( stapl::get_location_id() == 0 ) {
    cerr << "Test 201 PASS" << endl;
  }
  return true;
#endif
}

//////////////////////////////////////////////////////////////////////
// FEATURES: scan
// NESL:              plus_scan(a) # perform scan of sequence a
//////////////////////////////////////////////////////////////////////

struct test_202_process_wf {
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::scan(vw1, vw2, add_int_wf(), false);
  }
};

struct test_202_check_wf {
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 const &vw1, View2 const &vw2)
  {
    int size = vw1.size();
    vw2[0] = vw1[0];
    for (int i = 1; i < size; i++) {
      vw2[i] = vw1[i] + vw2[i-1];
    }
  }
};

bool test_202( size_t model ) {

  // build the data structures

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();

  vec_sz_tp p(size), q(size);
  vec_sz_vw_tp p_vw(p), q_vw(q);

  stapl::map_func(roll_wf(), p_vw, stapl::make_repeat_view(limit));
  stapl::rmi_fence();

  vec_vec_int_tp aa(p_vw), bb(p_vw), cc(p_vw);
  vec_vec_int_vw_tp aa_vw(aa), bb_vw(bb), cc_vw(cc);

  int len = 0;
  for ( size_t i= 0; i<p_vw.size(); i++ ) {
    len += p_vw[i];
  }

  vec_int_tp a(len), b(len), c(len);
  vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( a_vw, 0 );

  // create a segmented view over the source

  // subtract 1 from the size because splitter_partition adds an element
#ifdef PARTN_BUG
  // the wrong way: STAPL defect forces this implementation
  std::vector<size_t> stdq(p.size()-1);
  for (size_t i = 0; i<p.size()-1; i++ ) {
    stdq[i] = p.get_element(i);
  }
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), stdq, true) );
#else
  // the right way
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), p, true) );
#endif

  // copy from the segmented view of the source to the nested sequence

  assert( seg_vw.size() == aa_vw.size() );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, aa_vw );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, cc_vw );

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  stapl::map_func(test_202_process_wf(), aa_vw, bb_vw );

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  stapl::serial(test_202_check_wf(), aa_vw, cc_vw );

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

#if 0
  return check_container<vec_int_tp>( "202", "STAPL", "manual",
                                      time1, time2, 0, 0, a, c );
#else
  if ( stapl::get_location_id() == 0 ) {
    cerr << "Test 202 PASS" << endl;
  }
  return true;
#endif

}

//////////////////////////////////////////////////////////////////////
// FEATURES: max_value
// NESL: max_val     # maximum value of a sequence
//////////////////////////////////////////////////////////////////////

struct test_203_process_wf {
  typedef int result_type;

  template <typename View1>
  int operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), max_int_wf(), vw1);
  }
};

#define max_int(x,y) ((x>y)?(x):(y))

struct test_203_check_wf {
  typedef int result_type;

  template <typename View1>
  int operator()(View1 const &vw1)
  {
    int id_val = stapl::identity_value<stapl::max<int>, int>::value();
    int red = id_val;
    for (typename View1::size_type elem = 0; elem < vw1.size(); elem++) {
      //red = std::max(red, vw1[elem]);
      red = max_int(red, vw1[elem]);
    }
    return red;
  }
};

bool test_203( size_t model ) {

  // build the data structures

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();

  vec_sz_tp p(size), q(size);
  vec_sz_vw_tp p_vw(p), q_vw(q);

  stapl::map_func(roll_wf(), p_vw, stapl::make_repeat_view(limit));
  stapl::rmi_fence();

  vec_vec_int_tp aa(p_vw), bb(p_vw), cc(p_vw);
  vec_vec_int_vw_tp aa_vw(aa), bb_vw(bb), cc_vw(cc);

  int len = 0;
  for ( size_t i= 0; i<p_vw.size(); i++ ) {
    len += p_vw[i];
  }

  vec_int_tp a(len), b(len), c(len);
  vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( a_vw, 0 );

  // create a segmented view over the source

  // subtract 1 from the size because splitter_partition adds an element
#ifdef PARTN_BUG
  // the wrong way: STAPL defect forces this implementation
  std::vector<size_t> stdq(p.size()-1);
  for (size_t i = 0; i<p.size()-1; i++ ) {
    stdq[i] = p.get_element(i);
  }
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), stdq, true) );
#else
  // the right way
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), p, true) );
#endif

  // copy from the segmented view of the source to the nested sequence

  assert( seg_vw.size() == aa_vw.size() );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, aa_vw );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, cc_vw );

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  stapl::map_func(test_203_process_wf(), aa_vw );

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  stapl::serial(test_203_check_wf(), aa_vw );

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

#if 0
  return check_container<vec_int_tp>( "203", "STAPL", "manual",
                                      time1, time2, 0, 0, a, c );
#else
  if ( stapl::get_location_id() == 0 ) {
    cerr << "Test 203 PASS" << endl;
  }
  return true;
#endif

}

//////////////////////////////////////////////////////////////////////
// FEATURES: max_element
// NESL: max_index     # location of maximum value of a sequence
//////////////////////////////////////////////////////////////////////

struct test_204_process_wf {
  typedef int result_type;

  template <typename View1>
  int operator()(View1 const &vw1)
  {
    typename View1::reference alg_ref = stapl::max_element(vw1);
    if (stapl::is_null_reference(alg_ref) ) {
      return -1;
    }
    int alg_int = stapl::index_of(alg_ref);
    return alg_int;
  }
};

struct test_204_check_wf {
  typedef int result_type;

  template <typename View1>
  int operator()(View1 const &vw1)
  {
#if 0
  int size = a.size();
  int ref_int = -1;
  int temp = numeric_limits<int>::min();
  for (int i = 0; i < size; i++) {
    if (temp < a[i] ) {
      temp = a[i];
      ref_int = i;
    }
  }
#endif
  }
};

bool test_204( size_t model ) {

  // build data structures

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();

  vec_sz_tp p(size), q(size);
  vec_sz_vw_tp p_vw(p), q_vw(q);

  stapl::map_func(roll_wf(), p_vw, stapl::make_repeat_view(limit));
  stapl::rmi_fence();

  vec_vec_int_tp aa(p_vw), bb(p_vw), cc(p_vw);
  vec_vec_int_vw_tp aa_vw(aa), bb_vw(bb), cc_vw(cc);

  int len = 0;
  for ( size_t i= 0; i<p_vw.size(); i++ ) {
    len += p_vw[i];
  }

  vec_int_tp a(len), b(len), c(len);
  vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( a_vw, 0 );

  // create a segmented view over the source

  // subtract 1 from the size because splitter_partition adds an element
#ifdef PARTN_BUG
  // the wrong way: STAPL defect forces this implementation
  std::vector<size_t> stdq(p.size()-1);
  for (size_t i = 0; i<p.size()-1; i++ ) {
    stdq[i] = p.get_element(i);
  }
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), stdq, true) );
#else
  // the right way
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), p, true) );
#endif

  // copy from the segmented view of the source to the nested sequence

  assert( seg_vw.size() == aa_vw.size() );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, aa_vw );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, cc_vw );

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  stapl::map_func(test_204_process_wf(), aa_vw );

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  stapl::serial(test_204_check_wf(), aa_vw );

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

#if 0
  return check_scalar<int>( "204", "STAPL", "manual",
                            time1, time2, 0, 0, alg_int, ref_int );
#else
  if ( stapl::get_location_id() == 0 ) {
    cerr << "Test 204 PASS" << endl;
  }
  return true;
#endif
}

//////////////////////////////////////////////////////////////////////
// FEATURES: scan
// NESL: max_scan     # maximum of all subsequences of a sequence
//////////////////////////////////////////////////////////////////////

struct test_205_process_wf {
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::scan(vw1, vw2, max_int_wf(), false);
  }
};

struct test_205_check_wf {
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 const &vw1, View2 const &vw2)
  {
    int size = vw1.size();
    vw2[0] = vw1[0];
    for (int i = 1; i < size; i++) {
      //vw2[i] = std::max( vw1[i], vw2[i-1] );
      vw2[i] = max_int( vw1[i], vw2[i-1] );
    }
  }
};

bool test_205( size_t model ) {

  // build data structures

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();

  vec_sz_tp p(size), q(size);
  vec_sz_vw_tp p_vw(p), q_vw(q);

  stapl::map_func(roll_wf(), p_vw, stapl::make_repeat_view(limit));
  stapl::rmi_fence();

  vec_vec_int_tp aa(p_vw), bb(p_vw), cc(p_vw);
  vec_vec_int_vw_tp aa_vw(aa), bb_vw(bb), cc_vw(cc);

  int len = 0;
  for ( size_t i= 0; i<p_vw.size(); i++ ) {
    len += p_vw[i];
  }

  vec_int_tp a(len), b(len), c(len);
  vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( a_vw, 0 );

  // create a segmented view over the source

  // subtract 1 from the size because splitter_partition adds an element
#ifdef PARTN_BUG
  // the wrong way: STAPL defect forces this implementation
  std::vector<size_t> stdq(p.size()-1);
  for (size_t i = 0; i<p.size()-1; i++ ) {
    stdq[i] = p.get_element(i);
  }
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), stdq, true) );
#else
  // the right way
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), p, true) );
#endif

  // copy from the segmented view of the source to the nested sequence

  assert( seg_vw.size() == aa_vw.size() );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, aa_vw );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, cc_vw );

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  stapl::map_func(test_205_process_wf(), aa_vw, bb_vw );

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  stapl::serial(test_205_check_wf(), aa_vw, cc_vw );

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

#if 0
  return check_container<vec_int_tp>( "205", "STAPL", "manual",
                                      time1, time2, 0, 0, a, c );
#else
  if ( stapl::get_location_id() == 0 ) {
    cerr << "Test 205 PASS" << endl;
  }
  return true;
#endif

}

//////////////////////////////////////////////////////////////////////
// FEATURES: all_of
// NESL: all(a)     # true if any values in a sequence are true
//////////////////////////////////////////////////////////////////////

struct test_206_process_wf {
  typedef int result_type;

  template <typename View1>
  int operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), and_int_wf(), vw1);
  }
};

struct test_206_check_wf {
  typedef int result_type;

  template <typename View1>
  int operator()(View1 const &vw1)
  {
    int id_val = stapl::identity_value<stapl::logical_and<int>, int>::value();
    int red = id_val;
    for (typename View1::size_type elem = 0; elem < vw1.size(); elem++) {
      red &= vw1[elem];
    }
    return red;
  }
};

bool test_206( size_t model ) {

  // build data structures

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();

  vec_sz_tp p(size), q(size);
  vec_sz_vw_tp p_vw(p), q_vw(q);

  stapl::map_func(roll_wf(), p_vw, stapl::make_repeat_view(limit));
  stapl::rmi_fence();

  vec_vec_int_tp aa(p_vw), bb(p_vw), cc(p_vw);
  vec_vec_int_vw_tp aa_vw(aa), bb_vw(bb), cc_vw(cc);

  int len = 0;
  for ( size_t i= 0; i<p_vw.size(); i++ ) {
    len += p_vw[i];
  }

  vec_int_tp a(len), b(len), c(len);
  vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( a_vw, 0 );

  // create a segmented view over the source

  // subtract 1 from the size because splitter_partition adds an element
#ifdef PARTN_BUG
  // the wrong way: STAPL defect forces this implementation
  std::vector<size_t> stdq(p.size()-1);
  for (size_t i = 0; i<p.size()-1; i++ ) {
    stdq[i] = p.get_element(i);
  }
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), stdq, true) );
#else
  // the right way
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), p, true) );
#endif

  // copy from the segmented view of the source to the nested sequence

  assert( seg_vw.size() == aa_vw.size() );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, aa_vw );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, cc_vw );

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  stapl::map_func(test_206_process_wf(), aa_vw );

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  stapl::serial(test_206_check_wf(), aa_vw );

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

#if 0
  return check_container<vec_int_tp>( "206", "STAPL", "manual",
                                      time1, time2, 0, 0, a, c );
#else
  if ( stapl::get_location_id() == 0 ) {
    cerr << "Test 206 PASS" << endl;
  }
  return true;
#endif

}

//////////////////////////////////////////////////////////////////////
// FEATURES: scan
// NESL: or_scan     # logical or of all subsequences of a sequence
//////////////////////////////////////////////////////////////////////

struct test_207_process_wf {
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::scan(vw1, vw2, or_int_wf(), false);
  }
};

struct test_207_check_wf {
  typedef void result_type;

  template <typename View1, typename View2>
  void operator()(View1 const &vw1, View2 const &vw2)
  {
    int size = vw1.size();
    vw2[0] = vw1[0];
    for (int i = 1; i < size; i++) {
      vw2[i] = vw1[i] | vw2[i-1];
    }
  }
};

bool test_207( size_t model ) {

  // build data structures

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();

  vec_sz_tp p(size), q(size);
  vec_sz_vw_tp p_vw(p), q_vw(q);

  stapl::map_func(roll_wf(), p_vw, stapl::make_repeat_view(limit));
  stapl::rmi_fence();

  vec_vec_int_tp aa(p_vw), bb(p_vw), cc(p_vw);
  vec_vec_int_vw_tp aa_vw(aa), bb_vw(bb), cc_vw(cc);

  int len = 0;
  for ( size_t i= 0; i<p_vw.size(); i++ ) {
    len += p_vw[i];
  }

  vec_int_tp a(len), b(len), c(len);
  vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( a_vw, 0 );

  // create a segmented view over the source

  // subtract 1 from the size because splitter_partition adds an element
#ifdef PARTN_BUG
  // the wrong way: STAPL defect forces this implementation
  std::vector<size_t> stdq(p.size()-1);
  for (size_t i = 0; i<p.size()-1; i++ ) {
    stdq[i] = p.get_element(i);
  }
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), stdq, true) );
#else
  // the right way
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), p, true) );
#endif

  // copy from the segmented view of the source to the nested sequence

  assert( seg_vw.size() == aa_vw.size() );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, aa_vw );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, cc_vw );

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  stapl::map_func(test_207_process_wf(), aa_vw, bb_vw );

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  stapl::serial(test_207_check_wf(), aa_vw, cc_vw );

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

#if 0
  return check_container<vec_int_tp>( "207", "STAPL", "manual",
                                      time1, time2, 0, 0, a, c );
#else
  if ( stapl::get_location_id() == 0 ) {
    cerr << "Test 207 PASS" << endl;
  }
  return true;
#endif

}

/*=========================================================================
 * STRUCTURAL MANIPULATION OF NESTED SEQUENCES
 *=========================================================================*/

//////////////////////////////////////////////////////////////////////
// ## BASE ##
// FEATURES:
// NESL: split(a,f)    # split a into nested sequence using flags f
//////////////////////////////////////////////////////////////////////

extern int prime_nums[];
extern int prime_flags[], non_prime_flags[];

bool test_301( size_t model ) {

  stapl::counter<stapl::default_timer> ctr;

  // build data structures

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();

  vec_int_tp prime_flag( size );
  vec_int_vw_tp prime_flag_vw( prime_flag );
  vec_int_tp non_prime_flag( size );
  vec_int_vw_tp non_prime_flag_vw( non_prime_flag );

  if ( size <= 100000 ) {
    for ( size_t i=0; i<size; i++ ) {
      prime_flag[i] = prime_flags[i];
      non_prime_flag[i] = 1 - prime_flag[i];
    }
  } else {
    for ( size_t i=0; i<size; i++ ) {
      prime_flag[i] = prime_flags[i%100000];
      non_prime_flag[i] = 1 - prime_flag[i];
    }
  }

  vec_sz_tp p(2), q(2);
  vec_sz_vw_tp p_vw(p), q_vw(q);

  // HERE
#if 0

  vec_vec_int_tp aa(p_vw), bb(p_vw), cc(p_vw);
  vec_vec_int_vw_tp aa_vw(aa), bb_vw(bb), cc_vw(cc);

  vec_int_tp x(size), y(size), z(size);
  vec_int_vw_tp x_vw(x), y_vw(y), z_vw(z);

  stapl::iota( a_vw, 0 );

  // algorithm

  ctr.start();

  vec_int_vw_tp::iterator::difference_type alg_int;
  alg_int = stapl::count_if(prime_flag_vw,
                              bind(stapl::equal_to<int>(),_1,1) );
  int true_cnt = (int)alg_int;
  int false_cnt = prime_flag_vw.size() - true_cnt;

  int flag = 1;
  size_t count = stapl::nd_partition<vec_int_vw_tp,vec_int_vw_tp,vec_int_vw_tp,
                        eq_int_wf>( stapl::counting_view<int>(size),
                        prime_flag_vw, x_vw, eq_int_wf());

  // create a segmented view over the source

#ifdef PARTN_BUG
  // the wrong way: STAPL defect forces this implementation
  std::vector<size_t> stdq(1);
  stdq[0] = count;
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), stdq, true) );
#else
  // the right way
  seg_vw_tp seg_vw( a_vw, seg_splitter(a_vw.domain(), p, true) );
#endif

  // copy from the segmented view of the source to the nested sequence

  assert( seg_vw.size() == aa_vw.size() );
  stapl::map_func( copy_flat2nest_wf(), seg_vw, aa_vw );
#endif

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#if 0
  // reference

  ctr.start();

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

#if 0
  return check_container<vec_int_tp>( "301", "STAPL", "manual",
                         time1, time2, 0, 0, x, u );
#else
  if ( stapl::get_location_id() == 0 ) {
    cerr << "Test 301 PASS" << endl;
  }
  return true;
#endif
}

//////////////////////////////////////////////////////////////////////
// ## BASE ##
// FEATURES:
// NESL: bottop(a)     # split into nested sequence
//////////////////////////////////////////////////////////////////////

bool test_302( size_t model ) {

  //return known_fail( "302", "STAPL", "manual",
  //                   "stapl element assignment feature required ", 0 );

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  int left = 0;
  int right = 0;

  vec_int_tp x(left);
  vec_int_vw_tp x_vw(x);

  vec_int_tp y(right);
  vec_int_vw_tp y_vw(y);

  //stapl::map_func(test_302_process_wf(), x_vw, y_vw );

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#if 0
  // reference

  ctr.start();

  vec_int_tp u(left);
  vec_int_vw_tp u_vw(u);

  vec_int_tp v(right);
  vec_int_vw_tp v_vw(v);

  stapl::serial(test_302_check_wf(), u_vw, v_vw );

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

#if 0
  return check_container<vec_int_tp>( "302", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
#else
  if ( stapl::get_location_id() == 0 ) {
    cerr << "Test 302 PASS" << endl;
  }
  return true;
#endif
}

#ifdef NO_STAPL
/*=========================================================================
 * UNIMPLEMENTED FEATURES
 *=========================================================================*/

//////////////////////////////////////////////////////////////////////
// FEATURES:
// NESL: pack(v,f)     # pack the values in v corresponding to T in f
//////////////////////////////////////////////////////////////////////

bool test_401(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  return known_fail( "401", "STAPL", "manual",
                     "stapl::gather() feature required ", 0 );

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "401", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// NESL: permute(a,i) # permute elements of a to positions i
//////////////////////////////////////////////////////////////////////

bool test_402(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  return known_fail( "402", "STAPL", "manual",
                     "stapl::gather() feature required ", 0 );

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();


  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();


  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "402", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// NESL: d <- a       # write elements a in d
//////////////////////////////////////////////////////////////////////

bool test_403(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  return known_fail( "403", "STAPL", "manual",
                     "stapl::scatter() feature required ", 0 );

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();


  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();


  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "403", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// NESL: a -> i       # read from sequence a using indices i
//////////////////////////////////////////////////////////////////////

bool test_404(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  return known_fail( "404", "STAPL", "manual",
                     "stapl::gather() feature required ", 0 );

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();


  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();


  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "404", "STAPL", "manual",
                                          time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:  STAPL does not have a grade or rank algorithm
// NESL: rank(a)        # rank if sequence were sorted
//////////////////////////////////////////////////////////////////////

bool test_405(
     vec_int_tp &a, vec_int_tp &b, vec_int_tp &c, vec_int_tp &d,
     vec_int_tp &e, vec_int_tp &f, vec_int_tp &g, vec_int_tp &h,
     vec_sz_tp &p, vec_sz_tp &q,
     vec_int_vw_tp& a_vw, vec_int_vw_tp& b_vw,
     vec_int_vw_tp& c_vw, vec_int_vw_tp& d_vw,
     vec_int_vw_tp& e_vw, vec_int_vw_tp& f_vw,
     vec_int_vw_tp& g_vw, vec_int_vw_tp& h_vw,
     vec_sz_vw_tp& p_vw, vec_sz_vw_tp& q_vw)
{
  return known_fail( "405", "STAPL", "manual",
                     "stapl doesn't have a grade or rank algorithm", 0 );

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_container<vec_int_tp>( "405", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
}

//////////////////////////////////////////////////////////////////////
// ## PAIR ##
// FEATURES:
// NESL: zip(a,b)      # zip 2 sequences into 1 sequence of pairs
//////////////////////////////////////////////////////////////////////

bool test_406( size_t model ) {

  return known_fail( "406", "STAPL", "manual",
                     "stapl has zip_view ", 0 );

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();

  vec_sz_tp p(size), q(size);
  vec_sz_vw_tp p_vw(p), q_vw(q);

  stapl::map_func(roll_wf(), p_vw, stapl::make_repeat_view(limit));
  stapl::rmi_fence();

  vec_int_tp a(size), b(size), c(size), d(size);
  vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c), d_vw(d);

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

#ifdef NESTED
#endif

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

#ifdef NESTED
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

#if 0
  return check_container<vec_int_tp>( "406", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
#endif
}

//////////////////////////////////////////////////////////////////////
// ## PAIR ##
// FEATURES:
// NESL: unzip(a)      # unzip sequence of pairs into pair of sequences
//////////////////////////////////////////////////////////////////////

bool test_407( size_t model ) {

  return known_fail( "406", "STAPL", "manual",
                     "stapl has zip_view ", 0 );

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();

  vec_sz_tp p(size), q(size);
  vec_sz_vw_tp p_vw(p), q_vw(q);

  stapl::map_func(roll_wf(), p_vw, stapl::make_repeat_view(limit));
  stapl::rmi_fence();

  vec_int_tp a(size), b(size), c(size), d(size);
  vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c), d_vw(d);

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

#ifdef NESTED
#endif

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

  // reference

  ctr.start();

#ifdef NESTED
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

#if 0
  return check_container<vec_int_tp>( "407", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
#endif
}

//////////////////////////////////////////////////////////////////////
// ## BASE ##
// FEATURES:
// NESL: transpose(a)   # transpose nested sequence
//////////////////////////////////////////////////////////////////////

bool test_408( size_t model ) {

  return known_fail( "408", "STAPL", "manual",
                     "stapl element assignment feature required ", 0 );

  size_t size = 100 * model;
  size_t limit = 100 * model;
  set_random_seed();

  vec_int_tp a(size*limit), b(size*limit);
  vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::iota( a_vw, 0 );

  vec_sz_tp p(size), q(size);
  vec_sz_vw_tp p_vw(p), q_vw(q);

  stapl::map_func(roll_wf(), p_vw, stapl::make_repeat_view(limit));
  stapl::rmi_fence();

  vec_vec_int_tp aa(p_vw), bb(p_vw);
  vec_vec_int_vw_tp aa_vw(aa), bb_vw(bb);

  stapl::counter<stapl::default_timer> ctr;

  // algorithm

  ctr.start();

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#if 0
  // reference

  ctr.start();

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

#if 0
  return check_container<vec_int_tp>( "408", "STAPL", "manual",
                         time1, time2, 0, 0, g, h );
#else
  if ( stapl::get_location_id() == 0 ) {
    cerr << "Test 408 PASS" << endl;
  }
  return true;
#endif
}

#endif // NO_STAPL

/*=========================================================================*/

struct plus_red_seg
{
  typedef void result_type;

  //int id_val = stapl::identity_value<stapl::plus<int>, int>::value();

  template <typename SegRef, typename Reference>
  void operator()(SegRef seg, Reference res)
  {
    int sum = 0;
    typename SegRef::iterator iter;
    for ( iter = seg.begin(); iter != seg.end(); ++iter) {
      if (iter == seg.begin()) {
        sum = *iter;
      } else {
        sum += *iter;
      }
    }
  }
};

struct plus_scan_seg
{
  typedef void result_type;

  //int id_val = stapl::identity_value<stapl::plus<int>, int>::value();

  template <typename SegRef, typename IndexRef>
  void operator()(SegRef seg, IndexRef i)
  {
    int sum = 0;
    typename SegRef::iterator iter;
    for ( iter = seg.begin(); iter != seg.end(); ++iter) {
      if (iter == seg.begin()) {
        sum = 0;
      } else {
        sum += *iter;
      }
    }
  }
};

#if 0
//#define PARTN_BUG 1
typedef stapl::splitter_partition<vec_int_vw_tp::domain_type> seg_splitter;
typedef stapl::segmented_view<vec_int_vw_tp, seg_splitter> seg_vw;
#ifdef PARTN_BUG
  // the wrong way: STAPL defect forces this implementation
  std::vector<size_t> stdq(q.size());
  for (int i = 0; i<q.size(); i++ ) {
    stdq.push_back( q.get_element(i) );
  }
  seg_vw segments( a_vw, seg_splitter(a_vw.domain(), stdq, true) );
#else
  // the right way
  seg_vw segments( a_vw, seg_splitter(a_vw.domain(), q, true) );
#endif
#endif
