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
#include <iostream>

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/views/multiarray_view.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include <stapl/runtime.hpp>

#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include "json_parser.hpp"

#include "testutil.hpp"
#include "rel_alpha_data.h"

using namespace std;

//////////////////////////////////////////////////////////////////////

typedef stapl::negate<int> ineg_wf;
typedef stapl::max<int> imax_wf;
typedef stapl::identity<int> iid_wf;
typedef stapl::plus<int> iadd_wf;
typedef stapl::min<int> imin_wf;
typedef stapl::logical_or<int> ior_wf;
typedef stapl::minus<int> isub_wf;
typedef stapl::multiplies<int> imul_wf;

//////////////////////////////////////////////////////////////////////

typedef stapl::indexed_domain<size_t>              vec_dom_tp;
typedef stapl::balanced_partition<vec_dom_tp>      bal_part_tp;
typedef stapl::default_traversal<3>::type          trav3_tp;
typedef stapl::nd_partition<
          stapl::tuple<bal_part_tp, bal_part_tp, bal_part_tp>,
          trav3_tp>                                part3_tp;
typedef stapl::tuple<size_t, size_t, size_t>       gid_tp;
typedef stapl::multiarray<3, int, trav3_tp,
          part3_tp>                                cube_int_tp;

typedef stapl::multiarray_view<cube_int_tp>        cube_int_view_tp;
typedef cube_int_view_tp::domain_type              dom_tp;

//////////////////////////////////////////////////////////////////////

void init_cube( cube_int_tp &, cube_int_tp &,
                cube_int_tp &, cube_int_tp &,
                cube_int_tp &, cube_int_tp &,
                cube_int_tp &, cube_int_tp &,
                cube_int_tp &, cube_int_tp &, size_t, size_t *);

bool test_001( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_002( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_003( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_004( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_005( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_006( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_007( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_008( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_009( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_010( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_011( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_012( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_013( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_014( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_015( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_016( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_017( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_018( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_019( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_020( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_021( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_022( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_023( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_024( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_025( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_026( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_027( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);
bool test_028( cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &, cube_int_tp &, cube_int_tp &,
                  cube_int_tp &, cube_int_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &,
                  cube_int_view_tp &, cube_int_view_tp &, size_t *);

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
    model = 2;
    break;
  case 's':
    model = 4;
    break;
  case 'm':
    model = 8;
    break;
  case 'b':
    model = 16;
    break;
  case 'h':
    model = 32;
    break;
  default:
    cerr << "opt_data " << opt_data << endl;
    break;
  }
  if (model == -1) {
    std::cerr << "usage: exe -data tiny/small/medium/big/huge\n";
    exit(1);
  }
  int size = 10 * model;

  int first_test = 1;
  int last_test = 50;
  if ( opt_test != -1 ) {
    first_test = opt_test;
    last_test = opt_test;
  }

  size_t n = size;
  size_t m = size;
  size_t p = size;
  size_t shape[3];
  shape[0] = n;
  shape[1] = m;
  shape[2] = p;

  bal_part_tp p0(vec_dom_tp(0, n-1), stapl::get_num_locations());
  bal_part_tp p1(vec_dom_tp(0, m-1), stapl::get_num_locations());
  bal_part_tp p2(vec_dom_tp(0, p-1), stapl::get_num_locations());
  part3_tp part(p0,p1,p2);

  cube_int_tp a3(gid_tp(n,m,p));
  cube_int_tp b3(gid_tp(n,m,p));
  cube_int_tp c3(gid_tp(n,m,p));
  cube_int_tp d3(gid_tp(n,m,p));
  cube_int_tp e3(gid_tp(n,m,p));
  cube_int_tp f3(part);
  cube_int_tp g3(part);
  cube_int_tp h3(part);
  cube_int_tp p3(part);
  cube_int_tp q3(part);

  cube_int_view_tp a3_vw(a3), b3_vw(b3), c3_vw(c3), d3_vw(d3);
  cube_int_view_tp e3_vw(e3), f3_vw(f3), g3_vw(g3), h3_vw(h3);
  cube_int_view_tp p3_vw(p3), q3_vw(q3);

  init_cube( a3, b3, c3, d3, e3, f3, g3, h3, p3, q3, model, shape);

  int fail_count = 0;
  bool ok = true;
  for (int test = first_test; test <= last_test; test++ ) {
    cerr << "\n";
    switch (test) {
    case 1:
      ok = test_001(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 2:
      ok = test_002(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 3:
      ok = test_003(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 4:
      ok = test_004(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 5:
      ok = test_005(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 6:
      ok = test_006(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 7:
      ok = test_007(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 8:
      ok = test_008(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 9:
      ok = test_009(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 10:
      ok = test_010(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 11:
      ok = test_011(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 12:
      ok = test_012(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 13:
      ok = test_013(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 14:
      ok = test_014(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 15:
      ok = test_015(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 16:
      ok = test_016(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 17:
      ok = test_017(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 18:
      ok = test_018(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 19:
      ok = test_019(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 20:
      ok = test_020(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 21:
      ok = test_021(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 22:
      ok = test_022(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 23:
      ok = test_023(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 24:
      ok = test_024(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 25:
      ok = test_025(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 26:
      ok = test_026(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 27:
      ok = test_027(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    case 28:
      ok = test_028(a3, b3, c3, d3, e3, f3, g3, h3, p3, q3,
           a3_vw,b3_vw,c3_vw,d3_vw,e3_vw,f3_vw,g3_vw,h3_vw,p3_vw,q3_vw, shape);
      break;
    }
  }
  return EXIT_SUCCESS;
}

//////////////////////////////////////////////////////////////////////
// initialize integer arrays
//////////////////////////////////////////////////////////////////////

void init_cube( cube_int_tp & a, cube_int_tp & b,
               cube_int_tp & c, cube_int_tp & d,
               cube_int_tp & e, cube_int_tp & f,
               cube_int_tp & g, cube_int_tp & h,
               cube_int_tp & p, cube_int_tp & q, size_t model, size_t *shape) {

  for (size_t i = 0; i < model; i++) {
    for (size_t j = 0; j < 1000; j++) {
#if 0
      a[i*1000+j] = prime1000[rand1000_01[j]]; // integer in
      b[i*1000+j] = prime1000[rand1000_02[j]]; // integer in
      c[i*1000+j] = prime1000[rand1000_03[j]] % 2; // bool in
      d[i*1000+j] = prime1000[rand1000_04[j]] % 2; // bool in
      e[i*1000+j] = prime1000[rand1000_05[j] % 100 ]; // dup integer in
      f[i*1000+j] = prime1000[rand1000_06[j] % 100 ]; // dup integer in
      g[i*1000+j] = 0; // temp/output
      h[i*1000+j] = 0; // temp/output
      p[i*1000+j] = 0; // temp/output
      q[i*1000+j] = 0; // temp/output
#endif
    }
  }
}

//////////////////////////////////////////////////////////////////////
// FEATURES: fill_n
// constant assignment -  APL: A is con
//////////////////////////////////////////////////////////////////////

bool test_001( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  int x = 17, y = 42, z = 99;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  auto g_ln_vw = linear_view(g_vw);
  stapl::fill(g_ln_vw, x);

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

  for (size_t i = 0; i < shape[0]; i++) {
    for (size_t j = 0; j < shape[1]; j++) {
      for (size_t k = 0; k < shape[2]; k++) {
        gid_tp gid(i,j,k);
        h.set_element(gid, x);
      }
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "001", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: generate_n, sequence
// constant assignment -  APL: A is A mul iota B
//////////////////////////////////////////////////////////////////////

bool test_002( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int base = 10;

  auto g_ln_vw = linear_view(g_vw);
  typedef stapl::sequence<int> step_wf;
  stapl::generate(g_ln_vw, step_wf(base));

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

  for (size_t i = 0; i < shape[0]; i++) {
    for (size_t j = 0; j < shape[1]; j++) {
      for (size_t k = 0; k < shape[2]; k++) {
        gid_tp gid(i,j,k);
        h.set_element(gid, i+base);
      }
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "002", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: transform unary
// apply unary function to each scalar -  APL: A scalar-monadic B
//////////////////////////////////////////////////////////////////////

bool test_003( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  auto a_ln_vw = linear_view(a_vw);
  auto g_ln_vw = linear_view(g_vw);
  stapl::transform(a_ln_vw, g_ln_vw, ineg_wf());

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

  for (size_t i = 0; i < shape[0]; i++) {
    for (size_t j = 0; j < shape[1]; j++) {
      for (size_t k = 0; k < shape[2]; k++) {
        gid_tp gid(i,j,k);
        h.set_element(gid, -a.get_element(gid));
      }
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "003", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: transform binary
// apply binary function to each scalar -  APL: A scalar-dyadic B
//////////////////////////////////////////////////////////////////////

bool test_004( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  auto a_ln_vw = linear_view(a_vw);
  auto b_ln_vw = linear_view(b_vw);
  auto g_ln_vw = linear_view(g_vw);
  stapl::transform(a_ln_vw,b_ln_vw, g_ln_vw, imax_wf());

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

  for (size_t i = 0; i < shape[0]; i++) {
    for (size_t j = 0; j < shape[1]; j++) {
      for (size_t k = 0; k < shape[2]; k++) {
        gid_tp gid(i,j,k);
        h.set_element(gid, max( a.get_element(gid), b.get_element(gid) ));
      }
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "004", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: accumulate
// arithmetic reduction -  APL: plus reduce [0] A
//////////////////////////////////////////////////////////////////////

bool test_005( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USE_ALGO
  int alg_int = stapl::accumulate(a_vw,0);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#if USE_CODED
  //int ref_int = 0;
  //for (int i = 0; i < size; i++) {
  //  ref_int += a[i];
  //}
  for (size_t i = 0; i < size; i++) {
    for (size_t j = 0; j < size; j++) {
      for (size_t k = 1; k < size; k++) {
        gid_tp curr(i,j,k);
        h.set_element(curr, h.get_element(prev) + a.get_element(curr));
      }
    }
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "005", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: max_value
// arithmetic reduction -  APL: max reduce [2] A
//////////////////////////////////////////////////////////////////////

bool test_006( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USE_ALGO
  int alg_int = stapl::max_value(b_vw);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
#define max_val(x,y) ((x>y)?(x):(y))
  //int ref_int = numeric_limits<int>::min();
  //for (int i = 0; i < size; i++) {
  //  ref_int = max_val(ref_int,b[i]);
  //}
  for (size_t i = 0; i < size; i++) {
    for (size_t j = 0; j < size; j++) {
      for (size_t k = 1; k < size; k++) {
        gid_tp curr(i,j,k);
        h.set_element(curr, h.get_element(prev) + a.get_element(curr));
      }
    }
  }

#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "006", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: all_of
// logical reduction -  APL: and reduce [1] A
//////////////////////////////////////////////////////////////////////

bool test_007( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();


  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  //bool ref_bool = true;
  //for (int i = 0; i < size; i++) {
  //  ref_bool = ref_bool & c[i];
  //}
  for (size_t i = 0; i < size; i++) {
    for (size_t j = 0; j < size; j++) {
      for (size_t k = 1; k < size; k++) {
        gid_tp curr(i,j,k);
        h.set_element(curr, h.get_element(prev) + a.get_element(curr));
      }
    }
  }

#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "007", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: scan
// arithmetic scan -  APL: plus scan [2] A
//////////////////////////////////////////////////////////////////////

bool test_008( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USE_ALGO
  int final = stapl::accumulate(a_vw, 0, iadd_wf());
  stapl::scan(a_vw, g_vw, iadd_wf());
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

  for (size_t i = 0; i < shape[0]; i++) {
    for (size_t j = 0; j < shape[1]; j++) {
      gid_tp first(i,j,0);
      h.set_element(first, a.get_element(first));
      for (size_t k = 1; k < shape[2]; k++) {
        gid_tp curr(i,j,k);
        gid_tp prev(i,j,k-1);
        h.set_element(curr, h.get_element(prev) + a.get_element(curr));
      }
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "008", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: scan
// arithmetic scan -  APL min scan [0] A
//////////////////////////////////////////////////////////////////////

bool test_009( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USE_ALGO
  int final = stapl::accumulate(b_vw, 0, imin_wf());
  stapl::scan(b_vw, g_vw, imin_wf(), false);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

  for (size_t i = 0; i < shape[0]; i++) {
    for (size_t j = 0; j < shape[1]; j++) {
      gid_tp first(i,j,0);
      h.set_element(first, a.get_element(first));
      for (size_t k = 1; k < shape[2]; k++) {
        gid_tp curr(i,j,k);
        gid_tp prev(i,j,k-1);
        h.set_element(curr, min(h.get_element(prev), b.get_element(curr)));
      }
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "009", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: scan
// logical scan -  APL: or scan [1] A
//////////////////////////////////////////////////////////////////////

bool test_010( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USE_ALGO
  stapl::scan(d_vw, g_vw, ior_wf());
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

  for (size_t i = 0; i < shape[0]; i++) {
    for (size_t j = 0; j < shape[1]; j++) {
      gid_tp first(i,j,0);
      h.set_element(first, a.get_element(first));
      for (size_t k = 1; k < shape[2]; k++) {
        gid_tp curr(i,j,k);
        gid_tp prev(i,j,k-1);
        h.set_element(curr, h.get_element(prev) | d.get_element(curr));
      }
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "010", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// adjacent delta - APL: 0 chain ((-1 drop A) sub (1 drop A)))
//////////////////////////////////////////////////////////////////////

bool test_011( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USE_ALGO
  stapl::adjacent_difference(b_vw, g_vw);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  //for (int i = 1; i < size; i++) {
  //  h[i-1] = b[i] - b[i-1];
  //}
  for (size_t i = 0; i < shape[0]; i++) {
    for (size_t j = 0; j < shape[1]; j++) {
      gid_tp first(i,j,0);
      h.set_element(first, 0);
      for (size_t k = 1; k < shape[2]; k++) {
        gid_tp curr(i,j,k);
        gid_tp prev(i,j,k-1);
        h.set_element(curr, h.get_element(prev) | d.get_element(curr));
      }
    }
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "011", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// adjacent delta - APL: 0 splice ((-1 drop A) sub (1 drop A)))
//////////////////////////////////////////////////////////////////////

bool test_012( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USE_ALGO
  stapl::adjacent_difference(b_vw, g_vw);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  //for (int i = 1; i < size; i++) {
  //  h[i-1] = b[i] - b[i-1];
  //}
  for (size_t i = 0; i < shape[0]; i++) {
    for (size_t j = 0; j < shape[1]; j++) {
      gid_tp first(i,j,0);
      h.set_element(first, 0);
      for (size_t k = 1; k < shape[2]; k++) {
        gid_tp curr(i,j,k);
        gid_tp prev(i,j,k-1);
        h.set_element(curr, h.get_element(prev) | d.get_element(curr));
      }
    }
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "012", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: inner_product
// inner product -  APL: A op1 innerprod op2 B
//////////////////////////////////////////////////////////////////////

bool test_013( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();


  ctr.stop();
  double time1 = ctr.value();

  ctr.start();


  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// APL: A chain [0] B
//////////////////////////////////////////////////////////////////////

bool test_014( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();


  ctr.stop();
  double time1 = ctr.value();

  ctr.start();


  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// APL: A chain [0] B
//////////////////////////////////////////////////////////////////////

bool test_015( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();


  ctr.stop();
  double time1 = ctr.value();

  ctr.start();


  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

}

//////////////////////////////////////////////////////////////////////
// FEATURES: reverse_copy
// reverse -  APL: reverse [0] A
//////////////////////////////////////////////////////////////////////

bool test_016( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  return known_fail( "016", "STAPL", "manual",
                     "stapl::reverse_copy() aborts at compile time", 0 );

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USE_ALGO
  stapl::reverse_copy(a_vw, b_vw);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  int j = size - 1;
  for (int i = 0; i < size; i++) {
    h[j] = c[i];
    --j;
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "016", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: reverse_copy
// reverse -  APL: reverse [2] A
//////////////////////////////////////////////////////////////////////

bool test_017( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  return known_fail( "017", "STAPL", "manual",
                     "stapl::reverse_copy() aborts at compile time", 0 );

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USE_ALGO
  stapl::reverse_copy(a_vw, b_vw);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  int j = size - 1;
  for (int i = 0; i < size; i++) {
    h[j] = c[i];
    --j;
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "017", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: rotate_copy
// rotate elements -  APL: k rotate [0] A
//////////////////////////////////////////////////////////////////////

bool test_018( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 97;
#ifdef USE_ALGO
  stapl::rotate_copy(b_vw, g_vw, n);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  int j = 0;
  for (int i = n; i < size; i++) {
    h[j++] = b[i];
  }
  for (int i = 0; i <n; i++) {
    h[j++] = b[i];
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "018", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: rotate_copy
// rotate elements -  APL: k rotate [2] A
//////////////////////////////////////////////////////////////////////

bool test_019( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 97;
#ifdef USE_ALGO
  stapl::rotate_copy(b_vw, g_vw, n);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  int j = 0;
  for (int i = n; i < size; i++) {
    h[j++] = b[i];
  }
  for (int i = 0; i <n; i++) {
    h[j++] = b[i];
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "019", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// take head -  APL: n take [0] A
//////////////////////////////////////////////////////////////////////

bool test_020( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 97;

#ifdef USE_ALGO
  // input domains and views
  dom_tp b_dom = b_vw.domain();
  dom_tp bpre_dom( 0, n-1, b_dom);
  cube_int_view_tp bpre_vw(b_vw.container(), bpre_dom);

  // output domains and views
  dom_tp g_dom = g_vw.domain();
  dom_tp gpre_dom( 0, n-1, g_dom);
  dom_tp gpost_dom( n, size-1, g_dom);
  cube_int_view_tp gpre_vw(g_vw.container(), gpre_dom);
  cube_int_view_tp gpost_vw(g_vw.container(), gpost_dom);

  stapl::copy(bpre_vw, gpre_vw);
  stapl::fill_n(gpost_vw, -12345, size-n);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  int j = 0;
  for (int i = 0; i <=n-1; i++) {
    h[j++] = b[i];
  }
  for (int i = n; i <=size-1; i++) {
    h[j++] = -12345;
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "020", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// take head -  APL: n take [2] A
//////////////////////////////////////////////////////////////////////

bool test_021( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 97;

#ifdef USE_ALGO
  // input domains and views
  dom_tp b_dom = b_vw.domain();
  dom_tp bpost_dom( size-n, size-1, b_dom);
  cube_int_view_tp bpost_vw(b_vw.container(), bpost_dom);

  // output domains and views
  dom_tp g_dom = g_vw.domain();
  dom_tp gpre_dom( 0, n-1, g_dom);
  dom_tp gpost_dom( n, size-1, g_dom);
  cube_int_view_tp gpre_vw(g_vw.container(), gpre_dom);
  cube_int_view_tp gpost_vw(g_vw.container(), gpost_dom);

  int bcount = 1 + (bpost_dom.last() - bpost_dom.first());
  int gcount = 1 + (gpre_dom.last() - gpre_dom.first());
  assert( bcount == gcount );
  assert( bcount == n );

  stapl::copy(bpost_vw, gpre_vw);
  stapl::fill_n(gpost_vw, -12345, size-n);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  int j = 0;
  for (int i = size-n; i < size; i++) {
    h[j++] = b[i];
  }
  for (int i = n; i < size; i++) {
    h[j++] = -12345;
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "021", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// drop head -  APL: n drop [0] A
//////////////////////////////////////////////////////////////////////

bool test_022( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 97;

#ifdef USE_ALGO
  // input domains and views
  dom_tp b_dom = b_vw.domain();
  dom_tp bpre_dom( 0, n-1, b_dom); // not used
  dom_tp bpost_dom( n, size-1, b_dom);
  cube_int_view_tp bpost_vw(b_vw.container(), bpost_dom);

  // output domains and views
  dom_tp g_dom = g_vw.domain();
  dom_tp gpre_dom( 0, size-(n+1), g_dom);
  dom_tp gpost_dom( size-n, size-1, g_dom);

  cube_int_view_tp gpre_vw(g_vw.container(), gpre_dom);
  cube_int_view_tp gpost_vw(g_vw.container(), gpost_dom);

  int bcount = 1 + (bpost_dom.last() - bpost_dom.first());
  int gcount = 1 + (gpre_dom.last() - gpre_dom.first());
  assert( bcount == gcount );
  assert( bcount == size - n );

  stapl::copy(bpost_vw, gpre_vw);
  stapl::fill(gpost_vw, -12345);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  int j = 0;
  for (int i = n; i < size; i++) {
    h[j++] = b[i];
  }
  for (int i = 0; i <n; i++) {
    h[j++] = -12345;
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "022", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// drop head -  APL: n drop [2] A
//////////////////////////////////////////////////////////////////////

bool test_023( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 97;

#ifdef USE_ALGO
  // input domains and views
  dom_tp b_dom = b_vw.domain();
  dom_tp bpre_dom( 0, size-(n+1), b_dom);
  dom_tp bpost_dom( size-n, size-1, b_dom); // not used
  cube_int_view_tp bpre_vw(b_vw.container(), bpre_dom);

  // output domains and views
  dom_tp g_dom = g_vw.domain();
  dom_tp gpre_dom( 0, size-(n+1), g_dom);
  dom_tp gpost_dom( size-n, size-1, g_dom);
  cube_int_view_tp gpre_vw(g_vw.container(), gpre_dom);
  cube_int_view_tp gpost_vw(g_vw.container(), gpost_dom);

  int bcount = 1 + bpre_dom.last() - bpre_dom.first();
  int gcount = 1 + gpre_dom.last() - gpre_dom.first();
  assert( bcount == gcount );
  assert( gcount == size - n );

  stapl::copy(bpre_vw, gpre_vw);
  stapl::fill(gpost_vw, -12345);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  int j = 0;
  for (int i = 0; i < size-n; i++) {
    h[j++] = b[i];
  }
  for (int i = 0; i <n; i++) {
    h[j++] = -12345;
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "023", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n, fill_n
// pad array at back -  APL: (size+n) pad [0] A
//////////////////////////////////////////////////////////////////////

bool test_024( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 53;

#ifdef USE_ALGO
  cube_int_tp za(size+n);
  cube_int_view_tp za_vw(za);

  dom_tp za_dom = za_vw.domain();
  dom_tp x_dom( 0, size-1, za_dom);
  dom_tp y_dom( size, size+n, za_dom);

  cube_int_view_tp x_vw(za_vw.container(), x_dom);
  cube_int_view_tp y_vw(za_vw.container(), y_dom);

  stapl::copy(b_vw, x_vw);
  stapl::fill(y_vw, 0);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  cube_int_tp zr(size+n);
  cube_int_view_tp zr_vw(zr);

  int j = 0;
  for (int i = 0; i < size; i++) {
    zr[j++] = b[i];
  }
  for (int i = 0; i <n; i++) {
    zr[j++] = 0;
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "024", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n, fill_n
// pad array at front -  APL: (size+n) pad [2] A
//////////////////////////////////////////////////////////////////////

bool test_025( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 53;

#ifdef USE_ALGO
  cube_int_tp za(size+n);
  cube_int_view_tp za_vw(za);
  cube_int_tp zr(size+n);
  cube_int_view_tp zr_vw(zr);

  dom_tp za_dom = za_vw.domain();
  dom_tp x_dom( 0, n-1, za_dom);
  dom_tp y_dom( n, size+n, za_dom);

  cube_int_view_tp x_vw(za_vw.container(), x_dom);
  cube_int_view_tp y_vw(za_vw.container(), y_dom);

  stapl::fill(x_vw, 0);
  stapl::copy(b_vw, y_vw);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  int j = 0;
  for (int i = 0; i <n; i++) {
    zr[j++] = 0;
  }
  for (int i = 0; i < size; i++) {
    zr[j++] = b[i];
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "025", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: equal
// APL: A match B
//////////////////////////////////////////////////////////////////////

bool test_026( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  bool alg_bool = false, ref_bool = false;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USE_ALGO
  bool alg_bool = stapl::equal(a_vw, b_vw);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

  for (size_t i = 0; i < shape[0]; i++) {
    for (size_t j = 0; j < shape[1]; j++) {
      for (size_t k = 0; k < shape[2]; k++) {
        gid_tp gid(i,j,k);
        if (a.get_element(gid) != b.get_element(gid) ) {
          ref_bool = false;
          break;
        }
      }
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_scalar<bool>( "026", "STAPL", "manual",
                             time1, time2, 0, 0, alg_bool, ref_bool);
}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// APL: transpose A
//////////////////////////////////////////////////////////////////////

bool test_027( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  return known_fail( "027", "STAPL", "manual",
                     "stapl::gather/scatter() feature required", 0 );

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();


  ctr.stop();
  double time1 = ctr.value();

  ctr.start();


  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// APL: A slice B
//////////////////////////////////////////////////////////////////////

bool test_028( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  return known_fail( "028", "STAPL", "manual",
                     "stapl::gather/scatter() feature required", 0 );

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();


  ctr.stop();
  double time1 = ctr.value();

  ctr.start();


  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n, fill_n
// DEFECT: need gather/scatter implementation
// expand elements -  APL: A expand B
//////////////////////////////////////////////////////////////////////

bool test_029( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  return known_fail( "029", "STAPL", "manual",
                     "stapl::gather/scatter() feature required", 0 );

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USE_ALGO
  cube_int_view_tp::iterator::difference_type alg_int;
  alg_int = stapl::count_if (a_vw,
                              bind(stapl::equal_to<int>(),_1,0) );
  int n = (int)alg_int;

  cube_int_tp z(size+n);
  cube_int_view_tp z_vw(z);

  dom_tp b_dom = b_vw.domain();
  dom_tp x_dom( 0, size+n, b_dom);
  cube_int_view_tp x_vw(z_vw.container(), x_dom);

  stapl::fill_n(x_vw, 0, size+n);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  int j = 0;
  for (int i = 0; i <n; i++) {
    if (a[i] == 1 ) {
      h[j++] = b[i];
    } else {
      h[j++] = 0;
    }
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "027", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// DEFECT: need gather/scatter implementation
// replicate -  APL: A repeat B
//////////////////////////////////////////////////////////////////////

bool test_030( cube_int_tp &a, cube_int_tp &b, cube_int_tp &c, cube_int_tp &d,
               cube_int_tp &e, cube_int_tp &f, cube_int_tp &g, cube_int_tp &h,
               cube_int_tp &p, cube_int_tp &q,
               cube_int_view_tp& a_vw, cube_int_view_tp& b_vw,
               cube_int_view_tp& c_vw, cube_int_view_tp& d_vw,
               cube_int_view_tp& e_vw, cube_int_view_tp& f_vw,
               cube_int_view_tp& g_vw, cube_int_view_tp& h_vw,
               cube_int_view_tp& p_vw, cube_int_view_tp& q_vw, size_t *shape) {

  return known_fail( "030", "STAPL", "manual",
                     "stapl::gather/scatter() feature required", 0 );

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef USE_ALGO
  stapl::transform(a_vw, g_vw, bind(stapl::minus<int>(),_1,1) );
  int n = stapl::accumulate(g_vw, 0);

  cube_int_tp x(size+n);
  cube_int_view_tp x_vw(x);

  dom_tp a_dom = a_vw.domain();
  dom_tp z_dom( size, size+n, a_dom);
  cube_int_view_tp z_vw(a_vw.container(), z_dom);
#endif

  ctr.stop();
  double time1 = ctr.value();

  ctr.start();

#ifdef USE_CODED
  int k = 0;
  for (int i = 0; i <n; i++) {
    int temp = a[i];
    for (int j = 0; j<temp; j++) {
      h[k++] = b[i];
    }
  }
#endif

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();

  return check_multiarray<cube_int_tp>( "028", "STAPL", "manual",
                                  time1, time2, 0, 0, g, h, 3, shape );
}
