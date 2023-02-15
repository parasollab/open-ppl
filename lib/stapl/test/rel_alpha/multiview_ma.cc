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

#include <stapl/views/multiarray_view.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>

#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/partitions/explicit.hpp>
#include <stapl/containers/partitions/normal.hpp>
#include <stapl/containers/partitions/overlap.hpp>

#include <stapl/views/strided_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/views/functor_view.hpp>
#include <stapl/views/overlap_view.hpp>

#ifdef EXPLICIT
#include <stapl/domains/indexed.hpp>
#include <stapl/containers/partitions/explicit.hpp>
#include <stapl/views/explicit_view.hpp>
#endif

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

/*=========================================================================*/

template <typename T>
struct plus1
{
  typedef std::size_t index_type;
  typedef T           argument_type;
  typedef int         result_type;

  int operator() (const T& x) const {
    return 1 + x;
  }
};

//////////////////////////////////////////////////////////////////////

bool view_49( size_t model );
bool view_51( size_t model );
bool view_52( size_t model );
bool view_54( size_t model );
bool view_55( size_t model );
bool view_56( size_t model );

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
  size_t size = 1000 * model;

  int first_test = 1;
  int last_test = 100;
  if (opt_test != -1 ) {
    first_test = opt_test;
    last_test = opt_test;
  }

  bool ok = true;
  for (int test = first_test; test <= last_test; test++ ) {
    switch (test) {
    case 49:
      ok = view_49(model);
      break;
    case 51:
      ok = view_51(model);
      break;
    case 52:
      ok = view_52(model);
      break;
    case 54:
      ok = view_54(model);
      break;
    case 55:
      ok = view_55(model);
      break;
    case 56:
      ok = view_56(model);
      break;
    }
  }
  return EXIT_SUCCESS;
}

/*=========================================================================*/

typedef stapl::indexed_domain<size_t>                ndx_dom_tp;
typedef stapl::balanced_partition<ndx_dom_tp>        bal_part_tp;
typedef stapl::default_traversal<3>::type            trav3_tp;
typedef stapl::nd_partition<
          stapl::tuple<bal_part_tp, bal_part_tp, bal_part_tp>,
          trav3_tp>                                  bal3_part_tp;
typedef stapl::tuple<size_t, size_t, size_t>         gid3_tp;
typedef stapl::multiarray<3, int, trav3_tp,
               bal3_part_tp>                         ary3_int_bal_tp;
typedef stapl::multiarray_view<ary3_int_bal_tp>      ary3_int_bal_vw_tp;

typedef stapl::explicit_partition<ndx_dom_tp> expl_part_tp;

///////////////////////////////////////////////////////////////////////////
// multiarray : multiarray view : strided view
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

typedef stapl::strided_view<ary3_int_bal_vw_tp>::type str_ary3_int_vw_tp;

void view_49_build( size_t model,
                    ary3_int_bal_tp a, ary3_int_bal_tp b, ary3_int_bal_tp c )
{
  int size = model * 1000;

  int ndx = 0;
  ary3_int_bal_tp::dimensions_type dims = a.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        a[gid] = prime1000[ rand1000_01[k%1000] ];
        b[gid] = prime1000[ rand1000_02[k%1000] ];
      }
    }
  }

  stapl::rmi_fence();
}

size_t view_49_visit( ary3_int_bal_vw_tp a_vw, ary3_int_bal_vw_tp b_vw )
{
  size_t cksum = 0;

  //auto lin_vw = stapl::linear_view(a_vw); // ok

  ary3_int_bal_vw_tp::dimensions_type dims = a_vw.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        cksum ^= (unsigned int) a_vw.get_element(gid) + b_vw.get_element(gid);
      }
    }
  }

  stapl::rmi_fence();

  return cksum;
}

int view_49_traverse( str_ary3_int_vw_tp a_vw, str_ary3_int_vw_tp b_vw )
{
  int check = 0;

  //auto lin_vw = stapl::linear_view(a_vw); // ok

  str_ary3_int_vw_tp::dimensions_type dims = a_vw.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        check += a_vw.get_element(gid) + b_vw.get_element(gid);
      }
    }
  }

  stapl::rmi_fence();

  return check;
}

bool view_49( size_t model )
{
  size_t num_locs = stapl::get_num_locations();
  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int pages = 10 * model;
  int rows = 10 * model;
  int cols = 10 * model;

  bal_part_tp p0(ndx_dom_tp(0, pages-1), num_locs);
  bal_part_tp p1(ndx_dom_tp(0, rows-1), num_locs);
  bal_part_tp p2(ndx_dom_tp(0, cols-1), num_locs);
  bal3_part_tp bal_part(p0,p1,p2);

  ary3_int_bal_tp a(gid3_tp(pages,rows,cols));
  ary3_int_bal_tp b(gid3_tp(pages,rows,cols));
  ary3_int_bal_tp c(bal_part);

  ary3_int_bal_vw_tp a_vw1(a), b_vw1(b), c_vw1(c);

  view_49_build( model, a, b, c );

  size_t cksum = view_49_visit( a_vw1, b_vw1 );

  str_ary3_int_vw_tp a_vw2 = stapl::make_strided_view(a_vw1,
                             stapl::make_tuple(2,2,2),
                             stapl::make_tuple(0,0,0) );
  str_ary3_int_vw_tp b_vw2 = stapl::make_strided_view(a_vw1,
                             stapl::make_tuple(2,2,1),
                             stapl::make_tuple(0,0,0) );
  str_ary3_int_vw_tp c_vw2 = stapl::make_strided_view(a_vw1,
                             stapl::make_tuple(2,1,1),
                             stapl::make_tuple(0,0,0) );

  int check = view_49_traverse( a_vw2, b_vw2 );

  ctr.stop();
  double time1 = ctr.value();

  if ( 0 ) {
     cerr << "49 strided multiarray PASS" << endl;
  } else {
     cerr << "49 strided multiarray FAIL" << endl;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////
//  reversed multiarray view - no semantics, no test
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
// multiarray : multiarray view : native view
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

typedef
stapl::result_of::native_view<ary3_int_bal_vw_tp>::type nat_ary3_int_vw_tp;

void view_51_build( size_t model,
                    ary3_int_bal_tp a, ary3_int_bal_tp b, ary3_int_bal_tp c )
{
  int size = model * 1000;

  int ndx = 0;
  ary3_int_bal_tp::dimensions_type dims = a.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        a[gid] = prime1000[ rand1000_01[k%1000] ];
        b[gid] = prime1000[ rand1000_02[k%1000] ];
      }
    }
  }

  stapl::rmi_fence();
}

size_t view_51_visit( ary3_int_bal_vw_tp a_vw, ary3_int_bal_vw_tp b_vw )
{
  size_t cksum = 0;

  ary3_int_bal_tp::dimensions_type dims = a_vw.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        cksum ^= (unsigned int) a_vw.get_element(gid) + b_vw.get_element(gid);
      }
    }
  }

  stapl::rmi_fence();

  return cksum;
}

int view_51_traverse( nat_ary3_int_vw_tp a_vw, nat_ary3_int_vw_tp b_vw )
{
  int check = 0;

  nat_ary3_int_vw_tp::iterator a_it;
  for ( a_it = a_vw.begin(); a_it != a_vw.end(); a_it++ ) {
#ifdef NATIVE
    check += *a_it;
#endif
  }

  nat_ary3_int_vw_tp::iterator b_it;
  for ( b_it = b_vw.begin(); b_it != b_vw.end(); b_it++ ) {
#ifdef NATIVE
    check += *b_it;
#endif
  }

  stapl::rmi_fence();

  return check;
}

bool view_51( size_t model )
{
  size_t num_locs = stapl::get_num_locations();
  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int pages = 10 * model;
  int rows = 10 * model;
  int cols = 10 * model;

  bal_part_tp p0(ndx_dom_tp(0, pages-1), num_locs);
  bal_part_tp p1(ndx_dom_tp(0, rows-1), num_locs);
  bal_part_tp p2(ndx_dom_tp(0, cols-1), num_locs);
  bal3_part_tp bal_part(p0,p1,p2);

  ary3_int_bal_tp a(bal_part);
  ary3_int_bal_tp b(bal_part);
  ary3_int_bal_tp c(gid3_tp(pages,rows,cols));

  ary3_int_bal_vw_tp a_vw1(a), b_vw1(b), c_vw1(c);

  view_51_build( model, a, b, c );

  size_t cksum = view_51_visit( a_vw1, b_vw1 );

#ifdef NATIVE
  nat_ary3_int_vw_tp a_vw2 = nat_ary3_int_vw_tp(a_vw1);
  nat_ary3_int_vw_tp b_vw2 = nat_ary3_int_vw_tp(b_vw1);
  nat_ary3_int_vw_tp c_vw2 = nat_ary3_int_vw_tp(c_vw1);

  int check = view_51_traverse( a_vw2, b_vw2 );
#endif

  if ( 0 ) {
     cerr << "51 native multiarray PASS" << endl;
  } else {
     cerr << "51 native multiarray FAIL" << endl;
  }

  ctr.stop();
  double time1 = ctr.value();

  return true;
}

///////////////////////////////////////////////////////////////////////////
// multiarray : multiarray view : overlap view
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

typedef stapl::view_impl::overlap_view_builder<ary3_int_bal_vw_tp>::view_type
  ovl_ary3_int_vw_tp;

void view_52_build( size_t model,
                    ary3_int_bal_tp a, ary3_int_bal_tp b,
                    ary3_int_bal_tp c )
{
  int size = model * 1000;

  int ndx = 0;
  ary3_int_bal_tp::dimensions_type dims = a.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        a[gid] = prime1000[ rand1000_01[k%1000] ];
        b[gid] = prime1000[ rand1000_02[k%1000] ];
      }
    }
  }

  stapl::rmi_fence();
}

size_t view_52_visit( ary3_int_bal_vw_tp a_vw, ary3_int_bal_vw_tp b_vw )
{
  size_t cksum = 0;

  ary3_int_bal_tp::dimensions_type dims = a_vw.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        cksum ^= (unsigned int) a_vw.get_element(gid) + b_vw.get_element(gid);
      }
    }
  }

  stapl::rmi_fence();

  return cksum;
}

int view_52_traverse( ovl_ary3_int_vw_tp a_vw, ovl_ary3_int_vw_tp b_vw )
{
  int check = 0;

#ifdef TRAVERSE
  ovl_ary3_int_vw_tp::dimensions_type dims = a_vw.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        check += a_vw.get_element(gid) + b_vw.get_element(gid);
      }
    }
  }
#endif

  stapl::rmi_fence();

  return check;
}

bool view_52( size_t model )
{
  size_t num_locs = stapl::get_num_locations();
  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int pages = 10 * model;
  int rows = 10 * model;
  int cols = 10 * model;

  bal_part_tp p0(ndx_dom_tp(0, pages-1), num_locs);
  bal_part_tp p1(ndx_dom_tp(0, rows-1), num_locs);
  bal_part_tp p2(ndx_dom_tp(0, cols-1), num_locs);
  bal3_part_tp bal_part(p0,p1,p2);

  ary3_int_bal_tp a(gid3_tp(pages,rows,cols));
  ary3_int_bal_tp b(gid3_tp(pages,rows,cols));
  ary3_int_bal_tp c(gid3_tp(pages,rows,cols));

  ary3_int_bal_vw_tp a_vw1(a), b_vw1(b), c_vw1(c);

  view_52_build( model, a, b, c );

  size_t cksum = view_52_visit( a_vw1, b_vw1 );

  ovl_ary3_int_vw_tp a_vw2 = make_overlap_view(a_vw1,1,0,1);
  ovl_ary3_int_vw_tp b_vw2 = make_overlap_view(b_vw1,1,0,1);
  ovl_ary3_int_vw_tp c_vw2 = make_overlap_view(c_vw1,1,0,1);

  int check = view_52_traverse( a_vw2, b_vw2 );

  ctr.stop();
  double time1 = ctr.value();

  if ( 0 ) {
     cerr << "52 overlap multiarray PASS" << endl;
  } else {
     cerr << "52 overlap multiarray FAIL" << endl;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////
//  repeated multiarray view - no semantics, no test
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
// multiarray : multiarray view : counting view
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

typedef stapl::result_of::counting_view<size_t>::type cnt_ary3_int_vw_tp;

void view_54_build( size_t model,
                    ary3_int_bal_tp a, ary3_int_bal_tp b,
                    ary3_int_bal_tp c )
{
  int size = model * 1000;

  int ndx = 0;
  ary3_int_bal_tp::dimensions_type dims = a.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        a[gid] = prime1000[ rand1000_01[k%1000] ];
        b[gid] = prime1000[ rand1000_02[k%1000] ];
      }
    }
  }

  stapl::rmi_fence();
}

size_t view_54_visit( ary3_int_bal_vw_tp a_vw, ary3_int_bal_vw_tp b_vw )
{
  size_t cksum = 0;

  ary3_int_bal_tp::dimensions_type dims = a_vw.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        cksum ^= (unsigned int) a_vw.get_element(gid) + b_vw.get_element(gid);
      }
    }
  }

  stapl::rmi_fence();

  return cksum;
}

int view_54_traverse( cnt_ary3_int_vw_tp a_vw, cnt_ary3_int_vw_tp b_vw )
{
  int check = 0;

  cnt_ary3_int_vw_tp::iterator a_it;
  for ( a_it = a_vw.begin(); a_it != a_vw.end(); a_it++ ) {
    check += *a_it;
  }

  cnt_ary3_int_vw_tp::iterator b_it;
  for ( b_it = b_vw.begin(); b_it != b_vw.end(); b_it++ ) {
    check += *b_it;
  }

  stapl::rmi_fence();

  return check;
}

bool view_54( size_t model )
{
  size_t num_locs = stapl::get_num_locations();
  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int pages = 10 * model;
  int rows = 10 * model;
  int cols = 10 * model;

  bal_part_tp p0(ndx_dom_tp(0, pages-1), num_locs);
  bal_part_tp p1(ndx_dom_tp(0, rows-1), num_locs);
  bal_part_tp p2(ndx_dom_tp(0, cols-1), num_locs);
  bal3_part_tp bal_part(p0,p1,p2);

  ary3_int_bal_tp a(bal_part);
  ary3_int_bal_tp b(bal_part);
  ary3_int_bal_tp c(bal_part);

  ary3_int_bal_vw_tp a_vw1(a), b_vw1(b), c_vw1(c);

  view_54_build( model, a, b, c );

  size_t cksum = view_54_visit( a_vw1, b_vw1 );

  auto a_lin_vw = stapl::linear_view(a_vw1);
  int count = a_lin_vw.size();

  cnt_ary3_int_vw_tp a_vw2 = stapl::counting_view<size_t>(count);
  cnt_ary3_int_vw_tp b_vw2 = stapl::counting_view<size_t>(count);
  cnt_ary3_int_vw_tp c_vw2 = stapl::counting_view<size_t>(count);

  int check = view_54_traverse( a_vw2, b_vw2 );

  ctr.stop();
  double time1 = ctr.value();

  if ( 0 ) {
     cerr << "54 counted multiarray PASS" << endl;
  } else {
     cerr << "54 counted multiarray FAIL" << endl;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////
// multiarray : multiarray view : explicit view
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

#ifdef EXPLICIT
typedef stapl::result_of::explicit_view<ary3_int_bal_vw_tp> expl_ary3_int_vw_tp;
#endif

void view_55_build( size_t model,
                    ary3_int_bal_tp a, ary3_int_bal_tp b, ary3_int_bal_tp c )
{
  int size = model * 1000;

  int ndx = 0;
  ary3_int_bal_tp::dimensions_type dims = a.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        a[gid] = prime1000[ rand1000_01[k%1000] ];
        b[gid] = prime1000[ rand1000_02[k%1000] ];
      }
    }
  }

  stapl::rmi_fence();
}

size_t view_55_visit( ary3_int_bal_vw_tp a_vw, ary3_int_bal_vw_tp b_vw )
{
  size_t cksum = 0;

  ary3_int_bal_tp::dimensions_type dims = a_vw.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        cksum ^= (unsigned int) a_vw.get_element(gid) + b_vw.get_element(gid);
      }
    }
  }

  stapl::rmi_fence();

  return cksum;
}

#ifdef EXPLICIT
int view_55_traverse( expl_ary3_int_vw_tp a_vw, expl_ary3_int_vw_tp b_vw )
{
  int check = 0;

  expl_ary3_int_vw_tp::dimensions_type dims = a_vw.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        check += a_vw.get_element(gid) + b_vw.get_element(gid);
      }
    }
  }

  stapl::rmi_fence();

  return check;
}
#endif

bool view_55( size_t model )
{
  size_t num_locs = stapl::get_num_locations();
  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int pages = 10 * model;
  int rows = 10 * model;
  int cols = 10 * model;

  bal_part_tp p0(ndx_dom_tp(0, pages-1), num_locs);
  bal_part_tp p1(ndx_dom_tp(0, rows-1), num_locs);
  bal_part_tp p2(ndx_dom_tp(0, cols-1), num_locs);
  bal3_part_tp bal_part(p0,p1,p2);

  ary3_int_bal_tp a(gid3_tp(pages,rows,cols));
  ary3_int_bal_tp b(bal_part);
  ary3_int_bal_tp c(gid3_tp(pages,rows,cols));

  ary3_int_bal_vw_tp a_vw1(a), b_vw1(b), c_vw1(c);

  view_55_build( model, a, b, c );

  size_t cksum = view_55_visit( a_vw1, b_vw1 );

  std::vector<ndx_dom_tp> dom_vec;
  for ( size_t blk=0; blk < num_locs; blk++ ) {
    int blk_size = prime1kto10k[ rand1000_03[blk]%1024 ];
    dom_vec.push_back( blk_size );
  }
  ndx_dom_tp expl_dom(0, size-1);
  expl_part_tp expl_part(expl_dom, dom_vec);

#ifdef EXPLICIT
  int check = view_55_traverse( a_vw2, b_vw2 );
#endif

  ctr.stop();
  double time1 = ctr.value();

  if ( 0 ) {
     cerr << "55 explicit multiarray PASS" << endl;
  } else {
     cerr << "55 explicit multiarray FAIL" << endl;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////
// multiarray : multiarray view : functor view
// traverse with view 1, assign with view 1
// traverse with view 2, compute with view 2
///////////////////////////////////////////////////////////////////////////

using fun_ary3_int_vw_tp =
  stapl::functor_view_type<stapl::functor_container<ary3_int_bal_vw_tp, 1>>;

void view_56_build( size_t model,
                    ary3_int_bal_tp a, ary3_int_bal_tp b,
                    ary3_int_bal_tp c )
{
  int size = model * 1000;

  int ndx = 0;
  ary3_int_bal_tp::dimensions_type dims = a.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        a[gid] = prime1000[ rand1000_01[k%1000] ];
        b[gid] = prime1000[ rand1000_02[k%1000] ];
      }
    }
  }

  stapl::rmi_fence();
}

size_t view_56_visit( ary3_int_bal_vw_tp a_vw, ary3_int_bal_vw_tp b_vw )
{
  size_t cksum = 0;

  ary3_int_bal_tp::dimensions_type dims = a_vw.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        cksum ^= (unsigned int) a_vw.get_element(gid) + b_vw.get_element(gid);
      }
    }
  }

  stapl::rmi_fence();

  return cksum;
}

#ifdef FUNCTOR
int view_56_traverse( fun_ary3_int_vw_tp a_vw, fun_ary3_int_vw_tp b_vw )
{
  int check = 0;

  ary3_int_bal_vw_tp::dimensions_type dims = a_vw.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        check += a_vw.get_element(gid) + b_vw.get_element(gid);
      }
    }
  }

  stapl::rmi_fence();

  return check;
}
#endif

bool view_56( size_t model )
{
  size_t num_locs = stapl::get_num_locations();
  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int pages = 10 * model;
  int rows = 10 * model;
  int cols = 10 * model;

  bal_part_tp p0(ndx_dom_tp(0, pages-1), num_locs);
  bal_part_tp p1(ndx_dom_tp(0, rows-1), num_locs);
  bal_part_tp p2(ndx_dom_tp(0, cols-1), num_locs);
  bal3_part_tp bal_part(p0,p1,p2);

  ary3_int_bal_tp a(bal_part);
  ary3_int_bal_tp b(gid3_tp(pages,rows,cols));
  ary3_int_bal_tp c(gid3_tp(pages,rows,cols));

  ary3_int_bal_vw_tp a_vw1(a), b_vw1(b), c_vw1(c);

  view_56_build( model, a, b, c );

  size_t cksum = view_56_visit( a_vw1, b_vw1 );

#ifdef FUNCTOR
  plus1<int> plus1_fo;
  (void) stapl::functor_view(size, plus1_fo);

  int check = view_56_traverse( a_vw2, b_vw2 );
#endif

  ctr.stop();
  double time1 = ctr.value();

  if ( 0 ) {
     cerr << "56 functor multiarray PASS" << endl;
  } else {
     cerr << "56 functor multiarray FAIL" << endl;
  }

  return true;
}

