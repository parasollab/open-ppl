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
#include <stapl/containers/partitions/block_cyclic_partition.hpp>
#include <stapl/containers/partitions/blocked_partition.hpp>

#include <stapl/views/strided_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/views/functor_view.hpp>

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

typedef stapl::normal_partition<ndx_dom_tp>     norm_part_tp;
typedef stapl::blk_cyclic_part<ndx_dom_tp>      blk_cyc_part_tp;
typedef stapl::explicit_partition<ndx_dom_tp>   expl_part_tp;
typedef stapl::block_partitioner<ndx_dom_tp>    blk_part_tp;
typedef stapl::explicit_partition<ndx_dom_tp> expl_part_tp;

//////////////////////////////////////////////////////////////////////

bool redistrib_17( int model );
bool redistrib_18( int model );
bool redistrib_19( int model );
bool redistrib_20( int model );

//////////////////////////////////////////////////////////////////////

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
  int size = 1000 * model;

  int first_test = 1;
  int last_test = 36;
  if (opt_test != -1 ) {
    first_test = opt_test;
    last_test = opt_test;
  }

  bool ok = true;
  first_test = 1; last_test = 24;
  for (int test = first_test; test <= last_test; test++ ) {
    cerr << "\n";
    switch (test) {
    case 17:
      ok = redistrib_17(model);
      break;
    case 18:
      ok = redistrib_18(model);
      break;
    case 19:
      ok = redistrib_19(model);
      break;
    case 20:
      ok = redistrib_20(model);
      break;
    }
  }
  return EXIT_SUCCESS;
}

/*==========================================================================*/

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container
// with explicit distribution mechanism.
// Perform computation
// force redistribution to library blocked distribution mechanism,
// perform further computation.
///////////////////////////////////////////////////////////////////////////

void redistrib_17_build( int model,
                        ary3_int_bal_tp a, ary3_int_bal_vw_tp a_vw,
                        ary3_int_bal_tp b, ary3_int_bal_vw_tp b_vw,
                        ary3_int_bal_tp c, ary3_int_bal_vw_tp c_vw,
                        ary3_int_bal_tp d, ary3_int_bal_vw_tp d_vw ) {

  int ndx = 0;
  ary3_int_bal_tp::dimensions_type dims = a.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        a[gid] = prime1000[ rand1000_01[k%1000] ];
      }
    }
  }

  stapl::rmi_fence();
}

size_t redistrib_17_visit( ary3_int_bal_tp a, ary3_int_bal_vw_tp a_vw,
                           ary3_int_bal_tp b, ary3_int_bal_vw_tp b_vw,
                           ary3_int_bal_tp c, ary3_int_bal_vw_tp c_vw,
                           ary3_int_bal_tp d, ary3_int_bal_vw_tp d_vw ) {

  size_t cksum = 0;

  ary3_int_bal_vw_tp::dimensions_type dims = a_vw.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        cksum ^= (unsigned int) (a[gid] + b[gid]);
      }
    }
  }

  stapl::rmi_fence();

  return cksum;
}

bool redistrib_17( int model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int num_locs = stapl::get_num_locations();

  int size = 1000 * model;
  int pages = 10 * model;
  int rows = 10 * model;
  int cols = 10 * model;

  //- - - - - - - - - -  balanced partitioner

  bal_part_tp p0(ndx_dom_tp(0, pages-1), num_locs);
  bal_part_tp p1(ndx_dom_tp(0, rows-1), num_locs);
  bal_part_tp p2(ndx_dom_tp(0, cols-1), num_locs);
  bal3_part_tp bal_partition(p0,p1,p2);

  ary3_int_bal_tp a(gid3_tp(pages,rows,cols));
  ary3_int_bal_tp b(gid3_tp(pages,rows,cols));
  ary3_int_bal_tp c(bal_partition);
  ary3_int_bal_tp d(bal_partition);

  ary3_int_bal_vw_tp a_vw1(a), b_vw1(b), c_vw1(c), d_vw1(d);

  redistrib_17_build( model, a, a_vw1, b, b_vw1, c, c_vw1, d, d_vw1 );

  size_t cksum = redistrib_17_visit( a, a_vw1, b, b_vw1, c, c_vw1, d, d_vw1 );

  //- - - - - - - - - -  block partitioner

  typedef stapl::block_partitioner<ndx_dom_tp>        blk_part_tp;
  typedef stapl::nd_partition<
            stapl::tuple<blk_part_tp, blk_part_tp, blk_part_tp>,
            trav3_tp>                                 blk_3_part_tp;
  typedef stapl::multiarray<3, int, trav3_tp,
                            blk_3_part_tp>            ary3_int_blk_tp;
  typedef stapl::multiarray_view<ary3_int_blk_tp>     ary3_int_blk_vw_tp;

  blk_part_tp blk_p0(ndx_dom_tp(0, pages-1), num_locs);
  blk_part_tp blk_p1(ndx_dom_tp(0, rows-1), num_locs);
  blk_part_tp blk_p2(ndx_dom_tp(0, cols-1), num_locs);

  blk_3_part_tp blk_partition(blk_p0,blk_p1,blk_p2);

#ifdef REDISTRIB

  ndx_dom_tp blk_dom(0, size-1);
  blk_part_tp blk_partition(blk_dom, 1000);

  // apply the block partition to a, b, c, d

  // compute with redistributed data

#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container
// with explicit distribution mechanism.
// Perform computation,
// force redistribution to library blocked cyclic distribution mechanism,
// perform further computation.
///////////////////////////////////////////////////////////////////////////

void redistrib_18_build( int model,
                        ary3_int_bal_tp a, ary3_int_bal_vw_tp a_vw,
                        ary3_int_bal_tp b, ary3_int_bal_vw_tp b_vw,
                        ary3_int_bal_tp c, ary3_int_bal_vw_tp c_vw,
                        ary3_int_bal_tp d, ary3_int_bal_vw_tp d_vw ) {

  ary3_int_bal_tp::dimensions_type dims = a.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        a[gid] = prime1000[ rand1000_01[k%1000] ];
      }
    }
  }

  stapl::rmi_fence();
}

size_t redistrib_18_visit( ary3_int_bal_tp a, ary3_int_bal_vw_tp a_vw,
                           ary3_int_bal_tp b, ary3_int_bal_vw_tp b_vw,
                           ary3_int_bal_tp c, ary3_int_bal_vw_tp c_vw,
                           ary3_int_bal_tp d, ary3_int_bal_vw_tp d_vw ) {

  size_t cksum = 0;

  ary3_int_bal_tp::dimensions_type dims = a.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        cksum ^= (unsigned int) (a[gid] + b[gid]);
      }
    }
  }

  stapl::rmi_fence();

  return cksum;
}

bool redistrib_18( int model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int num_locs = stapl::get_num_locations();

  int size = 1000 * model;
  int pages = 10 * model;
  int rows = 10 * model;
  int cols = 10 * model;

  //- - - - - - - - - -  balanced partitioner

  bal_part_tp bal_p0(ndx_dom_tp(0, pages-1), num_locs);
  bal_part_tp bal_p1(ndx_dom_tp(0, rows-1), num_locs);
  bal_part_tp bal_p2(ndx_dom_tp(0, cols-1), num_locs);
  bal3_part_tp bal_partition(bal_p0,bal_p1,bal_p2);

  ary3_int_bal_tp a(bal_partition);
  ary3_int_bal_tp b(bal_partition);
  ary3_int_bal_tp c(gid3_tp(pages,rows,cols));
  ary3_int_bal_tp d(gid3_tp(pages,rows,cols));

  ary3_int_bal_vw_tp a_vw1(a), b_vw1(b), c_vw1(c), d_vw1(d);

  redistrib_18_build( model, a, a_vw1, b, b_vw1, c, c_vw1, d, d_vw1 );

  size_t cksum = redistrib_18_visit( a, a_vw1, b, b_vw1, c, c_vw1, d, d_vw1 );

  //- - - - - - - - - -  block cyclic partitioner

#ifdef BLOCK_CYCLIC
  typedef stapl::blk_cyclic_part<ndx_dom_tp>            blk_cyc_part_tp;
  typedef stapl::nd_partition<
            stapl::tuple<blk_cyc_part_tp, blk_cyc_part_tp, blk_cyc_part_tp>,
            trav3_tp>                                   blk_cyc_3_part_tp;
  typedef stapl::multiarray<3, int, trav3_tp,
                            blk_cyc_3_part_tp>          ary3_int_blk_cyc_tp;
  typedef stapl::multiarray_view<ary3_int_blk_cyc_tp>   ary3_int_blk_cyc_vw_tp;

  blk_cyc_part_tp bc_p0(ndx_dom_tp(0, pages-1), num_locs);
  blk_cyc_part_tp bc_p1(ndx_dom_tp(0, rows-1), num_locs);
  blk_cyc_part_tp bc_p2(ndx_dom_tp(0, cols-1), num_locs);
  blk_cyc_3_part_tp blk_cyc_partition(bc_p0,bc_p1,bc_p2);

#ifdef REDISTRIB
  // apply the block cyclic partition to a, b, c, d

  // compute with redistributed data

#endif
#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
}


///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container
// with explicit distribution mechanism.
// Perform computation,
// force redistribution to library balanced distribution mechanism,
// perform further computation.
///////////////////////////////////////////////////////////////////////////

void redistrib_19_build( int model,
                        ary3_int_bal_tp a, ary3_int_bal_vw_tp a_vw,
                        ary3_int_bal_tp b, ary3_int_bal_vw_tp b_vw,
                        ary3_int_bal_tp c, ary3_int_bal_vw_tp c_vw,
                        ary3_int_bal_tp d, ary3_int_bal_vw_tp d_vw ) {

  ary3_int_bal_tp::dimensions_type dims = a.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        a[gid] = prime1000[ rand1000_01[k%1000] ];
      }
    }
  }

  stapl::rmi_fence();
}

size_t redistrib_19_visit( ary3_int_bal_tp a, ary3_int_bal_vw_tp a_vw,
                           ary3_int_bal_tp b, ary3_int_bal_vw_tp b_vw,
                           ary3_int_bal_tp c, ary3_int_bal_vw_tp c_vw,
                           ary3_int_bal_tp d, ary3_int_bal_vw_tp d_vw ) {

  size_t cksum = 0;

  ary3_int_bal_tp::dimensions_type dims = a.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        cksum ^= (unsigned int) (a[gid] + b[gid]);
      }
    }
  }

  stapl::rmi_fence();

  return cksum;
}

bool redistrib_19( int model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int num_locs = stapl::get_num_locations();

  int size = 1000 * model;
  int pages = 10 * model;
  int rows = 10 * model;
  int cols = 10 * model;

  //- - - - - - - - - -  balanced partitioner

  bal_part_tp p0(ndx_dom_tp(0, pages-1), num_locs);
  bal_part_tp p1(ndx_dom_tp(0, rows-1), num_locs);
  bal_part_tp p2(ndx_dom_tp(0, cols-1), num_locs);
  bal3_part_tp bal_partition(p0,p1,p2);

  ary3_int_bal_tp a(gid3_tp(pages,rows,cols));
  ary3_int_bal_tp b(bal_partition);
  ary3_int_bal_tp c(gid3_tp(pages,rows,cols));
  ary3_int_bal_tp d(bal_partition);

  ary3_int_bal_vw_tp a_vw1(a), b_vw1(b), c_vw1(c), d_vw1(d);

  redistrib_19_build( model, a, a_vw1, b, b_vw1, c, c_vw1, d, d_vw1 );

  size_t cksum = redistrib_19_visit( a, a_vw1, b, b_vw1, c, c_vw1, d, d_vw1 );

  //- - - - - - - - - -  explicit partitioner

  typedef stapl::explicit_partition<ndx_dom_tp>          expl_part_tp;
  typedef stapl::nd_partition<
            stapl::tuple<expl_part_tp, expl_part_tp, expl_part_tp>,
            trav3_tp>                                    expl_3_part_tp;
  typedef stapl::multiarray<3, int, trav3_tp,
                            expl_3_part_tp>              ary3_int_expl_tp;
  typedef stapl::multiarray_view<ary3_int_expl_tp>       ary3_int_expl_vw_tp;

  int base = 0;
  std::vector<ndx_dom_tp> dom_vec;
  for (int blk = 0; blk < num_locs; blk++) {
    int blk_size = prime1kto10k[ rand1000_03[blk]%1024 ];
    for (int ndx= base; ndx<base+blk_size; ndx++ ) {
      dom_vec.push_back( ndx );
    }
    base += 1024;
  }
  ndx_dom_tp expl_dom(0, num_locs*1024);
  expl_part_tp expl_part(expl_dom, dom_vec);

#ifdef REDISTRIB
  // apply the explicit partition to a, b, c, d

  // compute with redistributed data

#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
}


///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container
// with explicit distribution mechanism.
// Perform computation,
// force redistribution to library normal distribution mechanism,
// perform further computation.
///////////////////////////////////////////////////////////////////////////

void redistrib_20_build( int model,
                        ary3_int_bal_tp a, ary3_int_bal_vw_tp a_vw,
                        ary3_int_bal_tp b, ary3_int_bal_vw_tp b_vw,
                        ary3_int_bal_tp c, ary3_int_bal_vw_tp c_vw,
                        ary3_int_bal_tp d, ary3_int_bal_vw_tp d_vw ) {

  ary3_int_bal_tp::dimensions_type dims = a.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        a[gid] = prime1000[ rand1000_01[k%1000] ];
      }
    }
  }

  stapl::rmi_fence();
}

size_t redistrib_20_visit( ary3_int_bal_tp a, ary3_int_bal_vw_tp a_vw,
                           ary3_int_bal_tp b, ary3_int_bal_vw_tp b_vw,
                           ary3_int_bal_tp c, ary3_int_bal_vw_tp c_vw,
                           ary3_int_bal_tp d, ary3_int_bal_vw_tp d_vw ) {

  size_t cksum = 0;

  ary3_int_bal_tp::dimensions_type dims = a.dimensions();
  for (size_t i = 0; i < stapl::get<0>(dims); i++ ) {
    for (size_t j = 0; j < stapl::get<1>(dims); j++ ) {
      for (size_t k = 0; k < stapl::get<2>(dims); k++ ) {
        gid3_tp gid(i,j,k);
        cksum ^= (unsigned int) (a[gid] + b[gid]);
      }
    }
  }

  stapl::rmi_fence();

  return cksum;
}

bool redistrib_20( int model ) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int num_locs = stapl::get_num_locations();

  int size = 1000 * model;
  int pages = 10 * model;
  int rows = 10 * model;
  int cols = 10 * model;

  //- - - - - - - - - -  balanced partitioner

  bal_part_tp p0(ndx_dom_tp(0, pages-1), num_locs);
  bal_part_tp p1(ndx_dom_tp(0, rows-1), num_locs);
  bal_part_tp p2(ndx_dom_tp(0, cols-1), num_locs);
  bal3_part_tp bal_partition(p0,p1,p2);

  ary3_int_bal_tp a(bal_partition);
  ary3_int_bal_tp b(gid3_tp(pages,rows,cols));
  ary3_int_bal_tp c(bal_partition);
  ary3_int_bal_tp d(gid3_tp(pages,rows,cols));

  ary3_int_bal_vw_tp a_vw1(a), b_vw1(b), c_vw1(c), d_vw1(d);

  redistrib_20_build( model, a, a_vw1, b, b_vw1, c, c_vw1, d, d_vw1 );

  size_t cksum = redistrib_20_visit( a, a_vw1, b, b_vw1, c, c_vw1, d, d_vw1 );

  //- - - - - - - - - -  normal partitioner

#if 1
  typedef stapl::normal_partition<ndx_dom_tp>            norm_part_tp;
  typedef stapl::nd_partition<
            stapl::tuple<norm_part_tp, norm_part_tp, norm_part_tp>,
            trav3_tp>                                    norm_3_part_tp;
  typedef stapl::multiarray<3, int, trav3_tp,
                            norm_3_part_tp>              ary3_int_norm_tp;
  typedef stapl::multiarray_view<ary3_int_norm_tp>       ary3_int_norm_vw_tp;

  norm_part_tp nm_p0(ndx_dom_tp(0, pages-1), num_locs, 0, 1.0);
  norm_part_tp nm_p1(ndx_dom_tp(0, rows-1), num_locs, 0, 1.0);
  norm_part_tp nm_p2(ndx_dom_tp(0, cols-1), num_locs, 0, 1.0);

  norm_3_part_tp norm_partition(nm_p0,nm_p1,nm_p2);
#endif

#ifdef REDISTRIB
  // apply the explicit partition to a, b, c, d

  // compute with redistributed data

#endif

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();
}
