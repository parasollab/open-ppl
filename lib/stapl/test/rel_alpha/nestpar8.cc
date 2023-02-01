/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

//#########################################################################
//  Data Distribution Tests
//  nestpar6  - distribution of components specified by functor individually
//              locations not specified
//              containers are array, vector, map,  nesting depth 2
//  nestpar7  - distribution of components specified by functor individually
//              locations specified explicitly
//              containers are array, vector, map,  nesting depth 2
//  nestpar8  - distribution of components specified by functor individually
//              locations specified by tag
//              containers are array, vector, map,  nesting depth 2
//
//  nestpar11 - distribution of components specified by vector per level
//              locations not specified
//              containers are array, vector, map,  nesting depth 3
//  nestpar12 - distribution of components specified by vector per level
//              locations specified explicitly
//              containers are array, vector, map,  nesting depth 3
//  nestpar13 - distribution of components specified by vector per level
//              locations specified by tag
//              containers are array, vector, map,  nesting depth 3
//#########################################################################

///////////////////////////////////////////////////////////////////////////
//  THIS FILE HAS NOT YET BEEN ADAPTED TO USE LOCATION TAGS
///////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <cstdlib>

#include <iostream>

#include <stapl/utility/tuple.hpp>
#include <stapl/domains/indexed.hpp>

#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>

#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>

#include <stapl/views/map_view.hpp>
#include <stapl/containers/map/map.hpp>

#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>

#include <stapl/skeletons/serial.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include <stapl/utility/do_once.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>

#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include "json_parser.hpp"

using namespace std;
#include "testutil.hpp"
#include "rel_alpha_data.h"
#include "rel_alpha_util.hpp"

//#define LEN_CONSTR

/*=========================================================================*/

typedef stapl::indexed_domain<int> ndx_dom_tp;

typedef stapl::array<size_t> ary_sz_tp;
typedef stapl::array_view<ary_sz_tp> ary_sz_vw_tp;

#ifdef LEN_CONSTR
typedef stapl::map<int,int> map_int_tp;
typedef stapl::map< int, stapl::map<int,int> > map_map_int_tp;
#else

typedef stapl::distribution_spec<> dist_spec_tp;

typedef stapl::vector<int,
                      stapl::view_based_partition<dist_spec_tp>,
                      stapl::view_based_mapper<dist_spec_tp> > vec_int_tp;
typedef stapl::vector<vec_int_tp,
                      stapl::view_based_partition<dist_spec_tp>,
                      stapl::view_based_mapper<dist_spec_tp> > vec_vec_int_tp;

typedef stapl::array<int,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> > ary_int_tp;
typedef stapl::array<ary_int_tp,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> > ary_ary_int_tp;

typedef stapl::map<int,int,
                   stapl::view_based_partition<dist_spec_tp>,
                   stapl::view_based_mapper<dist_spec_tp> > map_int_tp;
typedef stapl::map<int, map_int_tp,
                   stapl::view_based_partition<dist_spec_tp>,
                   stapl::view_based_mapper<dist_spec_tp> > map_map_int_tp;
#endif

typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;
typedef stapl::vector_view<vec_vec_int_tp> vec_vec_int_vw_tp;

typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;
typedef stapl::array_view<ary_ary_int_tp> ary_ary_int_vw_tp;

typedef stapl::map_view<map_int_tp> map_int_vw_tp;
typedef stapl::map_view<map_map_int_tp> map_map_int_vw_tp;

/*=========================================================================*/

bool open_zin(int model, int test, stapl::stream<ifstream>& zin, int &count) {
  string path;
  switch ( model ) {
  case 1:
    switch( test ) {
    case 812:
    case 815:
    case 818:
      path= "data/tiny_factors.zin";
      break;
    default:
      path="data/tiny_primes.zin";
      break;
    }
    break;
  case 100:
    switch( test ) {
    case 812:
    case 815:
    case 818:
      path = "data/small_factors.zin";
      break;
    default:
      path = "data/small_primes.zin";
      break;
    }
    break;
  case 10000:
    switch( test ) {
    case 812:
    case 815:
    case 818:
      path = "data/medium_factors.zin";
      break;
    default:
      path = "data/medium_primes.zin";
      break;
    }
    break;
  case 1000000:
    switch( test ) {
    case 812:
    case 815:
    case 818:
      path = "data/big_factors.zin";
      break;
    default:
      path = "data/big_primes.zin";
      break;
    }
    break;
  case 100000000:
    switch( test ) {
    case 812:
    case 815:
    case 818:
      path = "data/huge_factors.zin";
      break;
    default:
      path = "data/huge_primes.zin";
      break;
    }
    break;
  }
  map<string,int> meta_data;
  bool read_meta_data( map<string,int> &);

  zin.open(path.c_str());
  if ( zin.is_open() ) {
#ifdef DEBUG
    stapl::do_once([&](){
      cerr << "Opened input file: " << path << endl;
    });
#endif
    string base_name = path;
    string::size_type idx = base_name.find('/');
    if ( idx != string::npos ) {
      base_name = base_name.substr(idx+1);
    }
    idx = base_name.find('.');
    if ( idx != string::npos ) {
      base_name = base_name.substr(0,idx);
    }

    if ( read_meta_data(meta_data) ) {
      count = meta_data[base_name];
#ifdef DEBUG
      stapl::do_once([&](){
        cerr << "meta_data[" << base_name << "]=" << count << endl;
      });
#endif
      assert( count != 0 );
      return true;
    } else {
      stapl::do_once([&](){
        cerr << "Unable to open meta_data file: data/meta_data " << endl;
      });
      return false;
    }
  } else {
    stapl::do_once([&](){
      cerr << "Unable to open input file: " << path << endl;
    });
    return false;
  }
}

/*=========================================================================*/

size_t nestpar_801( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_802( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_803( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_804( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_805( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_806( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_807( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_808( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_809( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_810( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_811( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_812( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_813( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_814( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_815( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_816( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_817( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_818( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_819( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_820( size_t, int &,
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

  int first_test = 801;
  int last_test = 820;
  if (opt_test != -1) {
    first_test = opt_test;
    last_test = opt_test;
  }

  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;

  int count = 0;
  bool ok = true;
  for (int test=first_test; test<=last_test; test++ ) {
#ifdef DEBUG
    stapl::do_once([&](){
      std::cout << "Nested Parallel Dist " << test << endl;
    });
#endif
    set_random_seed();
    int result = 0;
    switch ( test) {
    case 801:
      zout.open("np801.zout");
      result = nestpar_801(model, zin, zout);
      break;
    case 802:
      zout.open("np802.zout");
      result = nestpar_802(model, zin, zout);
      break;
#ifdef MAP
    case 803:
      zout.open("np803.zout");
      result = nestpar_803(model, zin, zout);
      break;
#endif
    case 804:
      zout.open("np804.zout");
      result = nestpar_804(model, zin, zout);
      break;
    case 805:
      zout.open("np805.zout");
      result = nestpar_805(model, zin, zout);
      break;
#ifdef MAP
    case 806:
      zout.open("np806.zout");
      result = nestpar_806(model, zin, zout);
      break;
#endif
    case 807:
      zout.open("np807.zout");
      result = nestpar_807(model, zin, zout);
      break;
    case 808:
      zout.open("np808.zout");
      result = nestpar_808(model, zin, zout);
      break;
#ifdef MAP
    case 809:
      zout.open("np809.zout");
      result = nestpar_809(model, zin, zout);
      break;
#endif
    case 810:
      zout.open("np810.zout");
      if ( open_zin(model,810,zin,count) ) {
        result = nestpar_810(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 811:
      zout.open("np811.zout");
      if ( open_zin(model,811,zin,count) ) {
        result = nestpar_811(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#ifdef MAP
    case 812:
      zout.open("np812.zout");
      if ( open_zin(model,812,zin,count) ) {
        result = nestpar_812(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif
    case 813:
      zout.open("np813.zout");
      if ( open_zin(model,813,zin,count) ) {
        result = nestpar_813(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 814:
      zout.open("np814.zout");
      if ( open_zin(model,814,zin,count) ) {
        result = nestpar_814(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#ifdef MAP
    case 815:
      zout.open("np815.zout");
      if ( open_zin(model,815,zin,count) ) {
        result = nestpar_815(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif
    case 816:
      zout.open("np816.zout");
      if ( open_zin(model,816,zin,count) ) {
        result = nestpar_816(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 817:
      zout.open("np817.zout");
      if ( open_zin(model,817,zin,count) ) {
        result = nestpar_817(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#ifdef MAP
    case 818:
      zout.open("np818.zout");
      if ( open_zin(model,818,zin,count) ) {
        result = nestpar_818(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif
    case 819:
      zout.open("np819.zout");
      if ( open_zin(model,819,zin,count) ) {
        result = nestpar_819(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 820:
      zout.open("np820.zout");
      if ( open_zin(model,820,zin,count) ) {
        result = nestpar_820(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    }
    zout.close();
#ifdef DEBUG
    stapl::do_once([&](){
      std::cout << endl << "Nested Parallel Dist: " << result << endl;
    });
#endif
  }
  return EXIT_SUCCESS;
}

/*=========================================================================
 * functors for constructing distribution specs
 *=========================================================================*/

//////////////////////////////////////////////////////////////////////
// Even indices are blocked.  Odd indices are block-cyclic.
//////////////////////////////////////////////////////////////////////

struct gen_spec_blk_blkcyc_wf
{
private:
  size_t m_limit;
  dist_spec_tp *m_outer_dist;
  stapl::rmi_handle::reference m_ref;
public:
  gen_spec_blk_blkcyc_wf(size_t limit, dist_spec_tp *outer_dist)
    : m_limit(limit), m_outer_dist(outer_dist)
  { }
  typedef dist_spec_tp result_type;
  result_type operator()(std::vector<size_t> const& ndx) const
  {
    if ( ndx.empty() ) {
      return *m_outer_dist;
    } else {
      size_t size = 1 + (rand() % m_limit);
      size_t blk_fac = ( size >= 10 ) ? (blk_fac = size / 10) : size;
      if (ndx[0] % 2 == 0) {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blk_blkcyc 2: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block(size, blk_fac, stapl::lower_level);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blk_blkcyc 3: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac, stapl::lowest_level);
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_limit);
    t.transient(m_outer_dist,stapl::resolve_handle<dist_spec_tp>(m_ref));
    if (!m_outer_dist) {
      stapl::abort("failed to resolve handle\n");
    }
  }
};

//////////////////////////////////////////////////////////////////////
// Even indices are cyclic.  Odd indices are block-cyclic.
//////////////////////////////////////////////////////////////////////

struct gen_spec_cyc_blkcyc_wf
{
private:
  size_t m_limit;
  dist_spec_tp *m_outer_dist;
  stapl::rmi_handle::reference m_ref;
public:
  gen_spec_cyc_blkcyc_wf(size_t limit, dist_spec_tp *outer_dist)
    : m_limit(limit), m_outer_dist(outer_dist)
  { }

  typedef dist_spec_tp result_type;
  result_type operator()(std::vector<size_t> const& ndx) const
  {
    if ( ndx.empty() ) {
      return *m_outer_dist;
    } else {
      size_t size = 1 + (rand() % m_limit);
      if (ndx[0] % 2 == 0) {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc 2: " << size << endl;
});
#endif
        return stapl::cyclic(size, stapl::lower_level);
      } else {
        size_t blk_fac = ( size >= 10 ) ? (blk_fac = size / 10) : size;
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc 3: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac, stapl::lower_level);
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_limit);
    t.transient(m_outer_dist,stapl::resolve_handle<dist_spec_tp>(m_ref));
    if (!m_outer_dist) {
      stapl::abort("failed to resolve handle\n");
    }
  }
};

//////////////////////////////////////////////////////////////////////
// First half indices are block-cyclic. Second half indices are blocked
//////////////////////////////////////////////////////////////////////

struct gen_spec_blkcyc_blk_wf
{
private:
  size_t m_limit;
  dist_spec_tp *m_outer_dist;
  stapl::rmi_handle::reference m_ref;
public:
  gen_spec_blkcyc_blk_wf(size_t limit, dist_spec_tp *outer_dist)
    : m_limit(limit), m_outer_dist(outer_dist)
  { }

  typedef dist_spec_tp result_type;
  result_type operator()(std::vector<size_t> const& ndx) const
  {
    if ( ndx.empty() ) {
      return *m_outer_dist;
    } else {
      size_t size = 1 + (rand() % m_limit);
      size_t blk_fac = ( size >= 10 ) ? (blk_fac = size / 10) : size;
      if (ndx[0] < (size / 2) == 0) {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blkcyc_blk 2: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac, stapl::lowest_level);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blkcyc_blk 3: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block(size, blk_fac, stapl::lowest_level);
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_limit);
    t.transient(m_outer_dist,stapl::resolve_handle<dist_spec_tp>(m_ref));
    if (!m_outer_dist) {
      stapl::abort("failed to resolve handle\n");
    }
  }
};

//////////////////////////////////////////////////////////////////////
// First half indices are block-cyclic. Second half indices are cyclic
//////////////////////////////////////////////////////////////////////

struct gen_spec_blkcyc_cyc_wf
{
private:
  size_t m_limit;
  dist_spec_tp *m_outer_dist;
  stapl::rmi_handle::reference m_ref;
public:
  gen_spec_blkcyc_cyc_wf(size_t limit, dist_spec_tp *outer_dist)
    : m_limit(limit), m_outer_dist(outer_dist)
  { }

  typedef dist_spec_tp result_type;
  result_type operator()(std::vector<size_t> const& ndx) const
  {
    if ( ndx.empty() ) {
      return *m_outer_dist;
    } else {
      size_t size = 1 + (rand() % m_limit);
      if (ndx[0] < (size / 2) == 0) {
        size_t blk_fac = ( size >= 10 ) ? (blk_fac = size / 10) : size;
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blkcyc_cyc 2: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac, stapl::lower_level);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blkcyc_cyc 3: " << size << endl;
});
#endif
        return stapl::cyclic(size, stapl::lowest_level);
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_limit);
    t.transient(m_outer_dist,stapl::resolve_handle<dist_spec_tp>(m_ref));
    if (!m_outer_dist) {
      stapl::abort("failed to resolve handle\n");
    }
  }
};

/*=========================================================================
 * homogeneous nested structures
 *=========================================================================*/

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with generator, traverse with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_801_fill_wf {

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::iota( vw1, 0 );
  }
};

struct nestpar_801_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return vw1.size();
  }
};

struct nestpar_801_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_801_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_801( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp cyc_dist = stapl::cyclic(size);
  gen_spec_blk_blkcyc_wf nestpar_801_dist_wf(limit,&cyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_801_dist_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);

  stapl::map_func(nestpar_801_fill_wf(), a_vw );

  stapl::map_func(nestpar_801_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

#ifdef DEBUG_RT_ABORT
stapl::do_once([&](){
  cerr << "np801 a " << endl;
});
#endif

  stapl::serial_io(nestpar_801_show_wf(zout), a_vw );

#ifdef DEBUG_RT_ABORT
stapl::do_once([&](){
  cerr << "np801 b " << endl;
});
#endif

  ctr.stop();
  time_p = ctr.value();
  show_time("NP801","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with generator, traverse with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_802_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    int base = 10;
    int step = 2;
    typedef stapl::sequence<int> step_wf;
    stapl::generate(vw1, step_wf(base,step));
  }
};

struct nestpar_802_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return vw1.size();
  }
};

struct nestpar_802_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_802_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_802( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blk_dist = stapl::block(size, blk_size);
  gen_spec_cyc_blkcyc_wf nestpar_802_gen_wf(limit,&blk_dist);
  stapl::composed_dist_spec comp_spec(nestpar_802_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);

#ifdef DEBUG_HANG
stapl::do_once([&](){
  cerr << "np802 a " << endl;
});
#endif

  stapl::map_func(nestpar_802_fill_wf(), a_vw);

#ifdef DEBUG_HANG
stapl::do_once([&](){
  cerr << "np802 b " << endl;
});
#endif

  stapl::map_func(nestpar_802_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_802_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP802","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with data from C arrays, traverse with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_803_fill_wf {
private:
  size_t outer_;
  size_t inner_;
public:
  nestpar_803_fill_wf(size_t o, size_t i)
    : outer_(o), inner_(i)
  { }
  typedef int result_type;
  template <typename Data>
  result_type operator()(Data &a) {
    if (outer_ <= 100000 ) {
      for (size_t i = 0; i < outer_; i++ ) {
        for (size_t j = 0; j < inner_; j++ ) {
          int value = rand_nums[i] % (j+1);
          a[ prime_nums[i] ] [ rand_nums[j] ] = value;
        }
      }
    } else {
      for (size_t i = 0; i < outer_; i += 10000 ) {
        for (size_t j = 0; j < inner_; j++ ) {
          for (size_t k = 0; k < 10000; k++ ) {
            int value = fibo20[i%20];
            a[ prime_nums[k] * prime_nums[10000-k] ][ rand_nums[j] ] = value;
          }
        }
      }
    }
  }
};

struct nestpar_803_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    return elem.second.size();
  }
};

size_t nestpar_803( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

#ifdef LEN_CONSTR
  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));
  ndx_dom_tp map_dom(0, 100000);
  map_map_int_tp a(map_dom), b(map_dom);
  map_map_int_vw_tp a_vw(a), b_vw(b);
#else
  dist_spec_tp bal_dist = stapl::balance(size);
  gen_spec_blkcyc_blk_wf nestpar_803_gen_wf(limit,&bal_dist);
  stapl::composed_dist_spec comp_spec(nestpar_803_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_803_fill_wf(size, limit), a_vw);

  stapl::map_func(nestpar_803_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP803","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_804_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    int rep = 42;
    int base = 0;
    typedef stapl::block_sequence<int> repeat_wf;
    stapl::generate(vw1, repeat_wf(base,rep));
  }
};

struct nestpar_804_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_804_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_804_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_804( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blkcyc_dist = stapl::block_cyclic(size, blk_size);
  gen_spec_blkcyc_cyc_wf nestpar_804_gen_wf(limit,&blkcyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_804_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);

  stapl::map_func(nestpar_804_fill_wf(), a_vw );

  stapl::map_func(nestpar_804_process_wf(), a_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

#ifdef DEBUG_RT_ABORT
stapl::do_once([&](){
  cerr << "np804 a " << endl;
});
#endif

  stapl::serial_io(nestpar_804_show_wf(zout), a_vw);

#ifdef DEBUG_RT_ABORT
stapl::do_once([&](){
  cerr << "np804 b " << endl;
});
#endif

  ctr.stop();
  time_p = ctr.value();
  show_time("NP804","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_805_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
     stapl::generate( vw1, stapl::random_sequence());
  }
};

struct nestpar_805_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), vw1 );
  }
};

struct nestpar_805_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_805_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_805( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp cyc_dist = stapl::cyclic(size);
  gen_spec_cyc_blkcyc_wf nestpar_805_gen_wf(limit,&cyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_805_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);

#ifdef DEBUG_HANG
stapl::do_once([&](){
  cerr << "np805 a " << endl;
});
#endif

  stapl::map_func(nestpar_805_fill_wf(), a_vw );

#ifdef DEBUG_HANG
stapl::do_once([&](){
  cerr << "np805 b " << endl;
});
#endif

  stapl::map_func(nestpar_805_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_805_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP805","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map data from C arrays, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_806_fill_wf {
private:
  size_t outer_;
  size_t inner_;
public:
  nestpar_806_fill_wf(size_t o, size_t i)
    : outer_(o), inner_(i)
  { }
  typedef int result_type;
  template <typename Data>
  result_type operator()(Data &a) {
    if (outer_ <= 100000 ) {
      for (size_t i = 0; i < outer_; i++ ) {
        for (size_t j = 0; j < inner_; j++ ) {
          int value = rand_nums[i] % (j+1);
          a[ prime_nums[i] ] [ rand_nums[j] ] = value;
        }
      }
    } else {
      for (size_t i = 0; i < outer_; i += 10000 ) {
        for (size_t j = 0; j < inner_; j++ ) {
          for (size_t k = 0; k < 10000; k++ ) {
            int value = fibo20[i%20];
            a[ prime_nums[k] * prime_nums[10000-k] ][ rand_nums[j] ] = value;
          }
        }
      }
    }
  }
};

struct nestpar_806_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    return stapl::map_reduce( inner_map_elem_wf(), add_int_wf(), elem.second);
  }
};

size_t nestpar_806( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

#ifdef LEN_CONSTR
  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));
  ndx_dom_tp map_dom(0, 100000);
  map_map_int_tp a(map_dom), b(map_dom);
  map_map_int_vw_tp a_vw(a), b_vw(b);
#else
  dist_spec_tp blk_dist = stapl::block(size, blk_size);
  gen_spec_blkcyc_blk_wf nestpar_806_gen_wf(limit,&blk_dist);
  stapl::composed_dist_spec comp_spec(nestpar_806_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_806_fill_wf(size, limit), a_vw);

  stapl::map_func(nestpar_806_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP806","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with generator, process with stapl algorithm
// apply unary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_807_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    int base = 0;
    int step = 10;
    typedef stapl::sequence<int> step_wf;
    stapl::generate(vw1, step_wf(base,step));
  }
};

struct nestpar_807_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const& vw2)
  {
    stapl::transform(vw1, vw2, neg_int_wf());
  }
};

struct nestpar_807_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_807_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_807( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blkcyc_dist = stapl::block_cyclic(size, blk_size);
  gen_spec_blkcyc_blk_wf nestpar_807_gen_wf(limit,&blkcyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_807_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);
  vec_vec_int_tp b(comp_spec);
  vec_vec_int_vw_tp b_vw(b);

  stapl::map_func(nestpar_807_fill_wf(), a_vw );

#ifdef DEBUG_RT_ABORT
stapl::do_once([&](){
  cerr << "np807 a " << endl;
});
#endif

  stapl::serial_io(nestpar_807_show_wf(zout), a_vw);

#ifdef DEBUG_RT_ABORT
stapl::do_once([&](){
  cerr << "np807 b " << endl;
});
#endif

  stapl::map_func(nestpar_807_process_wf(), a_vw, b_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_807_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP807","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with generator, process with stapl algorithm
// apply binary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_808_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    int base = 0;
    int step = 10;
    typedef stapl::sequence<int> step_wf;
    stapl::generate(vw1, step_wf(base,step));
  }
};

struct nestpar_808_process_wf {
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const &vw1, View2 const& vw2, View3 const& vw3)
  {
    stapl::transform(vw1, vw2, vw3, add_int_wf());
  }
};

struct nestpar_808_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_808_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_808( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp bal_dist = stapl::balance(size);
  gen_spec_blkcyc_cyc_wf nestpar_808_gen_wf(limit,&bal_dist);
  stapl::composed_dist_spec comp_spec(nestpar_808_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);
  ary_ary_int_tp b(comp_spec);
  ary_ary_int_vw_tp b_vw(b);
  ary_ary_int_tp c(comp_spec);
  ary_ary_int_vw_tp c_vw(c);

#ifdef DEBUG_HANG
stapl::do_once([&](){
  cerr << "np808 a " << endl;
});
#endif

  stapl::map_func(nestpar_808_fill_wf(), a_vw);
  stapl::map_func(nestpar_808_fill_wf(), b_vw);

#ifdef DEBUG_HANG
stapl::do_once([&](){
  cerr << "np808 b " << endl;
});
#endif

  stapl::serial_io(nestpar_808_show_wf(zout), a_vw);
  stapl::serial_io(nestpar_808_show_wf(zout), b_vw);

  stapl::map_func(nestpar_808_process_wf(), a_vw, b_vw, c_vw);

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_808_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP808","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with data from C arrays,
// process with stapl algorithm
//////////////////////////////////////////////////////////////////////

struct nestpar_809_fill_wf {
private:
  size_t outer_;
  size_t inner_;
public:
  nestpar_809_fill_wf(size_t o, size_t i)
    : outer_(o), inner_(i)
  { }
  typedef int result_type;
  template <typename Data>
  result_type operator()(Data &a) {
    if (outer_ <= 100000 ) {
      for (size_t i = 0; i < outer_; i++ ) {
        for (size_t j = 0; j < inner_; j++ ) {
          int value = rand_nums[i] % (j+1);
          a[ prime_nums[i] ] [ rand_nums[j] ] = value;
        }
      }
    } else {
      for (size_t i = 0; i < outer_; i += 10000 ) {
        for (size_t j = 0; j < inner_; j++ ) {
          for (size_t k = 0; k < 10000; k++ ) {
            int value = fibo20[i%20];
            a[ prime_nums[k] * prime_nums[10000-k] ][ rand_nums[j] ] = value;
          }
        }
      }
    }
  }
};

struct nestpar_809_process_wf {
  typedef size_t result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    int found = 0;
    int const & value = prime_nums[elem.second.size()/2];
    typename Element::second_reference map_arg = elem.second;
    if (map_arg.end() != map_arg.begin() ) {
      found = 1;
    }
    return found;
  }
};

size_t nestpar_809( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

#ifdef LEN_CONSTR
  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));
  ndx_dom_tp map_dom(0, 100000);
  map_map_int_tp a(map_dom), b(map_dom);
  map_map_int_vw_tp a_vw(a), b_vw(b);
#else
  dist_spec_tp cyc_dist = stapl::cyclic(size);
  gen_spec_blk_blkcyc_wf nestpar_809_gen_wf(limit,&cyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_809_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_809_fill_wf(size, limit), a_vw);

  stapl::map_func(nestpar_809_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP809","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with serial_io I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_810_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_810_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(get_val_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_810_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const& vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_810_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_810_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_ndx_val_wf(m_zout),
                     stapl::counting_view<int>(vw1.size()), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_810( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blk_dist = stapl::block(size, blk_size);
  gen_spec_cyc_blkcyc_wf nestpar_810_gen_wf(limit,&blk_dist);
  stapl::composed_dist_spec comp_spec(nestpar_810_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);

  stapl::serial_io(nestpar_810_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_810_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP810","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with serial_io I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_811_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_811_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(get_val_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_811_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_811_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_811_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_ndx_val_wf(m_zout),
                     stapl::counting_view<int>(vw1.size()), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_811( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blkcyc_dist = stapl::block_cyclic(size, blk_size);
  gen_spec_blkcyc_blk_wf nestpar_811_gen_wf(limit,&blkcyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_811_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);
  ary_ary_int_tp b(comp_spec);
  ary_ary_int_vw_tp b_vw(b);

  stapl::serial_io(nestpar_811_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_811_process_wf(), a_vw, b_vw);

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_811_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP811","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with serial_io I/O, display map
//////////////////////////////////////////////////////////////////////

struct nestpar_812_fill_wf {
private:
  size_t sz_;
public:
  nestpar_812_fill_wf(size_t s )
    : sz_(s)
  { }
  typedef int result_type;
  template <typename Ref1, typename Ref2, typename Ref3, typename Data>
  result_type operator()(Ref1 lkey, Ref2 rkey, Ref3 val, Data &a) {
    for (size_t i = 0; i < sz_; i++ ) {
      a[lkey[i]][rkey[i]] = val[i];
    }
  }
  void define_type(stapl::typer& t) {
    t.member(sz_);
  }
};

size_t nestpar_812( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  int limit = count;
  ary_sz_tp lkey(limit), rkey(limit), val(limit);
  ary_sz_vw_tp lkey_vw(lkey), rkey_vw(rkey), val_vw(val);
  stapl::serial_io(get_triple_wf(zin), lkey_vw, rkey_vw, val_vw );

#ifdef LEN_CONSTR
  ndx_dom_tp map_dom(0, 16777216);
  map_map_int_tp a(map_dom), b(map_dom);
  map_map_int_vw_tp a_vw(a), b_vw(b);
#else
  dist_spec_tp bal_dist = stapl::balance(size);
  gen_spec_blkcyc_cyc_wf nestpar_812_gen_wf(limit,&bal_dist);
  stapl::composed_dist_spec comp_spec(nestpar_812_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_812_fill_wf(limit), lkey_vw, rkey_vw, val_vw, a_vw);

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP812","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with serial_io I/O,
// process with mapreduce / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_813_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_813_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(get_val_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_813_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_813_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_813_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_813( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp cyc_dist = stapl::cyclic(size);
  gen_spec_blk_blkcyc_wf nestpar_813_gen_wf(limit,&cyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_813_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);

  stapl::serial_io(nestpar_813_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_813_process_wf(), a_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_813_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP813","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with serial_io I/O,
// process with mapreduce / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_814_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_814_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(get_val_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_814_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_814_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_814_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_814( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blk_dist = stapl::block(size, blk_size);
  gen_spec_cyc_blkcyc_wf nestpar_814_gen_wf(limit,&blk_dist);
  stapl::composed_dist_spec comp_spec(nestpar_814_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);

  stapl::serial_io(nestpar_814_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_814_process_wf(), a_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_814_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP814","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with serial_io I/O,
// process with mapreduce / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_815_fill_wf {
private:
  size_t sz_;
public:
  nestpar_815_fill_wf(size_t s )
    : sz_(s)
  { }
  typedef int result_type;
  template <typename Ref1, typename Ref2, typename Ref3, typename Data>
  result_type operator()(Ref1 lkey, Ref2 rkey, Ref3 val, Data &a) {
    for (size_t i = 0; i < sz_; i++ ) {
      a[lkey[i]][rkey[i]] = val[i];
    }
  }
  void define_type(stapl::typer& t) {
    t.member(sz_);
  }
};

struct nestpar_815_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    return stapl::map_reduce( inner_map_elem_wf(), add_int_wf(), elem.second);
  }
};

size_t nestpar_815( size_t model, int &count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  int limit = count;
  ary_sz_tp lkey(limit), rkey(limit), val(limit);
  ary_sz_vw_tp lkey_vw(lkey), rkey_vw(rkey), val_vw(val);
  stapl::serial_io(get_triple_wf(zin), lkey_vw, rkey_vw, val_vw  );

#ifdef LEN_CONSTR
  ndx_dom_tp map_dom(0, 16777216);
  map_map_int_tp a(map_dom), b(map_dom);
  map_map_int_vw_tp a_vw(a), b_vw(b);
#else
  dist_spec_tp blkcyc_dist = stapl::block_cyclic(size, blk_size);
  gen_spec_blkcyc_blk_wf nestpar_815_gen_wf(limit,&blkcyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_815_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_815_fill_wf(limit), lkey_vw, rkey_vw, val_vw, a_vw);

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_815_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP815","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with serial_io I/O,
// process with map_func / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct user1_wf
{
  typedef void result_type;
  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 x, Ref2 z)
  {
    z = 1 + (x % 10);
  }
};

struct nestpar_816_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_816_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(get_val_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_816_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func( user1_wf(), vw1, vw2 );
  }
};

struct nestpar_816_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_816_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_816( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp bal_dist = stapl::balance(size);
  gen_spec_blkcyc_cyc_wf nestpar_816_gen_wf(limit,&bal_dist);
  stapl::composed_dist_spec comp_spec(nestpar_816_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);
  vec_vec_int_tp b(comp_spec);
  vec_vec_int_vw_tp b_vw(b);

  stapl::serial_io(nestpar_816_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_816_process_wf(), a_vw, b_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_816_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP816","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with serial_io I/O,
// process with map_func / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct user2_wf
{
  typedef void result_type;
  template <typename Ref1, typename Ref2, typename Ref3>
  result_type operator()(const Ref1 x, const Ref2 y, Ref3 z)
  {
    z = (x % 10) + (y % 10);
  }
};

struct nestpar_817_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_817_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const& vw1) const
  {
    stapl::serial_io(get_val_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_817_process_wf {
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const& vw1, View2 const& vw2, View3 const& vw3)
  {
    stapl::map_func( user2_wf(), vw1, vw2, vw3 );
  }
};

struct nestpar_817_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_817_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_817( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 10 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp cyc_dist = stapl::cyclic(size);
  gen_spec_blk_blkcyc_wf nestpar_817_gen_wf(limit,&cyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_817_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);
  ary_ary_int_tp b(comp_spec);
  ary_ary_int_vw_tp b_vw(b);
  ary_ary_int_tp c(comp_spec);
  ary_ary_int_vw_tp c_vw(c);

  stapl::serial_io(nestpar_817_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_817_process_wf(), a_vw, b_vw, c_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_817_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP817","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with serial_io I/O,
// process with map_func / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct user3_wf
{
  typedef void result_type;
  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 x, Ref2 y)
  {
    y.second = 1 + (x.second % 10);
  }
};

struct nestpar_818_fill_wf {
private:
  size_t sz_;
public:
  nestpar_818_fill_wf(size_t s )
    : sz_(s)
  { }
  typedef int result_type;
  template <typename Ref1, typename Ref2, typename Ref3,
            typename Data1, typename Data2>
  result_type operator()(Ref1 lkey, Ref2 rkey, Ref3 val, Data1 &a, Data2 &b) {
    for (size_t i = 0; i < sz_; i++ ) {
      a[lkey[i]][rkey[i]] = val[i];
      b[lkey[i]][rkey[i]] = (val[i] * 10) - 1;
    }
  }
  void define_type(stapl::typer& t) {
    t.member(sz_);
  }
};

struct nestpar_818_process_wf {
  typedef int result_type;
  template <typename Elem1, typename Elem2>
  result_type operator()(Elem1 elem1, Elem2 elem2)
  {
    stapl::map_func( user3_wf(), elem1.second, elem2.second );
  }
};

size_t nestpar_818( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  int limit = count;
  ary_sz_tp lkey(limit), rkey(limit), val(limit);
  ary_sz_vw_tp lkey_vw(lkey), rkey_vw(rkey), val_vw(val);
  stapl::serial_io(get_triple_wf(zin), lkey_vw, rkey_vw, val_vw  );

#ifdef LEN_CONSTR
  ndx_dom_tp map_dom(0, 16777216);
  map_map_int_tp a(map_dom), b(map_dom);
  map_map_int_vw_tp a_vw(a), b_vw(b);
  stapl::do_once(nestpar_818_fill_wf(limit), lkey_vw, rkey_vw, val_vw,
                                            a_vw, b_vw);
#else
  dist_spec_tp blk_dist = stapl::block(size, blk_size);
  gen_spec_cyc_blkcyc_wf nestpar_818_gen_wf(limit,&blk_dist);
  stapl::composed_dist_spec comp_spec(nestpar_818_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_818_fill_wf(limit), lkey_vw, rkey_vw, val_vw,
                                            a_vw, b_vw);

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_818_process_wf(), a_vw, b_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP818","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with serial_io I/O,
// process with partial_sum, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_819_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_819_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(get_val_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_819_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_819_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_819_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_819( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blkcyc_dist = stapl::block_cyclic(size, blk_size);
  gen_spec_blkcyc_blk_wf nestpar_819_gen_wf(limit,&blkcyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_819_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);
  vec_vec_int_tp b(comp_spec);
  vec_vec_int_vw_tp b_vw(b);

  stapl::serial_io(nestpar_819_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_819_process_wf(), a_vw, b_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_819_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP819","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with serial_io I/O,
// process with partial_sum, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_820_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_820_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(get_val_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_820_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_820_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_820_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_820( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp bal_dist = stapl::balance(size);
  gen_spec_blkcyc_cyc_wf nestpar_820_gen_wf(limit,&bal_dist);
  stapl::composed_dist_spec comp_spec(nestpar_820_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);
  ary_ary_int_tp b(comp_spec);
  ary_ary_int_vw_tp b_vw(b);

  stapl::serial_io(nestpar_820_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_820_process_wf(), a_vw, b_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_820_show_wf(zout), b_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP820","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
