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
    case 612:
    case 615:
    case 618:
      path= "data/tiny_factors.zin";
      break;
    default:
      path="data/tiny_primes.zin";
      break;
    }
    break;
  case 100:
    switch( test ) {
    case 612:
    case 615:
    case 618:
      path = "data/small_factors.zin";
      break;
    default:
      path = "data/small_primes.zin";
      break;
    }
    break;
  case 10000:
    switch( test ) {
    case 612:
    case 615:
    case 618:
      path = "data/medium_factors.zin";
      break;
    default:
      path = "data/medium_primes.zin";
      break;
    }
    break;
  case 1000000:
    switch( test ) {
    case 612:
    case 615:
    case 618:
      path = "data/big_factors.zin";
      break;
    default:
      path = "data/big_primes.zin";
      break;
    }
    break;
  case 100000000:
    switch( test ) {
    case 612:
    case 615:
    case 618:
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

size_t nestpar_601( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_602( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_603( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_604( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_605( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_606( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_607( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_608( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_609( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_610( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_611( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_612( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_613( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_614( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_615( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_616( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_617( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_618( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_619( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_620( size_t, int &,
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

  int first_test = 601;
  int last_test = 620;
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
    case 601:
      zout.open("np601.zout");
      result = nestpar_601(model, zin, zout);
      break;
    case 602:
      zout.open("np602.zout");
      result = nestpar_602(model, zin, zout);
      break;
#ifdef MAP
    case 603:
      zout.open("np603.zout");
      result = nestpar_603(model, zin, zout);
      break;
#endif
    case 604:
      zout.open("np604.zout");
      result = nestpar_604(model, zin, zout);
      break;
    case 605:
      zout.open("np605.zout");
      result = nestpar_605(model, zin, zout);
      break;
#ifdef MAP
    case 606:
      zout.open("np606.zout");
      result = nestpar_606(model, zin, zout);
      break;
#endif
    case 607:
      zout.open("np607.zout");
      result = nestpar_607(model, zin, zout);
      break;
    case 608:
      zout.open("np608.zout");
      result = nestpar_608(model, zin, zout);
      break;
#ifdef MAP
    case 609:
      zout.open("np609.zout");
      result = nestpar_609(model, zin, zout);
      break;
#endif
    case 610:
      zout.open("np610.zout");
      if ( open_zin(model,610,zin,count) ) {
        result = nestpar_610(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 611:
      zout.open("np611.zout");
      if ( open_zin(model,611,zin,count) ) {
        result = nestpar_611(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#ifdef MAP
    case 612:
      zout.open("np612.zout");
      if ( open_zin(model,612,zin,count) ) {
        result = nestpar_612(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif
    case 613:
      zout.open("np613.zout");
      if ( open_zin(model,613,zin,count) ) {
        result = nestpar_613(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 614:
      zout.open("np614.zout");
      if ( open_zin(model,614,zin,count) ) {
        result = nestpar_614(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#ifdef MAP
    case 615:
      zout.open("np615.zout");
      if ( open_zin(model,615,zin,count) ) {
        result = nestpar_615(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif
    case 616:
      zout.open("np616.zout");
      if ( open_zin(model,616,zin,count) ) {
        result = nestpar_616(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 617:
      zout.open("np617.zout");
      if ( open_zin(model,617,zin,count) ) {
        result = nestpar_617(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#ifdef MAP
    case 618:
      zout.open("np618.zout");
      if ( open_zin(model,618,zin,count) ) {
        result = nestpar_618(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif
    case 619:
      zout.open("np619.zout");
      if ( open_zin(model,619,zin,count) ) {
        result = nestpar_619(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 620:
      zout.open("np620.zout");
      if ( open_zin(model,620,zin,count) ) {
        result = nestpar_620(model, count, zin, zout);
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
      size_t blk_fac = ( size >= 10 ) ? ( size / 10) : size;
      if (ndx[0] % 2 == 0) {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blk_blkcyc 2: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block(size, blk_fac);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blk_blkcyc 3: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac);
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
        return stapl::cyclic(size);
      } else {
      size_t blk_fac = ( size >= 10 ) ? ( size / 10) : size;
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc 3: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac);
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
      size_t blk_fac = ( size >= 10 ) ? size / 10 : size;
      if ((ndx[0] < (size / 2)) == 0) {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blkcyc_blk 2: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blkcyc_blk 3: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block(size, blk_fac);
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
      if ((ndx[0] < (size / 2)) == 0) {
        size_t blk_fac = ( size >= 10 ) ? ( size / 10) : size;
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blkcyc_cyc 2: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blkcyc_cyc 3: " << size << endl;
});
#endif
        return stapl::cyclic(size);
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

struct nestpar_601_fill_wf {

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::iota( vw1, 0 );
  }
};

struct nestpar_601_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return vw1.size();
  }
};

struct nestpar_601_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_601_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_601( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp cyc_dist = stapl::cyclic(size);
  gen_spec_blk_blkcyc_wf nestpar_601_dist_wf(limit,&cyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_601_dist_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "np601 b: " << endl;
});
#endif

  stapl::map_func(nestpar_601_fill_wf(), a_vw );

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "np601 c: " << endl;
});
#endif

  stapl::map_func(nestpar_601_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_601_show_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP601","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with generator, traverse with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_602_fill_wf {
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

struct nestpar_602_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return vw1.size();
  }
};

struct nestpar_602_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_602_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_602( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blk_dist = stapl::block(size, blk_size);
  gen_spec_cyc_blkcyc_wf nestpar_602_gen_wf(limit,&blk_dist);
  stapl::composed_dist_spec comp_spec(nestpar_602_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);

#ifdef DEBUG_HANG
stapl::do_once([&](){
  cerr << "np602 a: " << endl;
});
#endif

  stapl::map_func(nestpar_602_fill_wf(), a_vw);

#ifdef DEBUG_HANG
stapl::do_once([&](){
  cerr << "np602 b: " << endl;
});
#endif

  stapl::map_func(nestpar_602_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_602_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP602","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with data from C arrays, traverse with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_603_fill_wf {
private:
  size_t outer_;
  size_t inner_;
public:
  nestpar_603_fill_wf(size_t o, size_t i)
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

struct nestpar_603_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    return elem.second.size();
  }
};

size_t nestpar_603( size_t model,
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
  gen_spec_blkcyc_blk_wf nestpar_603_gen_wf(limit,&bal_dist);
  stapl::composed_dist_spec comp_spec(nestpar_603_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_603_fill_wf(size, limit), a_vw);

  stapl::map_func(nestpar_603_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP603","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_604_fill_wf {
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

struct nestpar_604_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_604_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_604_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_604( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blkcyc_dist = stapl::block_cyclic(size, blk_size);
  gen_spec_blkcyc_cyc_wf nestpar_604_gen_wf(limit,&blkcyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_604_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);

  stapl::map_func(nestpar_604_fill_wf(), a_vw );

  stapl::map_func(nestpar_604_process_wf(), a_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_604_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP604","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_605_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
     stapl::generate( vw1, stapl::random_sequence());
  }
};

struct nestpar_605_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), vw1 );
  }
};

struct nestpar_605_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_605_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_605( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp cyc_dist = stapl::cyclic(size);
  gen_spec_cyc_blkcyc_wf nestpar_605_gen_wf(limit,&cyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_605_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);

#ifdef DEBUG_HANG
stapl::do_once([&](){
  cerr << "np605 a: " << endl;
});
#endif

  stapl::map_func(nestpar_605_fill_wf(), a_vw );

#ifdef DEBUG_HANG
stapl::do_once([&](){
  cerr << "np605 b: " << endl;
});
#endif

  stapl::map_func(nestpar_605_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_605_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP605","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map data from C arrays, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_606_fill_wf {
private:
  size_t outer_;
  size_t inner_;
public:
  nestpar_606_fill_wf(size_t o, size_t i)
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

struct nestpar_606_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    return stapl::map_reduce( inner_map_elem_wf(), add_int_wf(), elem.second);
  }
};

size_t nestpar_606( size_t model,
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
  gen_spec_blkcyc_blk_wf nestpar_606_gen_wf(limit,&blk_dist);
  stapl::composed_dist_spec comp_spec(nestpar_606_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_606_fill_wf(size, limit), a_vw);

  stapl::map_func(nestpar_606_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP606","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with generator, process with stapl algorithm
// apply unary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_607_fill_wf {
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

struct nestpar_607_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const& vw2)
  {
    stapl::transform(vw1, vw2, neg_int_wf());
  }
};

struct nestpar_607_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_607_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_607( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blkcyc_dist = stapl::block_cyclic(size, blk_size);
  gen_spec_blkcyc_blk_wf nestpar_607_gen_wf(limit,&blkcyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_607_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);
  vec_vec_int_tp b(comp_spec);
  vec_vec_int_vw_tp b_vw(b);

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "np607 a: " << endl;
});
#endif

  stapl::map_func(nestpar_607_fill_wf(), a_vw );

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "np607 b: " << endl;
});
#endif

  stapl::serial_io(nestpar_607_show_wf(zout), a_vw);

  stapl::map_func(nestpar_607_process_wf(), a_vw, b_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_607_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP607","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with generator, process with stapl algorithm
// apply binary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_608_fill_wf {
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

struct nestpar_608_process_wf {
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const &vw1, View2 const& vw2, View3 const& vw3)
  {
    stapl::transform(vw1, vw2, vw3, add_int_wf());
  }
};

struct nestpar_608_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_608_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_608( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp bal_dist = stapl::balance(size);
  gen_spec_blkcyc_cyc_wf nestpar_608_gen_wf(limit,&bal_dist);
  stapl::composed_dist_spec comp_spec(nestpar_608_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);
  ary_ary_int_tp b(comp_spec);
  ary_ary_int_vw_tp b_vw(b);
  ary_ary_int_tp c(comp_spec);
  ary_ary_int_vw_tp c_vw(c);

#ifdef DEBUG_HANG
stapl::do_once([&](){
  cerr << "np608 a: " << endl;
});
#endif

  stapl::map_func(nestpar_608_fill_wf(), a_vw);
  stapl::map_func(nestpar_608_fill_wf(), b_vw);

#ifdef DEBUG_HANG
stapl::do_once([&](){
  cerr << "np608 b: " << endl;
});
#endif

  stapl::serial_io(nestpar_608_show_wf(zout), a_vw);
  stapl::serial_io(nestpar_608_show_wf(zout), b_vw);

  stapl::map_func(nestpar_608_process_wf(), a_vw, b_vw, c_vw);

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_608_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP608","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with data from C arrays,
// process with stapl algorithm
//////////////////////////////////////////////////////////////////////

struct nestpar_609_fill_wf {
private:
  size_t outer_;
  size_t inner_;
public:
  nestpar_609_fill_wf(size_t o, size_t i)
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

struct nestpar_609_process_wf {
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

size_t nestpar_609( size_t model,
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
  gen_spec_blk_blkcyc_wf nestpar_609_gen_wf(limit,&cyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_609_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_609_fill_wf(size, limit), a_vw);

  stapl::map_func(nestpar_609_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP609","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with serial_io I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_610_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_610_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_610_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const& vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_610_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_610_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_610( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blk_dist = stapl::block(size, blk_size);
  gen_spec_cyc_blkcyc_wf nestpar_610_gen_wf(limit,&blk_dist);
  stapl::composed_dist_spec comp_spec(nestpar_610_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);

  stapl::serial_io(nestpar_610_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_610_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP610","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with serial_io I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_611_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_611_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_611_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_611_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_611_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_611( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blkcyc_dist = stapl::block_cyclic(size, blk_size);
  gen_spec_blkcyc_blk_wf nestpar_611_gen_wf(limit,&blkcyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_611_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);
  ary_ary_int_tp b(comp_spec);
  ary_ary_int_vw_tp b_vw(b);

  stapl::serial_io(nestpar_611_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_611_process_wf(), a_vw, b_vw);

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_611_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP611","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with serial_io I/O, display map
//////////////////////////////////////////////////////////////////////

struct nestpar_612_fill_wf {
private:
  size_t sz_;
public:
  nestpar_612_fill_wf(size_t s )
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

size_t nestpar_612( size_t model, int & count,
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
  gen_spec_blkcyc_cyc_wf nestpar_612_gen_wf(limit,&bal_dist);
  stapl::composed_dist_spec comp_spec(nestpar_612_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_612_fill_wf(limit), lkey_vw, rkey_vw, val_vw, a_vw);

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP612","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with serial_io I/O,
// process with mapreduce / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_613_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_613_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_613_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_613_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_613_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_613( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp cyc_dist = stapl::cyclic(size);
  gen_spec_blk_blkcyc_wf nestpar_613_gen_wf(limit,&cyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_613_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);

  stapl::serial_io(nestpar_613_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_613_process_wf(), a_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_613_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP613","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with serial_io I/O,
// process with mapreduce / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_614_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_614_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_614_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_614_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_614_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_614( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blk_dist = stapl::block(size, blk_size);
  gen_spec_cyc_blkcyc_wf nestpar_614_gen_wf(limit,&blk_dist);
  stapl::composed_dist_spec comp_spec(nestpar_614_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);

  stapl::serial_io(nestpar_614_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_614_process_wf(), a_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_614_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP614","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with serial_io I/O,
// process with mapreduce / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_615_fill_wf {
private:
  size_t sz_;
public:
  nestpar_615_fill_wf(size_t s )
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

struct nestpar_615_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    return stapl::map_reduce( inner_map_elem_wf(), add_int_wf(), elem.second);
  }
};

size_t nestpar_615( size_t model, int &count,
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
  gen_spec_blkcyc_blk_wf nestpar_615_gen_wf(limit,&blkcyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_615_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_615_fill_wf(limit), lkey_vw, rkey_vw, val_vw, a_vw);

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_615_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP615","STAPL",time_p,time_i+time_o);

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

struct nestpar_616_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_616_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_616_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func( user1_wf(), vw1, vw2 );
  }
};

struct nestpar_616_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_616_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_616( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp bal_dist = stapl::balance(size);
  gen_spec_blkcyc_cyc_wf nestpar_616_gen_wf(limit,&bal_dist);
  stapl::composed_dist_spec comp_spec(nestpar_616_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);
  vec_vec_int_tp b(comp_spec);
  vec_vec_int_vw_tp b_vw(b);

  stapl::serial_io(nestpar_616_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_616_process_wf(), a_vw, b_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_616_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP616","STAPL",time_p,time_i+time_o);

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

struct nestpar_617_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_617_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_617_process_wf {
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const& vw1, View2 const& vw2, View3 const& vw3)
  {
    stapl::map_func( user2_wf(), vw1, vw2, vw3 );
  }
};

struct nestpar_617_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_617_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_617( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 10 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp cyc_dist = stapl::cyclic(size);
  gen_spec_blk_blkcyc_wf nestpar_617_gen_wf(limit,&cyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_617_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);
  ary_ary_int_tp b(comp_spec);
  ary_ary_int_vw_tp b_vw(b);
  ary_ary_int_tp c(comp_spec);
  ary_ary_int_vw_tp c_vw(c);

  stapl::serial_io(nestpar_617_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_617_process_wf(), a_vw, b_vw, c_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_617_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP617","STAPL",time_p,time_i+time_o);

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

struct nestpar_618_fill_wf {
private:
  size_t sz_;
public:
  nestpar_618_fill_wf(size_t s )
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

struct nestpar_618_process_wf {
  typedef int result_type;
  template <typename Elem1, typename Elem2>
  result_type operator()(Elem1 elem1, Elem2 elem2)
  {
    stapl::map_func( user3_wf(), elem1.second, elem2.second );
  }
};

size_t nestpar_618( size_t model, int & count,
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
  stapl::do_once(nestpar_618_fill_wf(limit), lkey_vw, rkey_vw, val_vw,
                                            a_vw, b_vw);
#else
  dist_spec_tp blk_dist = stapl::block(size, blk_size);
  gen_spec_cyc_blkcyc_wf nestpar_618_gen_wf(limit,&blk_dist);
  stapl::composed_dist_spec comp_spec(nestpar_618_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_618_fill_wf(limit), lkey_vw, rkey_vw, val_vw,
                                            a_vw, b_vw);

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_618_process_wf(), a_vw, b_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP618","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with serial_io I/O,
// process with partial_sum, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_619_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_619_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_619_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_619_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_619_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_619( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blkcyc_dist = stapl::block_cyclic(size, blk_size);
  gen_spec_blkcyc_blk_wf nestpar_619_gen_wf(limit,&blkcyc_dist);
  stapl::composed_dist_spec comp_spec(nestpar_619_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);
  vec_vec_int_tp b(comp_spec);
  vec_vec_int_vw_tp b_vw(b);

  stapl::serial_io(nestpar_619_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_619_process_wf(), a_vw, b_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_619_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP619","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with serial_io I/O,
// process with partial_sum, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_620_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_620_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_620_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_620_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_620_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_620( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp bal_dist = stapl::balance(size);
  gen_spec_blkcyc_cyc_wf nestpar_620_gen_wf(limit,&bal_dist);
  stapl::composed_dist_spec comp_spec(nestpar_620_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);
  ary_ary_int_tp b(comp_spec);
  ary_ary_int_vw_tp b_vw(b);

  stapl::serial_io(nestpar_620_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_620_process_wf(), a_vw, b_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_620_show_wf(zout), b_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP620","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
