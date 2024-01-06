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
#include <algorithm>

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
    case 712:
    case 715:
    case 718:
      path= "data/tiny_factors.zin";
      break;
    default:
      path="data/tiny_primes.zin";
      break;
    }
    break;
  case 100:
    switch( test ) {
    case 712:
    case 715:
    case 718:
      path = "data/small_factors.zin";
      break;
    default:
      path = "data/small_primes.zin";
      break;
    }
    break;
  case 10000:
    switch( test ) {
    case 712:
    case 715:
    case 718:
      path = "data/medium_factors.zin";
      break;
    default:
      path = "data/medium_primes.zin";
      break;
    }
    break;
  case 1000000:
    switch( test ) {
    case 712:
    case 715:
    case 718:
      path = "data/big_factors.zin";
      break;
    default:
      path = "data/big_primes.zin";
      break;
    }
    break;
  case 100000000:
    switch( test ) {
    case 712:
    case 715:
    case 718:
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

size_t nestpar_701( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_702( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_703( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_704( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_705( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_706( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_707( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_708( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_709( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_710( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_711( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_712( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_713( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_714( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_715( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_716( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_717( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_718( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_719( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_720( size_t, int &,
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

  int first_test = 701;
  int last_test = 720;
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
    case 701:
      zout.open("np701.zout");
      result = nestpar_701(model, zin, zout);
      break;
    case 702:
      zout.open("np702.zout");
      result = nestpar_702(model, zin, zout);
      break;
#ifdef MAP
    case 703:
      zout.open("np703.zout");
      result = nestpar_703(model, zin, zout);
      break;
#endif
    case 704:
      zout.open("np704.zout");
      result = nestpar_704(model, zin, zout);
      break;
    case 705:
      zout.open("np705.zout");
      result = nestpar_705(model, zin, zout);
      break;
#ifdef MAP
    case 706:
      zout.open("np706.zout");
      result = nestpar_706(model, zin, zout);
      break;
#endif
    case 707:
      zout.open("np707.zout");
      result = nestpar_707(model, zin, zout);
      break;
    case 708:
      zout.open("np708.zout");
      result = nestpar_708(model, zin, zout);
      break;
#ifdef MAP
    case 709:
      zout.open("np709.zout");
      result = nestpar_709(model, zin, zout);
      break;
#endif
    case 710:
      zout.open("np710.zout");
      if ( open_zin(model,710,zin,count) ) {
        result = nestpar_710(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 711:
      zout.open("np711.zout");
      if ( open_zin(model,711,zin,count) ) {
        result = nestpar_711(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#ifdef MAP
    case 712:
      zout.open("np712.zout");
      if ( open_zin(model,712,zin,count) ) {
        result = nestpar_712(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif
    case 713:
      zout.open("np713.zout");
      if ( open_zin(model,713,zin,count) ) {
        result = nestpar_713(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 714:
      zout.open("np714.zout");
      if ( open_zin(model,714,zin,count) ) {
        result = nestpar_714(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#ifdef MAP
    case 715:
      zout.open("np715.zout");
      if ( open_zin(model,715,zin,count) ) {
        result = nestpar_715(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif
    case 716:
      zout.open("np716.zout");
      if ( open_zin(model,716,zin,count) ) {
        result = nestpar_716(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 717:
      zout.open("np717.zout");
      if ( open_zin(model,717,zin,count) ) {
        result = nestpar_717(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#ifdef MAP
    case 718:
      zout.open("np718.zout");
      if ( open_zin(model,718,zin,count) ) {
        result = nestpar_718(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif
    case 719:
      zout.open("np719.zout");
      if ( open_zin(model,719,zin,count) ) {
        result = nestpar_719(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 720:
      zout.open("np720.zout");
      if ( open_zin(model,720,zin,count) ) {
        result = nestpar_720(model, count, zin, zout);
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
  stapl::location_type m_num_locs;
public:
  gen_spec_blk_blkcyc_wf(size_t l, dist_spec_tp *od, stapl::location_type nl )
    : m_limit(l), m_outer_dist(od), m_num_locs(nl)
  { }
  typedef dist_spec_tp result_type;
  result_type operator()(vector<size_t> const& ndx) const
  {
    size_t size = 1 + (rand() % m_limit);
    size_t blk_fac = ( size >= 10 ) ? (size / 10) : size;

    vector<stapl::location_type> red_locs(m_num_locs/2),
                                 green_locs(m_num_locs/2);
    iota(red_locs.begin(), red_locs.end(), 0);
    iota(green_locs.begin(), green_locs.end(), 0);
    for_each(red_locs.begin(), red_locs.end(), bind(multiplies<int>(),_1,2));
    for_each(green_locs.begin(), green_locs.end(),
             bind(multiplies<int>(),_1,2));
    for_each(green_locs.begin(), green_locs.end(), bind(plus<int>(),_1,1));

    if ( ndx.empty() ) {
      return *m_outer_dist;
    } else if ( ndx.size() == 1 ) {
      if (ndx[0] % 2 != 0) {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blk_blkcyc_cyc red 2: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac, red_locs);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blk_blkcyc_cyc green 2: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block(size, blk_fac, green_locs);
      }
    } else if ( ndx.size() == 2 ) {
      if (ndx[0] % 2 == 0) {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blk_blkcyc_cyc red 3: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block(size, blk_fac, red_locs);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "blk_blkcyc_cyc green 3: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac, green_locs);
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
  stapl::location_type m_num_locs;
public:
  gen_spec_cyc_blkcyc_wf(size_t l, dist_spec_tp *od, stapl::location_type nl )
    : m_limit(l), m_outer_dist(od), m_num_locs(nl)
  { }

  typedef dist_spec_tp result_type;
  result_type operator()(vector<size_t> const& ndx) const
  {
    size_t size = 1 + (rand() % m_limit);
    size_t blk_fac = ( size >= 10 ) ? (size / 10) : size;
    size_t half = size/2;
    vector<stapl::location_type> red_locs(m_num_locs/2),
                                 green_locs(m_num_locs/2);
    iota(red_locs.begin(), red_locs.end(), 0);
    iota(green_locs.begin(), green_locs.end(), 0);
    for_each(green_locs.begin(), green_locs.end(), bind(plus<int>(),_1,half));

    if ( ndx.empty() ) {
      return *m_outer_dist;
    } else if ( ndx.size() == 1 ) {
      if (ndx[0] % 2 != 0) {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc_cyc red 2: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac, red_locs);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc_cyc green 2: " << size << endl;
});
#endif
        return stapl::cyclic(size, green_locs);
      }
    } else if ( ndx.size() == 2 ) {
      if (ndx[1] % 2 == 0) {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc_cyc red 3: " << size << endl;
});
#endif
        return stapl::cyclic(size, red_locs);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc_cyc green 3: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac, green_locs);
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
  stapl::location_type m_num_locs;
public:
  gen_spec_blkcyc_blk_wf(size_t l, dist_spec_tp *od, stapl::location_type nl )
    : m_limit(l), m_outer_dist(od), m_num_locs(nl)
  { }

  typedef dist_spec_tp result_type;
  result_type operator()(vector<size_t> const& ndx) const
  {
    size_t size = 1 + (rand() % m_limit);
    size_t blk_fac = ( size >= 10 ) ? (size / 10) : size;
    size_t half = size/2;
    vector<stapl::location_type> red_locs(m_num_locs/2),
                                 green_locs(m_num_locs/2);
    iota(red_locs.begin(), red_locs.end(), 0);
    iota(green_locs.begin(), green_locs.end(), 0);
    for_each(green_locs.begin(), green_locs.end(), bind(plus<int>(),_1,half));

    if ( ndx.empty() ) {
      return *m_outer_dist;
    } else if ( ndx.size() == 1 ) {
      if ((ndx[0] < (size / 2)) != 0) {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc_blk red 2: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac, red_locs);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc_blk green 2: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block(size, blk_fac, green_locs);
      }
    } else if ( ndx.size() == 2 ) {
      if ((ndx[1] < (size / 2)) == 0) {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc_blk red 3: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block(size, blk_fac, red_locs);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc_blk green 3: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac, green_locs);
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
  stapl::location_type m_num_locs;
public:
  gen_spec_blkcyc_cyc_wf(size_t l, dist_spec_tp *od, stapl::location_type nl )
    : m_limit(l), m_outer_dist(od), m_num_locs(nl)
  { }

  typedef dist_spec_tp result_type;
  result_type operator()(vector<size_t> const& ndx) const
  {
    size_t size = 1 + (rand() % m_limit);
    size_t blk_fac = ( size >= 10 ) ? (size / 10) : size;
    size_t half = size/2;
    vector<stapl::location_type> red_locs(m_num_locs/2),
                                 green_locs(m_num_locs/2);
    iota(red_locs.begin(), red_locs.end(), 0);
    iota(green_locs.begin(), green_locs.end(), 0);
    for_each(red_locs.begin(), red_locs.end(), bind(multiplies<int>(),_1,2));
    for_each(green_locs.begin(), green_locs.end(),
             bind(multiplies<int>(),_1,2));
    for_each(green_locs.begin(), green_locs.end(), bind(plus<int>(),_1,1));

    if ( ndx.empty() ) {
      return *m_outer_dist;
    } else if ( ndx.size() == 1 ) {
      if ((ndx[0] < (size / 2)) != 0) {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc_cyc red 2: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac, red_locs);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc_cyc green 2: " << size << endl;
});
#endif
        return stapl::cyclic(size, green_locs);
      }
    } else if ( ndx.size() == 2 ) {
      if ((ndx[1] < (size / 2)) == 0) {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc_cyc red 3: " << size << endl;
});
#endif
        return stapl::cyclic(size, red_locs);
      } else {
#ifdef DEBUG_GEN_SPEC
stapl::do_once([&](){
  cerr << "cyc_blkcyc_cyc green 3: " << size << " " << blk_fac << endl;
});
#endif
        return stapl::block_cyclic(size, blk_fac, green_locs);
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

struct nestpar_701_fill_wf {

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::iota( vw1, 0 );
  }
};

struct nestpar_701_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return vw1.size();
  }
};

struct nestpar_701_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_701_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_701( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "nestpar701 a" << endl;
});
#endif

  dist_spec_tp cyc_dist = stapl::cyclic(size);
  gen_spec_blk_blkcyc_wf nestpar_701_dist_wf(limit,&cyc_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_701_dist_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "nestpar701 b" << endl;
});
#endif

  stapl::map_func(nestpar_701_fill_wf(), a_vw );

  stapl::map_func(nestpar_701_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_701_show_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP701","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with generator, traverse with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_702_fill_wf {
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

struct nestpar_702_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return vw1.size();
  }
};

struct nestpar_702_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_702_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_702( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

stapl::do_once([&](){
  cerr << "nestpar702 a" << endl;
});

  dist_spec_tp blk_dist = stapl::block(size, blk_size);
  gen_spec_cyc_blkcyc_wf nestpar_702_gen_wf(limit,&blk_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_702_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "nestpar702 b" << endl;
});
#endif

  stapl::map_func(nestpar_702_fill_wf(), a_vw);

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "nestpar702 c" << endl;
});
#endif

  stapl::map_func(nestpar_702_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_702_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP702","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with data from C arrays, traverse with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_703_fill_wf {
private:
  size_t outer_;
  size_t inner_;
public:
  nestpar_703_fill_wf(size_t o, size_t i)
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

struct nestpar_703_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    return elem.second.size();
  }
};

size_t nestpar_703( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
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
  gen_spec_blkcyc_blk_wf nestpar_703_gen_wf(limit,&bal_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_703_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_703_fill_wf(size, limit), a_vw);

  stapl::map_func(nestpar_703_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP703","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_704_fill_wf {
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

struct nestpar_704_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_704_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_704_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_704( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "nestpar704 a" << endl;
});
#endif

  dist_spec_tp blkcyc_dist = stapl::block_cyclic(size, blk_size);
  gen_spec_blkcyc_cyc_wf nestpar_704_gen_wf(limit,&blkcyc_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_704_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "nestpar704 b" << endl;
});
#endif

  stapl::map_func(nestpar_704_fill_wf(), a_vw );

  stapl::map_func(nestpar_704_process_wf(), a_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_704_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP704","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_705_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
     stapl::generate( vw1, stapl::random_sequence());
  }
};

struct nestpar_705_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), vw1 );
  }
};

struct nestpar_705_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_705_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_705( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

stapl::do_once([&](){
  cerr << "nestpar705 a" << endl;
});

  dist_spec_tp cyc_dist = stapl::cyclic(size);
  gen_spec_cyc_blkcyc_wf nestpar_705_gen_wf(limit,&cyc_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_705_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "nestpar705 b" << endl;
});
#endif

  stapl::map_func(nestpar_705_fill_wf(), a_vw );

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "nestpar705 c" << endl;
});
#endif

  stapl::map_func(nestpar_705_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_705_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP705","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map data from C arrays, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_706_fill_wf {
private:
  size_t outer_;
  size_t inner_;
public:
  nestpar_706_fill_wf(size_t o, size_t i)
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

struct nestpar_706_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    return stapl::map_reduce( inner_map_elem_wf(), add_int_wf(), elem.second);
  }
};

size_t nestpar_706( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
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
  gen_spec_blkcyc_blk_wf nestpar_706_gen_wf(limit,&blk_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_706_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_706_fill_wf(size, limit), a_vw);

  stapl::map_func(nestpar_706_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP706","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with generator, process with stapl algorithm
// apply unary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_707_fill_wf {
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

struct nestpar_707_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const& vw2)
  {
    stapl::transform(vw1, vw2, neg_int_wf());
  }
};

struct nestpar_707_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_707_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_707( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "nestpar707 a" << endl;
});
#endif

  dist_spec_tp blkcyc_dist = stapl::block_cyclic(size, blk_size);
  gen_spec_blkcyc_blk_wf nestpar_707_gen_wf(limit,&blkcyc_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_707_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);
  vec_vec_int_tp b(comp_spec);
  vec_vec_int_vw_tp b_vw(b);

#ifdef DEBUG_INV_RANK
stapl::do_once([&](){
  cerr << "nestpar707 b" << endl;
});
#endif

  stapl::map_func(nestpar_707_fill_wf(), a_vw );

  stapl::serial_io(nestpar_707_show_wf(zout), a_vw);

  stapl::map_func(nestpar_707_process_wf(), a_vw, b_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_707_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP707","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with generator, process with stapl algorithm
// apply binary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_708_fill_wf {
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

struct nestpar_708_process_wf {
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const &vw1, View2 const& vw2, View3 const& vw3)
  {
    stapl::transform(vw1, vw2, vw3, add_int_wf());
  }
};

struct nestpar_708_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_708_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_708( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

stapl::do_once([&](){
  cerr << "nestpar708 a" << endl;
});

  dist_spec_tp bal_dist = stapl::balance(size);
  gen_spec_blkcyc_cyc_wf nestpar_708_gen_wf(limit,&bal_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_708_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);
  ary_ary_int_tp b(comp_spec);
  ary_ary_int_vw_tp b_vw(b);
  ary_ary_int_tp c(comp_spec);
  ary_ary_int_vw_tp c_vw(c);

stapl::do_once([&](){
  cerr << "nestpar708 b" << endl;
});

  stapl::map_func(nestpar_708_fill_wf(), a_vw);
  stapl::map_func(nestpar_708_fill_wf(), b_vw);

  stapl::serial_io(nestpar_708_show_wf(zout), a_vw);
  stapl::serial_io(nestpar_708_show_wf(zout), b_vw);

  stapl::map_func(nestpar_708_process_wf(), a_vw, b_vw, c_vw);

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_708_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP708","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with data from C arrays,
// process with stapl algorithm
//////////////////////////////////////////////////////////////////////

struct nestpar_709_fill_wf {
private:
  size_t outer_;
  size_t inner_;
public:
  nestpar_709_fill_wf(size_t o, size_t i)
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

struct nestpar_709_process_wf {
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

size_t nestpar_709( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
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
  gen_spec_blk_blkcyc_wf nestpar_709_gen_wf(limit,&cyc_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_709_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_709_fill_wf(size, limit), a_vw);

  stapl::map_func(nestpar_709_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP709","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with serial_io I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_710_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_710_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_710_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const& vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_710_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_710_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_710( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blk_dist = stapl::block(size, blk_size);
  gen_spec_cyc_blkcyc_wf nestpar_710_gen_wf(limit,&blk_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_710_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);

  stapl::serial_io(nestpar_710_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_710_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP710","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with serial_io I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_711_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_711_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_711_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_711_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_711_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_711( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blkcyc_dist = stapl::block_cyclic(size, blk_size);
  gen_spec_blkcyc_blk_wf nestpar_711_gen_wf(limit,&blkcyc_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_711_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);
  ary_ary_int_tp b(comp_spec);
  ary_ary_int_vw_tp b_vw(b);

  stapl::serial_io(nestpar_711_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_711_process_wf(), a_vw, b_vw);

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_711_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP711","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with serial_io I/O, display map
//////////////////////////////////////////////////////////////////////

struct nestpar_712_fill_wf {
private:
  size_t sz_;
public:
  nestpar_712_fill_wf(size_t s )
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

size_t nestpar_712( size_t model, int & count,
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
  gen_spec_blkcyc_cyc_wf nestpar_712_gen_wf(limit,&bal_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_712_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_712_fill_wf(limit), lkey_vw, rkey_vw, val_vw, a_vw);

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP712","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with serial_io I/O,
// process with mapreduce / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_713_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_713_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_713_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_713_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_713_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_713( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp cyc_dist = stapl::cyclic(size);
  gen_spec_blk_blkcyc_wf nestpar_713_gen_wf(limit,&cyc_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_713_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);

  stapl::serial_io(nestpar_713_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_713_process_wf(), a_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_713_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP713","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with serial_io I/O,
// process with mapreduce / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_714_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_714_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_714_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_714_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_714_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_714( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blk_dist = stapl::block(size, blk_size);
  gen_spec_cyc_blkcyc_wf nestpar_714_gen_wf(limit,&blk_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_714_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);

  stapl::serial_io(nestpar_714_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_714_process_wf(), a_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_714_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP714","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with serial_io I/O,
// process with mapreduce / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_715_fill_wf {
private:
  size_t sz_;
public:
  nestpar_715_fill_wf(size_t s )
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

struct nestpar_715_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    return stapl::map_reduce( inner_map_elem_wf(), add_int_wf(), elem.second);
  }
};

size_t nestpar_715( size_t model, int &count,
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
  gen_spec_blkcyc_blk_wf nestpar_715_gen_wf(limit,&blkcyc_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_715_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_715_fill_wf(limit), lkey_vw, rkey_vw, val_vw, a_vw);

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_715_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP715","STAPL",time_p,time_i+time_o);

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

struct nestpar_716_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_716_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_716_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func( user1_wf(), vw1, vw2 );
  }
};

struct nestpar_716_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_716_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_716( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp bal_dist = stapl::balance(size);
  gen_spec_blkcyc_cyc_wf nestpar_716_gen_wf(limit,&bal_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_716_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);
  vec_vec_int_tp b(comp_spec);
  vec_vec_int_vw_tp b_vw(b);

  stapl::serial_io(nestpar_716_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_716_process_wf(), a_vw, b_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_716_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP716","STAPL",time_p,time_i+time_o);

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

struct nestpar_717_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_717_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_717_process_wf {
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const& vw1, View2 const& vw2, View3 const& vw3)
  {
    stapl::map_func( user2_wf(), vw1, vw2, vw3 );
  }
};

struct nestpar_717_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_717_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_717( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 10 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp cyc_dist = stapl::cyclic(size);
  gen_spec_blk_blkcyc_wf nestpar_717_gen_wf(limit,&cyc_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_717_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);
  ary_ary_int_tp b(comp_spec);
  ary_ary_int_vw_tp b_vw(b);
  ary_ary_int_tp c(comp_spec);
  ary_ary_int_vw_tp c_vw(c);

  stapl::serial_io(nestpar_717_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_717_process_wf(), a_vw, b_vw, c_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_717_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP717","STAPL",time_p,time_i+time_o);

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

struct nestpar_718_fill_wf {
private:
  size_t sz_;
public:
  nestpar_718_fill_wf(size_t s )
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

struct nestpar_718_process_wf {
  typedef int result_type;
  template <typename Elem1, typename Elem2>
  result_type operator()(Elem1 elem1, Elem2 elem2)
  {
    stapl::map_func( user3_wf(), elem1.second, elem2.second );
  }
};

size_t nestpar_718( size_t model, int & count,
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
  stapl::do_once(nestpar_718_fill_wf(limit), lkey_vw, rkey_vw, val_vw,
                                            a_vw, b_vw);
#else
  dist_spec_tp blk_dist = stapl::block(size, blk_size);
  gen_spec_cyc_blkcyc_wf nestpar_718_gen_wf(limit,&blk_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_718_gen_wf);
  map_map_int_tp a(comp_spec);
  map_map_int_vw_tp a_vw(a);
#endif

  stapl::do_once(nestpar_718_fill_wf(limit), lkey_vw, rkey_vw, val_vw,
                                            a_vw, b_vw);

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_718_process_wf(), a_vw, b_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP718","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
#endif

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with serial_io I/O,
// process with partial_sum, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_719_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_719_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_719_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_719_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_719_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_719( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp blkcyc_dist = stapl::block_cyclic(size, blk_size);
  gen_spec_blkcyc_blk_wf nestpar_719_gen_wf(limit,&blkcyc_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_719_gen_wf);
  vec_vec_int_tp a(comp_spec);
  vec_vec_int_vw_tp a_vw(a);
  vec_vec_int_tp b(comp_spec);
  vec_vec_int_vw_tp b_vw(b);

  stapl::serial_io(nestpar_719_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_719_process_wf(), a_vw, b_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_719_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP719","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with serial_io I/O,
// process with partial_sum, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_720_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_720_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_720_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_720_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_720_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_720( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::location_type num_locs = stapl::get_num_locations();
  size_t size = 100 * model;
  size_t limit = 100 * model;
  size_t blk_size = size / (2 * stapl::get_num_locations() );

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  dist_spec_tp bal_dist = stapl::balance(size);
  gen_spec_blkcyc_cyc_wf nestpar_720_gen_wf(limit,&bal_dist,num_locs);
  stapl::composed_dist_spec comp_spec(nestpar_720_gen_wf);
  ary_ary_int_tp a(comp_spec);
  ary_ary_int_vw_tp a_vw(a);
  ary_ary_int_tp b(comp_spec);
  ary_ary_int_vw_tp b_vw(b);

  stapl::serial_io(nestpar_720_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_720_process_wf(), a_vw, b_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_720_show_wf(zout), b_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP720","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
