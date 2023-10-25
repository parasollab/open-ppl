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
#include <vector>

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

#if 1
// Uncomment these defines when the memory footprint of compilation has been
// reduced.  Currently, nestpar3_homogeneous.cc defines HOMOGENEOUS and then
// includes this file, while nestpar3_heterogeneous.cc defines HETEROGENEOUS
// and includes the file.
#define HOMOGENEOUS 1
#define HETEROGENEOUS 1
#endif

#define GEN_INPUT 1
#define FILE_INPUT 1

#undef MAP
#undef LEN_CONSTR

/*=========================================================================*/

typedef stapl::indexed_domain<int> ndx_dom_tp;

typedef stapl::array<size_t> ary_sz_tp;
typedef stapl::array<ary_sz_tp> ary2_sz_tp;

typedef stapl::array_view<ary_sz_tp> ary_sz_vw_tp;
typedef stapl::array_view<ary2_sz_tp> ary2_sz_vw_tp;

typedef stapl::vector<size_t> vec_sz_tp;
typedef stapl::vector< vec_sz_tp > vec2_sz_tp;

typedef stapl::vector_view< vec_sz_tp > vec_sz_vw_tp;
typedef stapl::vector_view< vec2_sz_tp > vec2_sz_vw_tp;

typedef stapl::indexed_domain<int> ndx_dom_tp;

typedef stapl::distribution_spec<> dist_spec_tp;
typedef vector<dist_spec_tp> vec_dist_spec_tp;

/*=========================================================================*/

//typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector<int,
                      stapl::view_based_partition<dist_spec_tp>,
                      stapl::view_based_mapper<dist_spec_tp> > vec_int_tp;

//typedef stapl::vector< vec_int_tp > vec2_int_tp;
typedef stapl::vector<vec_int_tp,
                      stapl::view_based_partition<dist_spec_tp>,
                      stapl::view_based_mapper<dist_spec_tp> > vec2_int_tp;

//typedef stapl::vector< vec2_int_tp > vec3_int_tp;
typedef stapl::vector<vec2_int_tp,
                      stapl::view_based_partition<dist_spec_tp>,
                      stapl::view_based_mapper<dist_spec_tp> > vec3_int_tp;

//typedef stapl::array<int> ary_int_tp;
typedef stapl::array<int,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> > ary_int_tp;

//typedef stapl::array< ary_int_tp > ary2_int_tp;
typedef stapl::array< ary_int_tp,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> > ary2_int_tp;

//typedef stapl::array< ary2_int_tp > ary3_int_tp;
typedef stapl::array< ary2_int_tp,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> > ary3_int_tp;

//typedef stapl::map<int,int> map_int_tp;
typedef stapl::map< int, int,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> > map_int_tp;

//typedef stapl::map< int, stapl::map<int,int> > map2_int_tp;
typedef stapl::map< int, map_int_tp,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> > map2_int_tp;

//typedef stapl::vector<ary_int_tp> vec_ary_int_tp;
typedef stapl::vector<ary_int_tp,
                      stapl::view_based_partition<dist_spec_tp>,
                      stapl::view_based_mapper<dist_spec_tp> > vec_ary_int_tp;

//typedef stapl::array<vec_int_tp> ary_vec_int_tp;
typedef stapl::array<vec_int_tp ,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> > ary_vec_int_tp;

//typedef stapl::array<vec_ary_int_tp> ary_vec_ary_int_tp;
typedef stapl::array<vec_ary_int_tp ,
  stapl::view_based_partition<dist_spec_tp>,
  stapl::view_based_mapper<dist_spec_tp> > ary_vec_ary_int_tp;

//typedef stapl::vector<ary_vec_int_tp> vec_ary_vec_int_tp;
typedef stapl::vector<ary_vec_int_tp ,
  stapl::view_based_partition<dist_spec_tp>,
  stapl::view_based_mapper<dist_spec_tp> > vec_ary_vec_int_tp;

//typedef stapl::array<map_int_tp> ary_map_int_tp;
typedef stapl::array<map_int_tp,
                     stapl::view_based_partition<dist_spec_tp>,
                     stapl::view_based_mapper<dist_spec_tp> > ary_map_int_tp;

//typedef stapl::vector<ary_map_int_tp> vec_ary_map_int_tp;
typedef stapl::vector<ary_map_int_tp ,
  stapl::view_based_partition<dist_spec_tp>,
  stapl::view_based_mapper<dist_spec_tp> > vec_ary_map_int_tp;

//typedef stapl::vector<map_int_tp> vec_map_int_tp;
typedef stapl::vector<map_int_tp,
                      stapl::view_based_partition<dist_spec_tp>,
                      stapl::view_based_mapper<dist_spec_tp> > vec_map_int_tp;

//typedef stapl::array<vec_map_int_tp> ary_vec_map_int_tp;
typedef stapl::array<vec_map_int_tp,
  stapl::view_based_partition<dist_spec_tp>,
  stapl::view_based_mapper<dist_spec_tp> > ary_vec_map_int_tp;

//typedef stapl::map<int,ary_int_tp> map_ary_int_tp;
typedef stapl::map<int, ary_int_tp,
                   stapl::view_based_partition<dist_spec_tp>,
                   stapl::view_based_mapper<dist_spec_tp> > map_ary_int_tp;

//typedef stapl::vector<map_ary_int_tp> vec_map_ary_int_tp;
typedef stapl::vector<map_ary_int_tp,
  stapl::view_based_partition<dist_spec_tp>,
  stapl::view_based_mapper<dist_spec_tp> > vec_map_ary_int_tp;

//typedef stapl::map<int,ary_vec_int_tp> map_ary_vec_int_tp;
typedef stapl::map<int,ary_vec_int_tp,
  stapl::view_based_partition<dist_spec_tp>,
  stapl::view_based_mapper<dist_spec_tp> > map_ary_vec_int_tp;

//typedef stapl::map<int,map_int_tp> map_map_int_tp;
typedef stapl::map<int, map_int_tp,
                   stapl::view_based_partition<dist_spec_tp>,
                   stapl::view_based_mapper<dist_spec_tp> > map_map_int_tp;


//typedef stapl::vector<map_map_int_tp> vec_map_map_int_tp;
typedef stapl::vector<map_map_int_tp,
  stapl::view_based_partition<dist_spec_tp>,
  stapl::view_based_mapper<dist_spec_tp> > vec_map_map_int_tp;

//typedef stapl::map<int,vec_map_int_tp> map_vec_map_int_tp;
typedef stapl::map<vec_map_int_tp,
  stapl::view_based_partition<dist_spec_tp>,
  stapl::view_based_mapper<dist_spec_tp> > map_vec_map_int_tp;

//typedef stapl::map<int,vec_ary_int_tp> map_vec_ary_int_tp;
typedef stapl::map<vec_ary_int_tp,
  stapl::view_based_partition<dist_spec_tp>,
  stapl::view_based_mapper<dist_spec_tp> > map_vec_ary_int_tp;

//typedef stapl::vector<map_map_int_tp> vec_map_map_int_tp;
typedef stapl::map<int, map_map_int_tp,
  stapl::view_based_partition<dist_spec_tp>,
  stapl::view_based_mapper<dist_spec_tp> > map_map_map_int_tp;

/*=========================================================================*/

typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;
typedef stapl::vector_view<vec2_int_tp> vec2_int_vw_tp;
typedef stapl::vector_view<vec3_int_tp> vec3_int_vw_tp;

typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;
typedef stapl::array_view<ary2_int_tp> ary2_int_vw_tp;
typedef stapl::array_view<ary3_int_tp> ary3_int_vw_tp;

typedef stapl::map_view<map_int_tp> map_int_vw_tp;
typedef stapl::map_view<map2_int_tp> map2_int_vw_tp;

typedef stapl::array_view<ary_vec_ary_int_tp> ary_vec_ary_int_vw_tp;
typedef stapl::vector_view<vec_ary_vec_int_tp> vec_ary_vec_int_vw_tp;

typedef stapl::vector_view<vec_ary_map_int_tp> vec_ary_map_int_vw_tp;
typedef stapl::array_view<vec_ary_map_int_tp> ary_vec_map_int_vw_tp;
typedef stapl::vector_view<vec_map_ary_int_tp> vec_map_ary_int_vw_tp;
typedef stapl::map_view<map_ary_vec_int_tp> map_ary_vec_int_vw_tp;
typedef stapl::vector_view<vec_map_map_int_tp> vec_map_map_int_vw_tp;
typedef stapl::map_view<map_map_map_int_tp> map_map_map_int_vw_tp;
typedef stapl::map_view<map_vec_map_int_tp> map_vec_map_int_vw_tp;
typedef stapl::map_view<map_vec_ary_int_tp> map_vec_ary_int_vw_tp;

/*=========================================================================*/

bool open_zin(int model, int test, stapl::stream<ifstream>& zin, int &count) {
  string path;
  switch ( model ) {
  case 1:
    switch( test ) {
    case 1123:
      path= "data/tiny_factors.zin";
      break;
    default:
      path="data/tiny_primes.zin";
      break;
    }
    break;
  case 100:
    switch( test ) {
    case 1123:
      path = "data/small_factors.zin";
      break;
    default:
      path = "data/small_primes.zin";
      break;
    }
    break;
  case 10000:
    switch( test ) {
    case 1123:
      path = "data/medium_factors.zin";
      break;
    default:
      path = "data/medium_primes.zin";
      break;
    }
    break;
  case 1000000:
    switch( test ) {
    case 1123:
      path = "data/big_factors.zin";
      break;
    default:
      path = "data/big_primes.zin";
      break;
    }
    break;
  case 100000000:
    switch( test ) {
    case 1123:
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

size_t nestpar_1101( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1102( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1103( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1104( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1105( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1106( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1107( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1108( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1109( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1110( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1111( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1112( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1113( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1114( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1115( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1116( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1117( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1118( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1119( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1120( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1121( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_1122( size_t,
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

// FAILING: 1113, 1114
  int first_test = 1101;
  int last_test = 1116;
  if (opt_test != -1) {
    first_test = opt_test;
    last_test = opt_test;
  }

  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;

#define DEBUG 1
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
#ifdef HOMOGENEOUS
#ifdef GEN_INPUT
    case 1101:
      zout.open("np1101.zout");
      result = nestpar_1101(model, zin, zout);
      break;
    case 1102:
      zout.open("np1102.zout");
      result = nestpar_1102(model, zin, zout);
      break;
    case 1103:
      zout.open("np1103.zout");
      result = nestpar_1103(model, zin, zout);
      break;
    case 1104:
      zout.open("np1104.zout");
      result = nestpar_1104(model, zin, zout);
      break;
    case 1105:
      zout.open("np1105.zout");
      result = nestpar_1105(model, zin, zout);
      break;
    case 1106:
      zout.open("np1106.zout");
      result = nestpar_1106(model, zin, zout);
      break;
#endif // GEN_INPUT
#ifdef FILE_INPUT
    case 1107:
      zout.open("np1107.zout");
      if ( open_zin(model,1107,zin,count) ) {
        result = nestpar_1107(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 1108:
      zout.open("np1108.zout");
      if ( open_zin(model,1108,zin,count) ) {
        result = nestpar_1108(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 1109:
      zout.open("np1109.zout");
      if ( open_zin(model,1109,zin,count) ) {
        result = nestpar_1109(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 1110:
      zout.open("np1110.zout");
      if ( open_zin(model,1110,zin,count) ) {
        result = nestpar_1110(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 1111:
      zout.open("np1111.zout");
      if ( open_zin(model,1111,zin,count) ) {
        result = nestpar_1111(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 1112:
      zout.open("np1112.zout");
      if ( open_zin(model,1112,zin,count) ) {
        result = nestpar_1112(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 1113:
      zout.open("np1113.zout");
      if ( open_zin(model,1113,zin,count) ) {
        result = nestpar_1113(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 1114:
      zout.open("np1114.zout");
      if ( open_zin(model,1114,zin,count) ) {
        result = nestpar_1114(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif // FILE_INPUT
#endif // HOMOGENEOUS
#ifdef HETEROGENEOUS
#ifdef GEN_INPUT
    case 1115:
      zout.open("np1115.zout");
      if ( open_zin(model,1115,zin,count) ) {
        result = nestpar_1115(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 1116:
      zout.open("np1116.zout");
      if ( open_zin(model,1116,zin,count) ) {
        result = nestpar_1116(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif // GEN_INPUT
#ifdef MAP
    case 1117:
      zout.open("np1117.zout");
      if ( open_zin(model,1117,zin,count) ) {
        result = nestpar_1117(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 1118:
      zout.open("np1118.zout");
      if ( open_zin(model,1118,zin,count) ) {
        result = nestpar_1118(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 1119:
      zout.open("np1119.zout");
      if ( open_zin(model,1119,zin,count) ) {
        result = nestpar_1119(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 1120:
      zout.open("np1120.zout");
      if ( open_zin(model,1120,zin,count) ) {
        result = nestpar_1120(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 1121:
      zout.open("np1121.zout");
      if ( open_zin(model,1121,zin,count) ) {
        result = nestpar_1121(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 1122:
      zout.open("np1122.zout");
      if ( open_zin(model,1122,zin,count) ) {
        result = nestpar_1122(model, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif
#endif
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

#ifdef HOMOGENEOUS

/*=========================================================================
 * homogeneous nested structures
 *=========================================================================*/

#ifdef GEN_INPUT

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(vec(int))
// construct vector with generator, traverse with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_1101_inner_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::iota( vw1, 0 );
  }
};

struct nestpar_1101_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_1101_inner_fill_wf(), vw1);
  }
};

struct nestpar_1101_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1101_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1101_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1101_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1101_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_1101( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  size_t blk_size = inner / (2 * num_locs );
  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::cyclic(outer);
  comp_spec[1] = stapl::block(inner, blk_size);
  comp_spec[2] = stapl::block_cyclic(inner, blk_size);
  vec3_int_tp a(comp_spec), b(comp_spec);
  vec3_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_1101_outer_fill_wf(), a_vw );

  stapl::serial_io(nestpar_1101_outer_show_wf(zout), a_vw );

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1101","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(ary(int))
// construct array with generator, traverse with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_1102_inner_fill_wf {
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

struct nestpar_1102_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_1102_inner_fill_wf(), vw1);
  }
};

struct nestpar_1102_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1102_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1102_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1102_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1102_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_1102( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::cyclic(outer);
  size_t blk_size = inner / (2 * num_locs );
  comp_spec[1] = stapl::block(inner, blk_size);
  comp_spec[2] = stapl::balance(inner);
  ary3_int_tp a(comp_spec), b(comp_spec);
  ary3_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_1102_outer_fill_wf(), a_vw);

  stapl::serial_io(nestpar_1102_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1102","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(vec(int))
// construct vector with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_1103_inner_fill_wf {
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

struct nestpar_1103_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_1103_inner_fill_wf(), vw1);
  }
};

struct nestpar_1103_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1103_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1103_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1103_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1103_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_1103( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::cyclic(outer);
  size_t blk_size = inner / (2 * num_locs );
  comp_spec[1] = stapl::block_cyclic(inner, blk_size);
  comp_spec[2] = stapl::balance(inner);
  vec3_int_tp a(comp_spec), b(comp_spec);
  vec3_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_1103_outer_fill_wf(), a_vw );

  stapl::serial_io(nestpar_1103_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1103","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(ary(int))
// construct array with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_1104_inner_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
     stapl::generate( vw1, stapl::random_sequence());
  }
};

struct nestpar_1104_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_1104_inner_fill_wf(), vw1);
  }
};

struct nestpar_1104_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1104_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1104_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1104_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1104_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_1104( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  size_t blk_size = outer / (2 * num_locs );
  comp_spec[0] = stapl::block(outer, blk_size);
  blk_size = inner / (2 * num_locs );
  comp_spec[1] = stapl::block_cyclic(inner, blk_size);
  comp_spec[2] = stapl::balance(inner);
  ary3_int_tp a(comp_spec), b(comp_spec);
  ary3_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_1104_outer_fill_wf(), a_vw );

  stapl::serial_io(nestpar_1104_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1104","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(vec(int))
// construct vector with generator, process with stapl algorithm
// apply unary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_1105_inner_fill_wf {
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

struct nestpar_1105_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_1105_inner_fill_wf(), vw1);
  }
};

struct nestpar_1105_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::transform(vw1, vw2, neg_int_wf());
  }
};

struct nestpar_1105_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_1105_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_1105_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1105_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1105_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1105_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1105_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_1105( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  comp_spec[1] = stapl::cyclic(outer);
  size_t blk_size = inner / (2 * num_locs );
  comp_spec[2] = stapl::block(inner, blk_size);
  comp_spec[0] = stapl::block_cyclic(inner, blk_size);
  vec3_int_tp a(comp_spec), b(comp_spec);
  vec3_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_1105_outer_fill_wf(), a_vw );
  stapl::map_func(nestpar_1105_outer_fill_wf(), b_vw );

  stapl::serial_io(nestpar_1105_outer_show_wf(zout), a_vw);

  stapl::map_func(nestpar_1105_outer_process_wf(), a_vw, b_vw );

  stapl::serial_io(nestpar_1105_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1105","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(ary(int))
// construct array with generator, process with stapl algorithm
// apply binary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_1106_inner_fill_wf {
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

struct nestpar_1106_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_1106_inner_fill_wf(), vw1);
  }
};

struct nestpar_1106_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const &vw1, View2 const &vw2, View3 const &vw3)
  {
    stapl::transform(vw1, vw2, vw3, add_int_wf());
  }
};

struct nestpar_1106_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 vw1, View2 vw2, View3 vw3)
  {
    stapl::map_func(nestpar_1106_inner_process_wf(), vw1, vw2, vw3);
  }
};

struct nestpar_1106_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1106_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1106_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1106_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1106_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_1106( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::balance(outer);
  comp_spec[1] = stapl::cyclic(outer);
  size_t blk_size = inner / (2 * num_locs );
  comp_spec[2] = stapl::block(inner, blk_size);
  ary3_int_tp a(comp_spec), b(comp_spec), c(comp_spec);
  ary3_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::map_func(nestpar_1106_outer_fill_wf(), a_vw );
  stapl::map_func(nestpar_1106_outer_fill_wf(), b_vw );

  stapl::serial_io(nestpar_1106_outer_show_wf(zout), a_vw);

  stapl::serial_io(nestpar_1106_outer_show_wf(zout), b_vw);

  stapl::map_func(nestpar_1106_outer_process_wf(), a_vw, b_vw, c_vw );

  stapl::serial_io(nestpar_1106_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1106","STAPL",time_p,time_i+time_o);

  return cksum;
}
#endif // GEN_INPUT

#ifdef FILE_INPUT

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(vec(int))
// construct vector with serial I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_1107_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1107_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_1107_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1107_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1107_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_1107_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_1107_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_1107_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_1107_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1107_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1107_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1107_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1107_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_1107( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::balance(outer);
  comp_spec[1] = stapl::cyclic(inner);
  size_t blk_size = inner / (2 * num_locs );
  comp_spec[2] = stapl::block_cyclic(inner, blk_size);
  vec3_int_tp a(comp_spec), b(comp_spec);
  vec3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_1107_outer_fill_wf(zin), a_vw ); // FAILS HERE

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_1107_outer_process_wf(), a_vw, b_vw );

  stapl::serial_io(nestpar_1107_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1107","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(ary(int))
// construct array with serial I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_1108_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1108_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_1108_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1108_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1108_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_1108_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_1108_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_1108_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_1108_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1108_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1108_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1108_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1108_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_1108( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::balance(outer);
  size_t blk_size = inner / (2 * num_locs );
  comp_spec[1] = stapl::block(inner, blk_size);
  comp_spec[2] = stapl::block_cyclic(inner, blk_size);
  ary3_int_tp a(comp_spec), b(comp_spec);
  ary3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_1108_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_1108_outer_process_wf(), a_vw, b_vw);

  stapl::serial_io(nestpar_1108_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1108","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(vec(int))
// construct vector with serial I/O,
// process with mapreduce / work function, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_1109_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1109_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_1109_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1109_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1109_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_1109_inner_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_1109_outer_process_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_1109_inner_process_wf(), vw1);
  }
};

struct nestpar_1109_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1109_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1109_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1109_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1109_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_1109( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  size_t blk_size = outer / (2 * num_locs );
  comp_spec[0] = stapl::block(outer, blk_size);
  blk_size = inner / (2 * num_locs );
  comp_spec[1] = stapl::block_cyclic(inner, blk_size);
  comp_spec[2] = stapl::cyclic(inner);
  vec3_int_tp a(comp_spec), b(comp_spec);
  vec3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_1109_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_1109_outer_process_wf(), a_vw );

  stapl::serial_io(nestpar_1109_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1109","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(ary(int))
// construct array with serial I/O,
// process with mapreduce / work function, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_1110_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1110_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_1110_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1110_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1110_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_1110_inner_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_1110_outer_process_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_1110_inner_process_wf(), vw1);
  }
};

struct nestpar_1110_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1110_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1110_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1110_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1110_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_1110( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  size_t blk_size = outer / (2 * num_locs );
  comp_spec[0] = stapl::block(outer, blk_size);
  comp_spec[1] = stapl::balance(inner);
  comp_spec[2] = stapl::cyclic(inner);
  ary3_int_tp a(comp_spec), b(comp_spec);
  ary3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_1110_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_1110_outer_process_wf(), a_vw );

  stapl::serial_io(nestpar_1110_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1110","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(vec(int))
// construct vector with serial I/O,
// process with map_func / work function, display with serial
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

struct nestpar_1111_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1111_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_1111_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1111_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1111_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_1111_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func( user1_wf(), vw1, vw2 );
  }
};

struct nestpar_1111_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_1111_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_1111_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1111_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1111_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1111_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1111_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_1111( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  size_t blk_size = outer / (2 * num_locs );
  comp_spec[0] = stapl::block_cyclic(outer, blk_size);
  comp_spec[1] = stapl::balance(inner);
  comp_spec[2] = stapl::cyclic(inner);
  vec3_int_tp a(comp_spec), b(comp_spec);
  vec3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_1111_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_1111_outer_process_wf(), a_vw, b_vw);

  stapl::serial_io(nestpar_1111_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1111","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(ary(int))
// construct array with serial I/O,
// process with map_func / work function, display with serial
//////////////////////////////////////////////////////////////////////

struct user2_wf
{
  typedef void result_type;
  template <typename Ref1, typename Ref2, typename Ref3>
  result_type operator()(Ref1 x, Ref2 y, Ref3 z)
  {
    z = (x % 10) + (y % 10);
  }
};

struct nestpar_1112_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1112_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_1112_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1112_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1112_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_1112_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const &vw1, View2 const &vw2, View3 const &vw3)
  {
    stapl::map_func( user2_wf(), vw1, vw2, vw3 );
  }
};

struct nestpar_1112_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 vw1, View2 vw2, View3 vw3)
  {
    stapl::map_func(nestpar_1112_inner_process_wf(), vw1, vw2, vw3);
  }
};

struct nestpar_1112_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1112_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1112_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1112_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1112_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_1112( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  size_t blk_size = outer / (2 * num_locs );
  comp_spec[0] = stapl::block_cyclic(outer, blk_size);
  comp_spec[1] = stapl::balance(inner);
  blk_size = inner / (2 * num_locs );
  comp_spec[2] = stapl::block(inner, blk_size);
  ary3_int_tp a(comp_spec), b(comp_spec), c(comp_spec);
  ary3_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(nestpar_1112_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_1112_outer_process_wf(), a_vw, b_vw, c_vw );

  stapl::serial_io(nestpar_1112_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1112","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(vec(int))
// construct vector with serial I/O,
// process with partial_sum, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_1113_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1113_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_1113_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1113_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1113_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_1113_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_1113_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_1113_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_1113_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1113_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1113_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1113_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1113_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_1113( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::cyclic(outer);
  size_t blk_size = inner / (2 * num_locs );
  comp_spec[1] = stapl::block_cyclic(inner, blk_size);
  comp_spec[2] = stapl::block(inner, blk_size);
  vec3_int_tp a(comp_spec), b(comp_spec);
  vec3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_1113_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_1113_outer_process_wf(), a_vw, b_vw );

  stapl::serial_io(nestpar_1113_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1113","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(ary(int))
// construct array with serial I/O,
// process with partial_sum, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_1114_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1114_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_1114_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_1114_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1114_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_1114_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_1114_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_1114_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_1114_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1114_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1114_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1114_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1114_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_1114( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::cyclic(outer);
  comp_spec[1] = stapl::balance(inner);
  size_t blk_size = inner / (2 * num_locs );
  comp_spec[2] = stapl::block(inner, blk_size);
  ary3_int_tp a(comp_spec), b(comp_spec);
  ary3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_1114_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_1114_outer_process_wf(), a_vw, b_vw );

  stapl::serial_io(nestpar_1114_outer_show_wf(zout), b_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1114","STAPL",time_p,time_i+time_o);

  return cksum;
}

#endif // FILE_INPUT

#endif // HOMOGENEOUS

/*=========================================================================
 * heterogeneous positional nested structures
 *=========================================================================*/

#ifdef HETEROGENEOUS

#ifdef GEN_INPUT

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(ary(vec(int
//////////////////////////////////////////////////////////////////////

struct nestpar_1115_inner_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::iota( vw1, 0 );
  }
};

struct nestpar_1115_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_1115_inner_fill_wf(), vw1);
  }
};

struct nestpar_1115_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    return vw1.size();
  }
};

struct nestpar_1115_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_1115_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_1115_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1115_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1115_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1115_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1115_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_1115( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::cyclic(outer);
  comp_spec[1] = stapl::balance(inner);
  size_t blk_size = inner / (2 * num_locs );
  comp_spec[2] = stapl::block_cyclic(inner, blk_size);
  vec_ary_vec_int_tp a(comp_spec), b(comp_spec);
  vec_ary_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_1115_outer_fill_wf(), a_vw );

  stapl::map_func(nestpar_1115_outer_process_wf(), a_vw, b_vw );

  stapl::serial_io(nestpar_1115_outer_show_wf(zout), b_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1115","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(vec(ary(int
//////////////////////////////////////////////////////////////////////

struct nestpar_1116_inner_fill_wf {
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

struct nestpar_1116_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_1116_inner_fill_wf(), vw1);
  }
};

struct nestpar_1116_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    return vw1.size();
  }
};

struct nestpar_1116_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_1116_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_1116_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1116_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_1116_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1116_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1116_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_1116( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_dist_spec_tp comp_spec(3);
  size_t blk_size = outer / (2 * num_locs );
  comp_spec[0] = stapl::block(outer, blk_size);
  comp_spec[1] = stapl::balance(inner);
  blk_size = inner / (2 * num_locs );
  comp_spec[2] = stapl::block_cyclic(inner, blk_size);
  ary_vec_ary_int_tp a(comp_spec), b(comp_spec);
  ary_vec_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_1116_outer_fill_wf(), a_vw );

  stapl::map_func(nestpar_1116_outer_process_wf(), a_vw, b_vw );

  stapl::serial_io(nestpar_1116_outer_show_wf(zout), b_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP1116","STAPL",time_p,time_i+time_o);

  return cksum;
}

#ifdef MAP

/*=========================================================================
 * heterogeneous positional & non-positional nested structures
 *=========================================================================*/

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(ary(map(int
//////////////////////////////////////////////////////////////////////

struct nestpar_1117_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_1117_fill_wf(size_t o, size_t m, size_t i)
    : inner(i), middle(m), outer(o)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a)
  {
    if ( inner <= data_cnt ) {
      for (size_t i = 0; i < outer; i++ ) {
        int outer_ndx = i;
        for (size_t j = 0; j < middle; j++ ) {
          int middle_ndx = j;
          for (size_t k = 0; k < inner; k++ ) {
            int inner_key = prime_nums[k];
            int value = rand_nums[k] % (k+1);
            a[outer_ndx][middle_ndx][inner_key] = value;
          }
        }
      }
    } else {
      for (size_t i = 0; i < outer; i++ ) {
        int outer_ndx = i;
        for (size_t j = 0; j < middle; j++ ) {
          int middle_ndx = j;
          for (size_t k = 0; k < inner; k++ ) {
            for (size_t m = 0; m < data_cnt; m++ ) {
              int ndx = data_cnt - m;
              int inner_key = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[outer_ndx][middle_ndx][inner_key] = value;
            }
          }
        }
      }
    }
  }
  void define_type(stapl::typer& t) {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};

// - - - - - - - - - -

struct nestpar_1117_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1117_inner_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(put_map_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct nestpar_1117_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1117_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1117_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_1117( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

#ifdef LEN_CONSTR
  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);
  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));
  ary2_sz_tp len_2(len_1);
  ary2_sz_vw_tp len_2_vw(len_2);
  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));
  vec_ary_map_int_tp a(len_1_vw), b(len_1_vw);
  vec_ary_map_int_vw_tp a_vw(a), b_vw(b);
  ndx_dom_tp map_dom(0,16277216);
#else
  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::cyclic(outer);
  size_t blk_size = inner / (2 * num_locs );
  comp_spec[1] = stapl::block(inner, blk_size);
  comp_spec[2] = stapl::block_cyclic(inner, blk_size);
#endif

  stapl::do_once(nestpar_1117_fill_wf(outer,middle,inner), a_vw );

  stapl::serial_io(nestpar_1117_outer_show_wf(zout), b_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP1117","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}
#endif

#ifdef MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(map(ary(int
//////////////////////////////////////////////////////////////////////

struct nestpar_1118_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_1118_fill_wf(size_t o, size_t m, size_t i)
    : inner(i), middle(m), outer(o)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a) {
    if ( inner <= data_cnt ) {
      for (size_t i = 0; i < outer; i++ ) {
        int outer_ndx = i;
        for (size_t j = 0; j < middle; j++ ) {
          int middle_key = prime_nums[j];
          for (size_t k = 0; k < inner; k++ ) {
            int inner_ndx = k;
            int value = rand_nums[k] % (k+1);
            a[outer_ndx][middle_key][inner_ndx] = value;
          }
        }
      }
    } else {
      for (size_t i = 0; i < outer; i++ ) {
        int outer_ndx = i;
        for (size_t j = 0; j < middle; j++ ) {
          int middle_key = prime_nums[j];
          for (size_t k = 0; k < inner; k++ ) {
            for (size_t m = 0; m < data_cnt; m++ ) {
              int ndx = data_cnt - m;
              int inner_ndx = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[outer_ndx][middle_key][inner_ndx] = value;
            }
          }
        }
      }
    }
  }
  void define_type(stapl::typer& t) {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};

// - - - - - - - - - -

struct nestpar_1118_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1118_inner_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1.second);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct nestpar_1118_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1118_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1118_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_1118( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

#ifdef LEN_CONSTR
  vec_map_ary_int_tp a(outer), b(outer);
  vec_map_ary_int_vw_tp a_vw(a), b_vw(b);
#else
  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::cyclic(outer);
  size_t blk_size = middle / (2 * num_locs );
  comp_spec[1] = stapl::block(middle, blk_size);
  comp_spec[2] = stapl::balance(inner);
#endif

  stapl::do_once(nestpar_1118_fill_wf(outer,middle,inner), a_vw );

  stapl::serial_io(nestpar_1118_outer_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP1118","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_seq_l1_map_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

#endif

#ifdef MAP

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(ary(vec(int
//////////////////////////////////////////////////////////////////////

struct nestpar_1119_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_1119_fill_wf(size_t o, size_t m, size_t i)
    : inner(i), middle(m), outer(o)
  { }

  typedef void result_type;
  template <typename Data, typename Length>
  result_type operator()(Data &a, Length &len_vw) {
    if ( inner <= data_cnt ) {

stapl::do_once([&](){
cerr << "319 T1: " << len_vw.size() << endl;
});
      for (size_t i = 0; i < len_vw.size(); i++ ) {
        int outer_key = prime_nums[i];
        a[outer_key].resize(len_vw[i].size());
      }
stapl::do_once([&](){
cerr << "319 T2: " << len_vw.size() << endl;
});
      for (size_t i = 0; i < len_vw.size(); i++ ) {
        int outer_key = prime_nums[i];
stapl::do_once([&](){
cerr << "319 T3: " << len_vw[i].size() << endl;
});
        for (size_t j = 0; j < len_vw[i].size(); j++ ) {
stapl::do_once([&](){
cerr << "319 T4: " << len_vw[i][j] << endl;
});
          a[outer_key][j].resize(len_vw[i][j]);
        }
      }

      for (size_t i = 0; i < outer; i++ ) {
        int outer_key = prime_nums[i];
        for (size_t j = 0; j < middle; j++ ) {
          int middle_ndx = j;
          for (size_t k = 0; k < inner; k++ ) {
            int inner_ndx = k;
            int value = rand_nums[k] % (k+1);
            a[outer_key][middle_ndx][inner_ndx] = value;
          }
        }
      }
    } else {
#if 0
      for (size_t i = 0; i < len_vw.size(); i++ ) {
        int outer_key = prime_nums[i];
        a[outer_key].resize(len_vw[i].size());
      }
      for (size_t i = 0; i < len_vw.size(); i++ ) {
        int outer_key = prime_nums[i];
        for (size_t j = 0; j < len_vw[i].size(); j++ ) {
          a[outer_key][j].resize(len_vw[i][j]);
        }
      }
#endif

      for (size_t i = 0; i < outer; i++ ) {
        int outer_key = prime_nums[i];
        for (size_t j = 0; j < middle; j++ ) {
          int middle_ndx = j;
          for (size_t k = 0; k < inner; k++ ) {
            for (size_t m = 0; m < data_cnt; m++ ) {
              int ndx = data_cnt - m;
              int inner_ndx = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[outer_key][middle_ndx][inner_ndx] = value;
            }
          }
        }
      }
    }
  }
  void define_type(stapl::typer& t) {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};

// - - - - - - - - - -

struct nestpar_1119_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1119_inner_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct nestpar_1119_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1119_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1119_inner_show_wf(m_zout), vw1.second);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_1119( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

#ifdef LEN_CONSTR
  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);
  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));
  ary2_sz_tp len_2(len_1);
  ary2_sz_vw_tp len_2_vw(len_2);
  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));
  map_vec_ary_int_tp a, b;
  map_vec_ary_int_vw_tp a_vw(a), b_vw(b);
#else
  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::cyclic(outer);
  size_t blk_size = middle / (2 * num_locs );
  comp_spec[1] = stapl::block_cyclic(middle, blk_size);
  comp_spec[2] = stapl::balance(inner);
#endif

  stapl::do_once(nestpar_1119_fill_wf(outer,middle,inner), a_vw, len_2_vw );

  stapl::serial_io(nestpar_1119_outer_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP1119","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

#endif

#ifdef MAP

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(map(map(int
//////////////////////////////////////////////////////////////////////

struct nestpar_1120_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_1120_fill_wf(size_t o, size_t m, size_t i)
    : inner(i), middle(m), outer(o)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a) {
    if ( inner <= data_cnt && middle <= data_cnt ) {
      for (size_t i = 0; i < outer; i++ ) {
        int outer_ndx = i;
        for (size_t j = 0; j < middle; j++ ) {
          int middle_key = prime_nums[j];
          for (size_t k = 0; k < inner; k++ ) {
            int inner_key = prime_nums[k];
            int value = rand_nums[k] % (k+1);
            a[outer_ndx][middle_key][inner_key] = value;
          }
        }
      }
    } else {
      for (size_t i = 0; i < outer; i++ ) {
        int outer_ndx = i;
        for (size_t j = 0; j < middle; j++ ) {
          int middle_key = prime_nums[data_cnt % j];
          for (size_t k = 0; k < inner; k++ ) {
            for (size_t m = 0; m < data_cnt; m++ ) {
              int ndx = data_cnt - m;
              int inner_key = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[outer_ndx][middle_key][inner_key] = value;
            }
          }
        }
      }
    }
  }
  void define_type(stapl::typer& t) {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};

// - - - - - - - - - -

struct nestpar_1120_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1120_inner_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(put_map_val_wf(m_zout), vw1.second);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct nestpar_1120_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1120_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1120_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_1120( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

#ifdef LEN_CONSTR
  vec_map2_int_tp a(outer), b(outer);
  vec_map2_int_vw_tp a_vw(a), b_vw(b);
  stapl::do_once(nestpar_1120_fill_wf(outer,middle,inner), a_vw );
#else
  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::balance(outer);
  size_t blk_size = middle / (2 * num_locs );
  comp_spec[1] = stapl::block(middle, blk_size);
  blk_size = inner / (2 * num_locs );
  comp_spec[2] = stapl::block_cyclic(inner, blk_size);
#endif

  stapl::serial_io(nestpar_1120_outer_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP1120","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_seq_l1_map_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

#endif

#ifdef MAP

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(vec(map(int
//////////////////////////////////////////////////////////////////////

struct nestpar_1121_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_1121_fill_wf(size_t o, size_t m, size_t i)
    : inner(i), middle(m), outer(o)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a)
  {
    if ( outer <= data_cnt && inner <= data_cnt ) {
      for (size_t i = 0; i < outer; i++ ) {
        int outer_key = prime_nums[i];
        for (size_t j = 0; j < middle; j++ ) {
          int middle_ndx = j;
          for (size_t k = 0; k < inner; k++ ) {
            int inner_key = prime_nums[k];
            int value = rand_nums[k] % (k+1);
            a[outer_key][middle_ndx][inner_key] = value;
          }
        }
      }
    } else {
      for (size_t i = 0; i < outer; i++ ) {
        int outer_key = prime_nums[data_cnt % i];
        for (size_t j = 0; j < middle; j++ ) {
          int middle_ndx = j;
          for (size_t k = 0; k < inner; k++ ) {
            for (size_t m = 0; m < data_cnt; m++ ) {
              int ndx = data_cnt - m;
              int inner_key = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[outer_key][middle_ndx][inner_key] = value;
            }
          }
        }
      }
    }
  }
  void define_type(stapl::typer& t) {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};

// - - - - - - - - - -

struct nestpar_1121_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1121_inner_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(put_map_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct nestpar_1121_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1121_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1121_inner_show_wf(m_zout), vw1.second);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_1121( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

#ifdef LEN_CONSTR
  map_vec_map_int_tp a, b;
  map_vec_map_int_vw_tp a_vw(a), b_vw(b);
#else
  vec_dist_spec_tp comp_spec(3);
  size_t blk_size = outer / (2 * num_locs );
  comp_spec[0] = stapl::block_cyclic(outer, blk_size);
  comp_spec[1] = stapl::cyclic(middle);
  blk_size = inner / (2 * num_locs );
  comp_spec[2] = stapl::block(inner, blk_size);
#endif

  stapl::do_once(nestpar_1121_fill_wf(outer,middle,inner), a_vw );

  stapl::serial_io(nestpar_1121_outer_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP1121","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_seq_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

#endif // MAP

#endif // GEN_INPUT

#endif // HETEROGENEOUS

#ifdef HOMOGENEOUS

#ifdef MAP

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(map(int
//////////////////////////////////////////////////////////////////////

struct nestpar_1122_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_1122_fill_wf(size_t o, size_t m, size_t i)
    : inner(i), middle(m), outer(o)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a)
  {
    if ( outer <= data_cnt && middle <= data_cnt && inner <= data_cnt ) {
      for (size_t i = 0; i < outer; i++ ) {
        int outer_key = prime_nums[i];
        for (size_t j = 0; j < middle; j++ ) {
          int middle_key = prime_nums[j];
          for (size_t k = 0; k < inner; k++ ) {
            int inner_key = prime_nums[k];
            int value = rand_nums[k] % (k+1);
            a[outer_key][middle_key][inner_key] = value;
          }
        }
      }
    } else {
      for (size_t i = 0; i < outer; i++ ) {
        int outer_key = prime_nums[data_cnt % i];
        for (size_t j = 0; j < middle; j++ ) {
          int middle_key = prime_nums[data_cnt % j];
          for (size_t k = 0; k < inner; k++ ) {
            for (size_t m = 0; m < data_cnt; m++ ) {
              int ndx = data_cnt - m;
              int inner_key = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[outer_key][middle_key][inner_key] = value;
            }
          }
        }
      }
    }
  }
  void define_type(stapl::typer& t) {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};

// - - - - - - - - - -

struct nestpar_1122_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1122_inner_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(put_map_val_wf(m_zout), vw1.second);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct nestpar_1122_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_1122_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_1122_inner_show_wf(m_zout), vw1.second);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_1122( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;
  size_t num_locs = stapl::get_num_locations();

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

#ifdef LEN_CONSTR
  map_map2_int_tp a, b;
  map_map2_int_vw_tp a_vw(a), b_vw(b);
#else
  size_t blk_size = outer / (2 * num_locs );
  vec_dist_spec_tp comp_spec(3);
  comp_spec[0] = stapl::block_cyclic(outer, blk_size);
  comp_spec[1] = stapl::cyclic(middle);
  blk_size = inner / (2 * num_locs );
  comp_spec[2] = stapl::block(inner, blk_size);
#endif

  stapl::do_once(nestpar_1122_fill_wf(outer,middle,inner), a_vw );

  stapl::serial_io(nestpar_1122_outer_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP1122","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_map_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

#endif // FILL_MAP

#endif // HOMOGENEOUS

