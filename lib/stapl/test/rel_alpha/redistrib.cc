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

#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>

#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>

#include <stapl/views/map_view.hpp>
#include <stapl/containers/map/map.hpp>

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>

#include <stapl/views/multiarray_view.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>

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

// when redistribution of map is checked in
//#define MAP 1

#define MAP_OUT 1

// when redistribution of arbitrary is checked in
// #define EXPLICIT

// when redistribute() is checked in
#define RE_CKIN 1

/*=========================================================================*/

typedef stapl::negate<int>      neg_int_wf;
typedef stapl::identity<int>     id_int_wf;
typedef stapl::plus<int>        add_int_wf;
typedef stapl::minus<int>       sub_int_wf;
typedef stapl::multiplies<int>  mul_int_wf;
typedef stapl::min<int>         min_int_wf;
typedef stapl::max<int>         max_int_wf;
typedef stapl::modulus<int>     mod_int_wf;
typedef stapl::divides<int>     div_int_wf;

typedef stapl::identity<size_t>  id_un_wf;
typedef stapl::bit_or<size_t>    or_un_wf;
typedef stapl::bit_and<size_t>  and_un_wf;
typedef stapl::bit_xor<size_t>  xor_un_wf;

//////////////////////////////////////////////////////////////////////

typedef stapl::indexed_domain<size_t>           ndx_dom_tp;

typedef stapl::distribution_spec<>              distribution_spec;

typedef stapl::vector<int>             vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

typedef stapl::vector<int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> vec_int_blk_tp;
typedef stapl::vector_view<vec_int_blk_tp> vec_int_blk_vw_tp;

typedef stapl::vector<int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> vec_int_cyc_tp;
typedef stapl::vector_view<vec_int_cyc_tp> vec_int_cyc_vw_tp;

typedef stapl::vector<int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> vec_int_blk_cyc_tp;
typedef stapl::vector_view<vec_int_blk_cyc_tp> vec_int_blk_cyc_vw_tp;

typedef stapl::vector<int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> vec_int_bal_tp;
typedef stapl::vector_view<vec_int_bal_tp> vec_int_bal_vw_tp;

#ifdef EXPLICIT
typedef stapl::vector<int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> vec_int_expl_tp;
typedef stapl::vector_view<vec_int_expl_tp> vec_int_expl_vw_tp;
#endif


typedef stapl::array<int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> ary_int_blk_tp;
typedef stapl::array_view<ary_int_blk_tp> ary_int_blk_vw_tp;

typedef stapl::array<int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> ary_int_cyc_tp;
typedef stapl::array_view<ary_int_cyc_tp> ary_int_cyc_vw_tp;

typedef stapl::array<int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> ary_int_blk_cyc_tp;
typedef stapl::array_view<ary_int_blk_cyc_tp> ary_int_blk_cyc_vw_tp;

typedef stapl::array<int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> ary_int_bal_tp;
typedef stapl::array_view<ary_int_bal_tp> ary_int_bal_vw_tp;

#ifdef EXPLICIT
typedef stapl::array<int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> ary_int_expl_tp;
typedef stapl::array_view<ary_int_expl_tp> ary_int_expl_vw_tp;
#endif


#ifdef MAP
typedef stapl::map<int,int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> map_int_blk_tp;
typedef stapl::map_view<map_int_blk_tp> map_int_blk_vw_tp;

typedef stapl::map<int,int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> map_int_cyc_tp;
typedef stapl::map_view<map_int_cyc_tp> map_int_cyc_vw_tp;

typedef stapl::map<int,int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> map_int_blk_cyc_tp;
typedef stapl::map_view<map_int_blk_cyc_tp> map_int_blk_cyc_vw_tp;

typedef stapl::map<int,int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> map_int_bal_tp;
typedef stapl::map_view<map_int_bal_tp> map_int_bal_vw_tp;

#ifdef EXPLICIT
typedef stapl::map<int,int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> map_int_expl_tp;
typedef stapl::map_view<map_int_expl_tp> map_int_expl_vw_tp;
#endif
#endif


typedef stapl::dynamic_graph<
        stapl::UNDIRECTED, stapl::NONMULTIEDGES, int, int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> dygraf_int_blk_tp;
typedef stapl::graph_view<dygraf_int_blk_tp> dygraf_int_blk_vw_tp;

typedef stapl::dynamic_graph<
        stapl::UNDIRECTED, stapl::NONMULTIEDGES, int, int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> dygraf_int_cyc_tp;
typedef stapl::graph_view<dygraf_int_cyc_tp> dygraf_int_cyc_vw_tp;

typedef stapl::dynamic_graph<
        stapl::UNDIRECTED, stapl::NONMULTIEDGES, int, int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> dygraf_int_blk_cyc_tp;
typedef stapl::graph_view<dygraf_int_blk_cyc_tp> dygraf_int_blk_cyc_vw_tp;

typedef stapl::dynamic_graph<
        stapl::UNDIRECTED, stapl::NONMULTIEDGES, int, int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> dygraf_int_bal_tp;
typedef stapl::graph_view<dygraf_int_bal_tp> dygraf_int_bal_vw_tp;

#ifdef EXPLICIT
typedef stapl::dynamic_graph<
        stapl::UNDIRECTED, stapl::NONMULTIEDGES, int, int,
        stapl::view_based_partition<distribution_spec>,
        stapl::view_based_mapper<distribution_spec>
> dygraf_int_expl_tp;
typedef stapl::graph_view<dygraf_int_expl_tp> dygraf_int_expl_vw_tp;
#endif

/*=========================================================================*/

void open_zin(int model, int test, stapl::stream<ifstream>& zin) {
  switch ( model ) {
  case 1:
    switch( test ) {
    default:
      zin.open("tiny_primes.zin");
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

bool redistrib_101( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_102( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_103( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_104( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_105( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_106( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_107( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_108( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_109( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_110( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_111( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_112( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_113( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_114( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_115( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_116( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_117( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_118( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_119( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_120( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_121( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_122( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_123( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_124( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_125( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );

bool redistrib_201( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_202( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_203( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_204( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_205( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_206( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_207( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_208( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_209( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_210( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_211( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_212( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_213( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_214( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_215( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_216( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_217( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_218( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_219( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_220( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_221( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_222( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_223( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_224( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_225( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );

bool redistrib_301( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_302( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_303( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_304( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_305( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_306( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_307( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_308( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_309( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_310( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_311( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_312( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_313( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_314( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_315( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_316( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_317( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_318( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_319( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_320( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_321( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_322( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_323( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_324( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
bool redistrib_325( int model,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );

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

  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;

  bool ok = true;
  first_test = 101; last_test = 125;
  for (int test = first_test; test <= last_test; test++ ) {
    set_random_seed();
    switch (test) {
    case 101:
      zout.open("re101.zout");
      ok = redistrib_101(model, zin, zout);
      zout.close();
      break;
    case 102:
      zout.open("re102.zout");
      ok = redistrib_102(model, zin, zout);
      zout.close();
      break;
    case 103:
      zout.open("re103.zout");
      ok = redistrib_103(model, zin, zout);
      zout.close();
      break;
    case 104:
      zout.open("re104.zout");
      ok = redistrib_104(model, zin, zout);
      zout.close();
      break;
    case 105:
      zout.open("re105.zout");
      ok = redistrib_105(model, zin, zout);
      zout.close();
      break;
    case 106:
      zout.open("re106.zout");
      ok = redistrib_106(model, zin, zout);
      zout.close();
      break;
    case 107:
      zout.open("re107.zout");
      ok = redistrib_107(model, zin, zout);
      zout.close();
      break;
    case 108:
      zout.open("re108.zout");
      ok = redistrib_108(model, zin, zout);
      zout.close();
      break;
    case 109:
      zout.open("re109.zout");
      ok = redistrib_109(model, zin, zout);
      zout.close();
      break;
    case 110:
      zout.open("re110.zout");
      ok = redistrib_110(model, zin, zout);
      zout.close();
      break;
    case 111:
      zout.open("re111.zout");
      open_zin(model,111,zin);
      ok = redistrib_111(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 112:
      zout.open("re112.zout");
      open_zin(model,112,zin);
      ok = redistrib_112(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 113:
      zout.open("re113.zout");
      open_zin(model,113,zin);
      ok = redistrib_113(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 114:
      zout.open("re114.zout");
      open_zin(model,114,zin);
      ok = redistrib_114(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 115:
      zout.open("re115.zout");
      open_zin(model,115,zin);
      ok = redistrib_115(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 116:
      zout.open("re116.zout");
      open_zin(model,116,zin);
      ok = redistrib_116(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 117:
      zout.open("re117.zout");
      open_zin(model,117,zin);
      ok = redistrib_117(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 118:
      zout.open("re118.zout");
      open_zin(model,118,zin);
      ok = redistrib_118(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 119:
      zout.open("re119.zout");
      open_zin(model,119,zin);
      ok = redistrib_119(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 120:
      zout.open("re120.zout");
      open_zin(model,120,zin);
      ok = redistrib_120(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 121:
      zout.open("re121.zout");
      open_zin(model,121,zin);
      ok = redistrib_121(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 122:
      zout.open("re122.zout");
      open_zin(model,122,zin);
      ok = redistrib_122(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 123:
      zout.open("re123.zout");
      open_zin(model,123,zin);
      ok = redistrib_123(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 124:
      zout.open("re124.zout");
      open_zin(model,124,zin);
      ok = redistrib_124(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 125:
      zout.open("re125.zout");
      open_zin(model,125,zin);
      ok = redistrib_125(model, zin, zout);
      zout.close();
      zin.close();
      break;

#ifdef MAP
    case 201:
      zout.open("re201.zout");
      ok = redistrib_201(model, zin, zout);
      zout.close();
      break;
    case 202:
      zout.open("re202.zout");
      ok = redistrib_202(model, zin, zout);
      zout.close();
      break;
    case 203:
      zout.open("re203.zout");
      ok = redistrib_203(model, zin, zout);
      zout.close();
      break;
    case 204:
      zout.open("re204.zout");
      ok = redistrib_204(model, zin, zout);
      zout.close();
      break;
    case 205:
      zout.open("re205.zout");
      ok = redistrib_205(model, zin, zout);
      zout.close();
      break;
    case 206:
      zout.open("re206.zout");
      ok = redistrib_206(model, zin, zout);
      zout.close();
      break;
    case 207:
      zout.open("re207.zout");
      ok = redistrib_207(model, zin, zout);
      zout.close();
      break;
    case 208:
      zout.open("re208.zout");
      ok = redistrib_208(model, zin, zout);
      zout.close();
      break;
    case 209:
      zout.open("re209.zout");
      ok = redistrib_209(model, zin, zout);
      zout.close();
      break;
    case 210:
      zout.open("re210.zout");
      ok = redistrib_210(model, zin, zout);
      zout.close();
      break;
    case 211:
      zout.open("re211.zout");
      open_zin(model,211,zin);
      ok = redistrib_211(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 212:
      zout.open("re212.zout");
      open_zin(model,212,zin);
      ok = redistrib_212(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 213:
      zout.open("re213.zout");
      open_zin(model,213,zin);
      ok = redistrib_213(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 214:
      zout.open("re214.zout");
      open_zin(model,214,zin);
      ok = redistrib_214(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 215:
      zout.open("re215.zout");
      open_zin(model,215,zin);
      ok = redistrib_215(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 216:
      zout.open("re216.zout");
      open_zin(model,216,zin);
      ok = redistrib_216(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 217:
      zout.open("re217.zout");
      open_zin(model,217,zin);
      ok = redistrib_217(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 218:
      zout.open("re218.zout");
      open_zin(model,218,zin);
      ok = redistrib_218(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 219:
      zout.open("re219.zout");
      open_zin(model,219,zin);
      ok = redistrib_219(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 220:
      zout.open("re220.zout");
      open_zin(model,220,zin);
      ok = redistrib_220(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 221:
      zout.open("re221.zout");
      open_zin(model,221,zin);
      ok = redistrib_221(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 222:
      zout.open("re222.zout");
      open_zin(model,222,zin);
      ok = redistrib_222(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 223:
      zout.open("re223.zout");
      open_zin(model,223,zin);
      ok = redistrib_223(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 224:
      zout.open("re224.zout");
      open_zin(model,224,zin);
      ok = redistrib_224(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 225:
      zout.open("re225.zout");
      open_zin(model,225,zin);
      ok = redistrib_225(model, zin, zout);
      zout.close();
      zin.close();
      break;
#endif

    case 301:
      zout.open("re301.zout");
      ok = redistrib_301(model, zin, zout);
      zout.close();
      break;
    case 302:
      zout.open("re302.zout");
      ok = redistrib_302(model, zin, zout);
      zout.close();
      break;
    case 303:
      zout.open("re303.zout");
      ok = redistrib_303(model, zin, zout);
      zout.close();
      break;
    case 304:
      zout.open("re304.zout");
      ok = redistrib_304(model, zin, zout);
      zout.close();
      break;
    case 305:
      zout.open("re305.zout");
      ok = redistrib_305(model, zin, zout);
      zout.close();
      break;
    case 306:
      zout.open("re306.zout");
      ok = redistrib_306(model, zin, zout);
      zout.close();
      break;
    case 307:
      zout.open("re307.zout");
      ok = redistrib_307(model, zin, zout);
      zout.close();
      break;
    case 308:
      zout.open("re308.zout");
      ok = redistrib_308(model, zin, zout);
      zout.close();
      break;
    case 309:
      zout.open("re309.zout");
      ok = redistrib_309(model, zin, zout);
      zout.close();
      break;
    case 310:
      zout.open("re310.zout");
      ok = redistrib_310(model, zin, zout);
      zout.close();
      break;
    case 311:
      zout.open("re311.zout");
      open_zin(model,311,zin);
      ok = redistrib_311(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 312:
      zout.open("re312.zout");
      open_zin(model,312,zin);
      ok = redistrib_312(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 313:
      zout.open("re313.zout");
      open_zin(model,313,zin);
      ok = redistrib_313(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 314:
      zout.open("re314.zout");
      open_zin(model,314,zin);
      ok = redistrib_314(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 315:
      zout.open("re315.zout");
      open_zin(model,315,zin);
      ok = redistrib_315(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 316:
      zout.open("re316.zout");
      open_zin(model,316,zin);
      ok = redistrib_316(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 317:
      zout.open("re317.zout");
      open_zin(model,317,zin);
      ok = redistrib_317(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 318:
      zout.open("re318.zout");
      open_zin(model,318,zin);
      ok = redistrib_318(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 319:
      zout.open("re319.zout");
      open_zin(model,319,zin);
      ok = redistrib_319(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 320:
      zout.open("re320.zout");
      open_zin(model,320,zin);
      ok = redistrib_320(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 321:
      zout.open("re321.zout");
      open_zin(model,321,zin);
      ok = redistrib_321(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 322:
      zout.open("re322.zout");
      open_zin(model,322,zin);
      ok = redistrib_322(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 323:
      zout.open("re323.zout");
      open_zin(model,323,zin);
      ok = redistrib_323(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 324:
      zout.open("re324.zout");
      open_zin(model,324,zin);
      ok = redistrib_324(model, zin, zout);
      zout.close();
      zin.close();
      break;
    case 325:
      zout.open("re325.zout");
      open_zin(model,325,zin);
      ok = redistrib_325(model, zin, zout);
      zout.close();
      zin.close();
      break;
    }
  }
  return EXIT_SUCCESS;
}

/*==========================================================================*/

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with block(n) data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_101( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::block(size,50);
  ary_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( a_vw, 0 );
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate( b_vw, step_wf(base,step));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int max_val = stapl::map_reduce( id_int_wf(), max_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return max_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with block(n) data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_102( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::cyclic(size);
  ary_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));
  int rep = 42;
  base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(b_vw, repeat_wf(base,rep));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int max_val = stapl::map_reduce( id_int_wf(), max_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return max_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with block(n) data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_103( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  ary_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(a_vw, repeat_wf(base,rep));
  stapl::generate( b_vw, stapl::random_sequence(1000));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int max_val = stapl::map_reduce( id_int_wf(), max_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return max_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with block(n) data distribution, perform computation,
// Force redistribution to balance(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_104( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::balance(size);
  ary_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( a_vw, repeat_wf(base,rep));
  stapl::generate( b_vw, stapl::random_sequence(1000));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int max_val = stapl::map_reduce( id_int_wf(), max_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return max_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with block(n) data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_105( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::arbitrary(size,20);
  ary_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::generate( a_vw, stapl::random_sequence(1000));
  stapl::iota( b_vw, 0 );

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int max_val = stapl::map_reduce( id_int_wf(), max_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return max_val;
#endif
}


///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with cyclic data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_106( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::cyclic(size);
  distribution_spec new_dist = stapl::block(size,50);
  ary_int_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( b_vw, 0 );
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate( a_vw, step_wf(base,step));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int min_val = stapl::map_reduce( id_int_wf(), min_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return min_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_107( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::cyclic(size);
  ary_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(b_vw, step_wf(base,step));
  int rep = 42;
  base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(a_vw, repeat_wf(base,rep));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int min_val = stapl::map_reduce( id_int_wf(), min_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return min_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with cyclic data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_108( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::cyclic(size);
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  ary_int_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( b_vw, repeat_wf(base,rep));
  stapl::generate( a_vw, stapl::random_sequence(1000));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int min_val = stapl::map_reduce( id_int_wf(), min_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return min_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with cyclic data distribution, perform computation,
// Force redistribution to balance, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_109( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::cyclic(size);
  distribution_spec new_dist = stapl::balance(size);
  ary_int_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( b_vw, 0 );
  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( a_vw, repeat_wf(base,rep));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int min_val = stapl::map_reduce( id_int_wf(), min_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return min_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with cyclic data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_110( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::cyclic(size);
  distribution_spec new_dist = stapl::arbitrary(size,20);
  ary_int_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::generate( b_vw, stapl::random_sequence(1000));
  stapl::iota( a_vw, 0 );

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int min_val = stapl::map_reduce( id_int_wf(), min_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return min_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_111( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::block(size,50);
  ary_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( a_vw, 0 );
  stapl::serial_io(get_val_wf(zin), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, sub_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_112( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::cyclic(size);
  ary_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));
  stapl::serial_io(get_val_wf(zin), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, sub_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_113( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  ary_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(a_vw, repeat_wf(base,rep));
  stapl::serial_io(get_val_wf(zin), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, sub_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to balance, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_114( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::balance(size);
  ary_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::generate( a_vw, stapl::random_sequence(1000));
  stapl::serial_io(get_val_wf(zin), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, sub_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_115( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::arbitrary(size,20);
  ary_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::serial_io(get_val_wf(zin), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, sub_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with balance data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_116( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::block(size,50);
  ary_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::iota( b_vw, 0 );

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with balance data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_117( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::cyclic(size);
  ary_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(b_vw, step_wf(base,step));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with balance data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_118( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  ary_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( b_vw, repeat_wf(base,rep));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with balance data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_119( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::block(size,20);
  ary_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::generate( b_vw, stapl::random_sequence(1000));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with balance data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_120( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::arbitrary(size,20); // FIXME
  ary_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::serial_io(get_val_wf(), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with arbitrary(n) data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_121( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::block(size,50);
  ary_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::iota( b_vw, 0 );

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with arbitrary(n) data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_122( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::cyclic(size);
  ary_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(b_vw, step_wf(base,step));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with arbitrary(n) data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_123( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  ary_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( b_vw, repeat_wf(base,rep));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with arbitrary(n) data distribution, perform computation,
// Force redistribution to balance, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_124( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::balance(size);
  ary_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::generate( b_vw, stapl::random_sequence(1000));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - array
// Start with explict(n) data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_125( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::arbitrary(size,20); // FIXME
  ary_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  ary_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::serial_io(get_val_wf(zin), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}


#ifdef MAP
///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with block(n) data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_201( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::block(size,50);
  map_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( a_vw, 0 );
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate( b_vw, step_wf(base,step));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
  stapl::do_once( msg( "AFTER" ) );

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int max_val = stapl::map_reduce( id_int_wf(), max_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return max_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with block(n) data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_202( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::cyclic(size);
  map_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));
  int rep = 42;
  base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(b_vw, repeat_wf(base,rep));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
  stapl::do_once( msg( "BEFORE" ) );
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int max_val = stapl::map_reduce( id_int_wf(), max_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return max_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with block(n) data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_203( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  map_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(a_vw, repeat_wf(base,rep));
  stapl::generate( b_vw, stapl::random_sequence(1000));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int max_val = stapl::map_reduce( id_int_wf(), max_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return max_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with block(n) data distribution, perform computation,
// Force redistribution to balance, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_204( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::balance(size);
  map_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( a_vw, repeat_wf(base,rep));
  stapl::generate( b_vw, stapl::random_sequence(1000));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int max_val = stapl::map_reduce( id_int_wf(), max_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return max_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with block(n) data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_205( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::arbitrary(size,20);
  map_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::generate( a_vw, stapl::random_sequence(1000));
  stapl::iota( b_vw, 0 );

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int max_val = stapl::map_reduce( id_int_wf(), max_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return max_val;
#endif
}


///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with cyclic data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_206( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::cyclic(size);
  distribution_spec new_dist = stapl::block(size,50);
  map_int_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( b_vw, 0 );
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate( a_vw, step_wf(base,step));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int min_val = stapl::map_reduce( id_int_wf(), min_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return min_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_207( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::cyclic(size);
  map_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(b_vw, step_wf(base,step));
  int rep = 42;
  base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(a_vw, repeat_wf(base,rep));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int min_val = stapl::map_reduce( id_int_wf(), min_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return min_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with cyclic data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_208( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::cyclic(size);
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  map_int_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( b_vw, repeat_wf(base,rep));
  stapl::generate( a_vw, stapl::random_sequence(1000));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int min_val = stapl::map_reduce( id_int_wf(), min_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return min_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with cyclic data distribution, perform computation,
// Force redistribution to balance, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_209( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::cyclic(size);
  distribution_spec new_dist = stapl::balance(size);
  map_int_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( b_vw, 0 );
  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( a_vw, repeat_wf(base,rep));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int min_val = stapl::map_reduce( id_int_wf(), min_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return min_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with cyclic data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_210( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::cyclic(size);
  distribution_spec new_dist = stapl::arbitrary(size,20);
  map_int_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::generate( b_vw, stapl::random_sequence(1000));
  stapl::iota( a_vw, 0 );

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, add_int_wf() );
  int min_val = stapl::map_reduce( id_int_wf(), min_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return min_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_211( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::block(size,50);
  map_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::iota( a_vw, 0 );
  stapl::serial_io(get_val_wf(zin), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, sub_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_212( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::cyclic(size);
  map_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));
  stapl::serial_io(get_val_wf(zin), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, sub_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_213( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  map_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(a_vw, repeat_wf(base,rep));
  stapl::serial_io(get_val_wf(zin), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, sub_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to balance, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_214( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::balance(size);
  map_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::generate( a_vw, stapl::random_sequence(1000));
  stapl::serial_io(get_val_wf(zin), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, sub_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_215( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::arbitrary(size,20);
  map_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::serial_io(get_val_wf(zin), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, sub_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with balance data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_216( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::block(size,50);
  map_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::iota( b_vw, 0 );

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with balance data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_217( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::cyclic(size);
  map_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(b_vw, step_wf(base,step));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with balance data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_218( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  map_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( b_vw, repeat_wf(base,rep));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with balance data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_219( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::block(size,20);
  map_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::generate( b_vw, stapl::random_sequence(1000));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with balance data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_220( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::arbitrary(size,20); // FIXME
  map_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::serial_io(get_val_wf(zin), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with arbitrary(n) data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_221( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::block(size,50);
  map_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::iota( b_vw, 0 );

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with arbitrary(n) data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_222( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::cyclic(size);
  map_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(b_vw, step_wf(base,step));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with arbitrary(n) data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_223( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  map_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( b_vw, repeat_wf(base,rep));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with arbitrary(n) data distribution, perform computation,
// Force redistribution to balance, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_224( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::balance(size);
  map_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::generate( b_vw, stapl::random_sequence(1000));

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - map
// Start with explict(n) data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_225( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::arbitrary(size,20); // FIXME
  map_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  map_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::serial_io(get_val_wf(zin), b_vw);

  size_t cksum = stapl::map_reduce( id_un_wf(), xor_un_wf(), a_vw );

  stapl::do_once( msg( "BEFORE" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif
#ifdef RE_CKIN
  a.redistribute(new_dist);
#endif
  stapl::do_once( msg( "AFTER" ) );
#ifdef MAP_OUT
  stapl::serial_io(show_log_assoc_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
#endif

  stapl::transform( a_vw, b_vw, c_vw, max_int_wf() );
  int sum_val = stapl::map_reduce( id_int_wf(), add_int_wf(), c_vw );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return sum_val;
#endif
}


#endif
///////////////////////////////////////////////////////////////////////////

struct set_prop_wf
{
  typedef void result_type;
  template <typename Vertex, typename Elem>
  void operator()(Vertex vtx, Elem elem)
  {
    vtx.property() = elem;
  }
};

struct get_prop_wf
{
  typedef int result_type;
  template <typename Vertex>
  result_type operator()(Vertex vtx)
  {
    return vtx.property();
  }
};

struct vtx_property
{
  typedef int result_type;
  template<typename Vertex>
  result_type operator() (Vertex &v) const {
    return v.property();
  }
};

struct sum_adj_prop_wf
{
  typedef int result_type;
  template <typename Vertex, typename Graph>
  result_type operator()(Vertex src, Graph gr_vw)
  {
    result_type sum = 0;
    typename Vertex::adj_edge_iterator edge_it;
    for (edge_it = src.begin(); edge_it != src.end(); ++edge_it ) {
      size_t target = (*edge_it).target();
      sum += gr_vw.apply_get(target, vtx_property() );
    }
    return sum;
  }
};

template <typename Container>
void build_edges(int size, Container &gr)
{
  // link each fibonacci to its adjacent
  for ( int i=1; i<20; i++ ) {
    int src = fibo20[i-1];
    int dest = fibo20[i];
    if ( src < size && dest << size ) {
      gr.add_edge(src,dest);
    }
  }
  // link each prime to its adjacent
  for ( int i=1; i<size; i++ ) {
    int src = prime_nums[i-1];
    int dest = prime_nums[i];
    if ( src < size && dest << size ) {
      gr.add_edge(src,dest);
    }
  }
  // link odds to adjacent odds and evens to adjacent evens
  for ( int i=2; i<size-1; i+=2 ) {
    int src = i-2;
    int dest = i;
    if ( src < size && dest << size ) {
      gr.add_edge(src,dest);
      gr.add_edge(src+1,dest+1);
    }
  }
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with block(n) data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_301( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::block(size,50);
  dygraf_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // assign node weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::iota( v_vw, 0 );
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate( w_vw, step_wf(base,step));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_blk_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                      stapl::make_repeat_view<dygraf_int_blk_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with block(n) data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_302( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::cyclic(size);
  dygraf_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(v_vw, step_wf(base,step));
  int rep = 42;
  base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(w_vw, repeat_wf(base,rep));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_blk_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with block(n) data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_303( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  dygraf_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( v_vw, repeat_wf(base,rep));
  stapl::generate( w_vw, stapl::random_sequence(1000));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_blk_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with block(n) data distribution, perform computation,
// Force redistribution to balance, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_304( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::balance(size);
  dygraf_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( v_vw, repeat_wf(base,rep));
  stapl::generate( w_vw, stapl::random_sequence(1000));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_blk_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with block(n) data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_305( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::block(size,10);
  distribution_spec new_dist = stapl::arbitrary(size,20);
  dygraf_int_blk_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_blk_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::generate( v_vw, stapl::random_sequence(1000));
  stapl::iota( w_vw, 0 );
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_blk_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
#endif
}


///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with cyclic data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_306( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::cyclic(size);
  distribution_spec new_dist = stapl::block(size,50);
  dygraf_int_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::iota( v_vw, 0 );
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate( w_vw, step_wf(base,step));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_cyc_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_cyc_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_cyc_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with cyclic(n) data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_307( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::cyclic(size,10);
  distribution_spec new_dist = stapl::cyclic(size);
  dygraf_int_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(v_vw, step_wf(base,step));
  int rep = 42;
  base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(w_vw, repeat_wf(base,rep));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_cyc_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_cyc_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_cyc_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with cyclic data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_308( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::cyclic(size);
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  dygraf_int_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( v_vw, repeat_wf(base,rep));
  stapl::generate( w_vw, stapl::random_sequence(1000));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_cyc_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_cyc_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_cyc_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with cyclic data distribution, perform computation,
// Force redistribution to balance, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_309( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::cyclic(size);
  distribution_spec new_dist = stapl::balance(size);
  dygraf_int_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::iota( v_vw, 0 );
  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( w_vw, repeat_wf(base,rep));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_cyc_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_cyc_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_cyc_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with cyclic data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_310( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::cyclic(size);
  distribution_spec new_dist = stapl::arbitrary(size,20);
  dygraf_int_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::generate( v_vw, stapl::random_sequence(1000));
  stapl::iota( w_vw, 0 );
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_cyc_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_cyc_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_cyc_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_311( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::block(size,50);
  dygraf_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::iota( v_vw, 0 );
  stapl::serial_io(get_val_wf(zin), w_vw);
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_blk_cyc_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_cyc_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_cyc_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_312( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::cyclic(size);
  dygraf_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(v_vw, step_wf(base,step));
  stapl::serial_io(get_val_wf(zin), w_vw);
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_blk_cyc_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_cyc_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_cyc_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_313( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  dygraf_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(v_vw, repeat_wf(base,rep));
  stapl::serial_io(get_val_wf(zin), w_vw);
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_blk_cyc_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_cyc_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_cyc_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to balance, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_314( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::balance(size);
  dygraf_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::generate( v_vw, stapl::random_sequence(1000));
  stapl::serial_io(get_val_wf(zin), w_vw);
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_blk_cyc_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_cyc_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_cyc_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with block_cyclic(n) data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_315( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::block_cyclic(size,10);
  distribution_spec new_dist = stapl::arbitrary(size,20);
  dygraf_int_blk_cyc_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_blk_cyc_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::serial_io(get_val_wf(zin), v_vw);
  stapl::serial_io(get_val_wf(zin), w_vw);
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_blk_cyc_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_cyc_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_blk_cyc_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with balance data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_316( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::block(size,50);
  dygraf_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::serial_io(get_val_wf(zin), v_vw);
  stapl::iota( w_vw, 0 );
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_bal_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_bal_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_bal_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with balance data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_317( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::cyclic(size);
  dygraf_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::serial_io(get_val_wf(zin), v_vw);
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(w_vw, step_wf(base,step));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_bal_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_bal_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_bal_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with balance data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_318( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  dygraf_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::serial_io(get_val_wf(zin), v_vw);
  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( w_vw, repeat_wf(base,rep));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_bal_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_bal_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_bal_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with balance data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_319( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::block(size,20);
  dygraf_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::serial_io(get_val_wf(zin), v_vw);
  stapl::generate( w_vw, stapl::random_sequence(1000));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_bal_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_bal_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_bal_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with balance data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_320( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::balance(size);
  distribution_spec new_dist = stapl::arbitrary(size,20); // FIXME
  dygraf_int_bal_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_bal_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::serial_io(get_val_wf(zin), v_vw);
  stapl::serial_io(get_val_wf(zin), w_vw);
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_bal_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_bal_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_bal_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with arbitrary(n) data distribution, perform computation,
// Force redistribution to block(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_321( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::block(size,50);
  dygraf_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::serial_io(get_val_wf(zin), v_vw);
  stapl::iota( w_vw, 0 );
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_expl_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_expl_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_expl_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with arbitrary(n) data distribution, perform computation,
// Force redistribution to cyclic, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_322( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;
  zout.open("red322.zout");

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::cyclic(size);
  dygraf_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::serial_io(get_val_wf(zin), v_vw);
  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(w_vw, step_wf(base,step));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_expl_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_expl_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_expl_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with arbitrary(n) data distribution, perform computation,
// Force redistribution to block_cyclic(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_323( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::block_cyclic(size,20);
  dygraf_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::serial_io(get_val_wf(zin), v_vw);
  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate( w_vw, repeat_wf(base,rep));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_expl_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_expl_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_expl_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with arbitrary(n) data distribution, perform computation,
// Force redistribution to balance, perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_324( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::balance(size);
  dygraf_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::serial_io(get_val_wf(zin), v_vw);
  stapl::generate( w_vw, stapl::random_sequence(1000));
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_expl_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_expl_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_expl_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
#endif
}

///////////////////////////////////////////////////////////////////////////
// Specify parallel indexed container - graph
// Start with explict(n) data distribution, perform computation,
// Force redistribution to arbitrary(m), perform further computation.
///////////////////////////////////////////////////////////////////////////

bool redistrib_325( int model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout ) {

  size_t size = 1000 * model;

#ifdef EXPLICIT
  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  // build graph and vertices
  distribution_spec old_dist = stapl::arbitrary(size,10); // FIXME
  distribution_spec new_dist = stapl::arbitrary(size,20); // FIXME
  dygraf_int_expl_tp a(old_dist), b(old_dist), c(old_dist);
  dygraf_int_expl_vw_tp a_vw(a), b_vw(b), c_vw(c);

  // set vertex weights
  vec_int_tp v(size), w(size);
  vec_int_vw_tp v_vw(v), w_vw(w);
  stapl::serial_io(get_val_wf(zin), v_vw);
  stapl::serial_io(get_val_wf(zin), b_vw);
  stapl::map_func( set_prop_wf(), a_vw, v_vw );
  stapl::map_func( set_prop_wf(), b_vw, w_vw );

  // build edges
  if ( stapl::get_location_id() == 0 ) {
    build_edges<dygraf_int_expl_tp>(size, a);
  }
  stapl::rmi_fence();

  int before = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_expl_vw_tp>(a_vw) );

  // display sample, redistribute, display sample
  stapl::do_once( msg( "BEFORE" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );
  a.redistribute(new_dist);
  stapl::do_once( msg( "AFTER" ) );
  stapl::serial_io(show_log_graf_wf(zout), a_vw,
                   stapl::counting_view<int>(size) );

  int after = stapl::map_reduce( sum_adj_prop_wf(), add_int_wf(), a_vw,
                     stapl::make_repeat_view<dygraf_int_expl_vw_tp>(a_vw) );

  stapl::rmi_fence();
  ctr.stop();
  double time1 = ctr.value();

  return before == after;
#endif
}

