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

#include <stapl/utility/tuple.hpp>
#include <stapl/domains/indexed.hpp>

#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>

#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>

#include <stapl/views/map_view.hpp>
#include <stapl/containers/map/map.hpp>

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

#if 0
// Uncomment these defines when the memory footprint of compilation has been
// reduced.  Currently, nestpar3_homogeneous.cc defines HOMOGENEOUS and then
// includes this file, while nestpar3_heterogeneous.cc defines HETEROGENEOUS
// and includes the file.
#define HOMOGENEOUS 1
#define HETEROGENEOUS 1
#endif

#define FILE_INPUT 1
#define GEN_INPUT 1
//#undef FILL_MAP

/*=========================================================================*/

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector< vec_int_tp > vec_2_int_tp;
typedef stapl::vector< vec_2_int_tp > vec_3_int_tp;

typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;
typedef stapl::vector_view<vec_2_int_tp> vec_2_int_vw_tp;
typedef stapl::vector_view<vec_3_int_tp> vec_3_int_vw_tp;

typedef stapl::array<int> ary_int_tp;
typedef stapl::array< ary_int_tp > ary_2_int_tp;
typedef stapl::array< ary_2_int_tp > ary_3_int_tp;

typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;
typedef stapl::array_view<ary_2_int_tp> ary_2_int_vw_tp;
typedef stapl::array_view<ary_3_int_tp> ary_3_int_vw_tp;

typedef stapl::array<size_t> ary_sz_tp;
typedef stapl::array<ary_sz_tp> ary_2_sz_tp;

typedef stapl::array_view<ary_sz_tp> ary_sz_vw_tp;
typedef stapl::array_view<ary_2_sz_tp> ary_2_sz_vw_tp;

typedef stapl::vector<size_t> vec_sz_tp;
typedef stapl::vector< vec_sz_tp > vec_2_sz_tp;

typedef stapl::vector_view< vec_sz_tp > vec_sz_vw_tp;
typedef stapl::vector_view< vec_2_sz_tp > vec_2_sz_vw_tp;

typedef stapl::indexed_domain<int> ndx_dom_tp;

/*=========================================================================*/

bool open_zin(int model, int test, stapl::stream<ifstream>& zin, int &count) {
  string path;
  switch ( model ) {
  case 1:
    switch( test ) {
    case 317: case 318: case 319:
    case 320: case 321: case 322:
      path = "data/tiny_bits.zin";
      break;
    default:
      path = "data/tiny_primes.zin";
      break;
    }
    break;
  case 100:
    switch( test ) {
    case 317: case 318: case 319:
    case 320: case 321: case 322:
      path = "data/small_bits.zin";
      break;
    default:
      path = "data/small_primes.zin";
      break;
    }
    break;
  case 10000:
    switch( test ) {
    case 317: case 318: case 319:
    case 320: case 321: case 322:
      path = "data/medium_bits.zin";
      break;
    default:
      path = "data/medium_primes.zin";
      break;
    }
    break;
  case 1000000:
    switch( test ) {
    case 317: case 318: case 319:
    case 320: case 321: case 322:
      path = "data/big_bits.zin";
      break;
    default:
      path = "data/big_primes.zin";
      break;
    }
    break;
  case 100000000:
    switch( test ) {
    case 317: case 318: case 319:
    case 320: case 321: case 322:
      path = "data/huge_bits.zin";
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
        cerr << "Unable to open metadata file: data/meta_data" << endl;
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
#ifdef HOMOGENEOUS
#ifdef GEN_INPUT
size_t nestpar_301( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_302( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_303( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_304( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_305( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_306( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
#endif
#ifdef FILE_INPUT
size_t nestpar_307( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_308( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_309( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_310( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_311( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_312( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_313( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_314( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
#endif // FILE_INPUT
#endif // HOMOGENEOUS

#ifdef HETEROGENEOUS
#ifdef GEN_INPUT
size_t nestpar_315( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_316( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
#endif
#ifdef FILL_MAP
size_t nestpar_317( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_318( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_319( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_320( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_321( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
#endif // FILL_MAP
#endif // HETEROGENEOUS

#ifdef HOMOGENEOUS
#ifdef FILL_MAP
size_t nestpar_322( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
#endif // FILL_MAP
#endif // HOMOGENEOUS
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
  data_cnt = model * 1000;

  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;

  int first_test = 301;
  int last_test = 316;
  if (opt_test != -1 ) {
    first_test = opt_test;
    last_test = opt_test;
  }

  bool ok = true;
  int count = 0;
  for ( int test=first_test; test<=last_test; test++ ) {
#define DEBUG 1
#ifdef DEBUG
    stapl::do_once([&](){
        std::cout << "Nested Parallel Run " << test << endl;
    });
#endif
    set_random_seed();
    int result = 0;
    switch ( test) {
#ifdef HOMOGENEOUS
#ifdef GEN_INPUT
    case 301:
      zout.open("np301.zout");
      result = nestpar_301(model, zin, zout);
      break;
    case 302:
      zout.open("np302.zout");
      result = nestpar_302(model, zin, zout);
      break;
    case 303:
      zout.open("np303.zout");
      result = nestpar_303(model, zin, zout);
      break;
    case 304:
      zout.open("np304.zout");
      result = nestpar_304(model, zin, zout);
      break;
    case 305:
      zout.open("np305.zout");
      result = nestpar_305(model, zin, zout);
      break;
    case 306:
      zout.open("np306.zout");
      result = nestpar_306(model, zin, zout);
      break;
#endif
#ifdef FILE_INPUT
    case 307:
      zout.open("np307.zout");
      if ( open_zin(model,307,zin,count) ) {
        result = nestpar_307(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 308:
      zout.open("np308.zout");
      if ( open_zin(model,308,zin,count) ) {
        result = nestpar_308(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 309:
      zout.open("np309.zout");
      if ( open_zin(model,309,zin,count) ) {
        result = nestpar_309(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 310:
      zout.open("np310.zout");
      if ( open_zin(model,310,zin,count) ) {
        result = nestpar_310(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 311:
      zout.open("np311.zout");
      if ( open_zin(model,311,zin,count) ) {
        result = nestpar_311(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 312:
      zout.open("np312.zout");
      if ( open_zin(model,312,zin,count) ) {
        result = nestpar_312(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 313:
      zout.open("np313.zout");
      if ( open_zin(model,313,zin,count) ) {
        result = nestpar_313(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 314:
      zout.open("np314.zout");
      if ( open_zin(model,314,zin,count) ) {
        result = nestpar_314(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif // FILE_INPUT
#endif // HOMOGENEOUS

#ifdef HETEROGENEOUS
#ifdef GEN_INPUT
    case 315:
      zout.open("np315.zout");
      if ( open_zin(model,315,zin,count) ) {
        result = nestpar_315(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 316:
      zout.open("np316.zout");
      if ( open_zin(model,316,zin,count) ) {
        result = nestpar_316(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif
#ifdef FILL_MAP
    case 317:
      zout.open("np317.zout");
      if ( open_zin(model,317,zin,count) ) {
        result = nestpar_317(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 318:
      zout.open("np318.zout");
      if ( open_zin(model,318,zin,count) ) {
        result = nestpar_318(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 319:
      zout.open("np319.zout");
      if ( open_zin(model,319,zin,count) ) {
        result = nestpar_319(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 320:
      zout.open("np320.zout");
      if ( open_zin(model,320,zin,count) ) {
        result = nestpar_320(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 321:
      zout.open("np321.zout");
      if ( open_zin(model,321,zin,count) ) {
        result = nestpar_321(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif // FILL_MAP
#endif // HETEROGENEOUS

#ifdef HOMOGENEOUS
#ifdef FILL_MAP
    case 322:
      zout.open("np322.zout");
      if ( open_zin(model,322,zin,count) ) {
        result = nestpar_322(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif // HOMOGENEOUS
#endif // FILL_MAP
    }
    zout.close();
#ifdef DEBUG
    stapl::do_once([&](){
      std::cout << "Nested Parallel Run: " << result << endl;
    });
#endif
  }
  return EXIT_SUCCESS;
}

typedef stapl::vector<ary_int_tp> vec_ary_int_tp;
typedef stapl::array<vec_ary_int_tp> ary_vec_ary_int_tp;
typedef stapl::array<vec_int_tp> ary_vec_int_tp;
typedef stapl::vector<ary_vec_int_tp> vec_ary_vec_int_tp;
typedef stapl::array_view<ary_vec_ary_int_tp> ary_vec_ary_int_vw_tp;
typedef stapl::vector_view<vec_ary_vec_int_tp> vec_ary_vec_int_vw_tp;

typedef stapl::map<int,int> map_int_tp;
typedef stapl::map_view<map_int_tp> map_int_vw_tp;

typedef stapl::array<map_int_tp> ary_map_int_tp;

typedef stapl::vector<ary_map_int_tp> vec_ary_map_int_tp;
typedef stapl::vector_view<vec_ary_map_int_tp> vec_ary_map_int_vw_tp;
typedef stapl::vector<map_int_tp> vec_map_int_tp;
typedef stapl::array<vec_map_int_tp> ary_vec_map_int_tp;
typedef stapl::array_view<vec_ary_map_int_tp> ary_vec_map_int_vw_tp;
typedef stapl::map<int,ary_int_tp> map_ary_int_tp;
typedef stapl::vector<map_ary_int_tp> vec_map_ary_int_tp;
typedef stapl::vector_view<vec_map_ary_int_tp> vec_map_ary_int_vw_tp;
typedef stapl::map<int,ary_vec_int_tp> map_ary_vec_int_tp;
typedef stapl::map_view<map_ary_vec_int_tp> map_ary_vec_int_vw_tp;
typedef stapl::map<int,map_int_tp> map_map_int_tp;
typedef stapl::vector<map_map_int_tp> vec_map_map_int_tp;
typedef stapl::vector_view<vec_map_map_int_tp> vec_map_map_int_vw_tp;

typedef stapl::map<int,map_int_tp> map_map_int_tp;
typedef stapl::map<int,map_map_int_tp> map_map_map_int_tp;
typedef stapl::map_view<map_map_map_int_tp> map_map_map_int_vw_tp;

typedef stapl::vector<map_int_tp> vec_map_int_tp;
typedef stapl::map<int,vec_map_int_tp> map_vec_map_int_tp;
typedef stapl::map_view<map_vec_map_int_tp> map_vec_map_int_vw_tp;

typedef stapl::map<int,vec_ary_int_tp> map_vec_ary_int_tp;
typedef stapl::map_view<map_vec_ary_int_tp> map_vec_ary_int_vw_tp;

#ifdef HOMOGENEOUS

/*=========================================================================
 * homogeneous nested structures
 *=========================================================================*/

#ifdef GEN_INPUT
//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(vec(int))
// construct vector with generator, traverse with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_301_inner_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::iota( vw1, 0 );
  }
};

struct nestpar_301_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_301_inner_fill_wf(), vw1);
  }
};

struct nestpar_301_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_301_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_301_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_301_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_301_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_301( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

stapl::do_once([&](){
    std::cout << "nestpar 301 before" << endl;
});

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

stapl::do_once([&](){
    std::cout << "nestpar 301 after" << endl;
});

  vec_3_int_tp a(len_2_vw), b(len_2_vw);
  vec_3_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_301_outer_fill_wf(), a_vw );

  stapl::serial_io(nestpar_301_outer_show_wf(zout), a_vw );

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP301","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(ary(int))
// construct array with generator, traverse with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_302_inner_fill_wf {
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

struct nestpar_302_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_302_inner_fill_wf(), vw1);
  }
};

struct nestpar_302_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_302_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_302_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_302_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_302_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_302( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

stapl::do_once([&](){
    std::cout << "nestpar 302 before" << endl;
});

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

stapl::do_once([&](){
    std::cout << "nestpar 302 after" << endl;
});

  ary_3_int_tp a(len_2_vw), b(len_2_vw);
  ary_3_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_302_outer_fill_wf(), a_vw);

  stapl::serial_io(nestpar_302_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP302","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(vec(int))
// construct vector with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_303_inner_fill_wf {
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

struct nestpar_303_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_303_inner_fill_wf(), vw1);
  }
};

struct nestpar_303_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_303_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_303_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_303_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_303_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_303( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

stapl::do_once([&](){
    std::cout << "nestpar 303 before" << endl;
});

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

stapl::do_once([&](){
    std::cout << "nestpar 303 after" << endl;
});

  vec_3_int_tp a(len_2_vw), b(len_2_vw);
  vec_3_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_303_outer_fill_wf(), a_vw );

  stapl::serial_io(nestpar_303_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP303","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(ary(int))
// construct array with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_304_inner_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
     stapl::generate( vw1, stapl::random_sequence());
  }
};

struct nestpar_304_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_304_inner_fill_wf(), vw1);
  }
};

struct nestpar_304_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_304_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_304_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_304_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_304_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_304( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

stapl::do_once([&](){
    std::cout << "nestpar 304 before" << endl;
});

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

stapl::do_once([&](){
    std::cout << "nestpar 304 after" << endl;
});

  ary_3_int_tp a(len_2_vw), b(len_2_vw);
  ary_3_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_304_outer_fill_wf(), a_vw );

  stapl::serial_io(nestpar_304_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP304","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(vec(int))
// construct vector with generator, process with stapl algorithm
// apply unary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_305_inner_fill_wf {
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

struct nestpar_305_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_305_inner_fill_wf(), vw1);
  }
};

struct nestpar_305_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::transform(vw1, vw2, neg_int_wf());
  }
};

struct nestpar_305_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_305_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_305_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_305_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_305_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_305_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_305_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_305( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

stapl::do_once([&](){
    std::cout << "nestpar 305 before" << endl;
});

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

stapl::do_once([&](){
    std::cout << "nestpar 305 after" << endl;
});

  vec_3_int_tp a(len_2_vw), b(len_2_vw);
  vec_3_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_305_outer_fill_wf(), a_vw );
  stapl::map_func(nestpar_305_outer_fill_wf(), b_vw );

  stapl::serial_io(nestpar_305_outer_show_wf(zout), a_vw);

  stapl::map_func(nestpar_305_outer_process_wf(), a_vw, b_vw );

  stapl::serial_io(nestpar_305_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP305","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(ary(int))
// construct array with generator, process with stapl algorithm
// apply binary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_306_inner_fill_wf {
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

struct nestpar_306_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_306_inner_fill_wf(), vw1);
  }
};

struct nestpar_306_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const &vw1, View2 const &vw2, View3 const &vw3)
  {
    stapl::transform(vw1, vw2, vw3, add_int_wf());
  }
};

struct nestpar_306_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 vw1, View2 vw2, View3 vw3)
  {
    stapl::map_func(nestpar_306_inner_process_wf(), vw1, vw2, vw3);
  }
};

struct nestpar_306_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_306_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_306_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_306_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_306_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_306( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

stapl::do_once([&](){
    std::cout << "nestpar 306 before" << endl;
});

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

stapl::do_once([&](){
    std::cout << "nestpar 306 after" << endl;
});

  ary_3_int_tp a(len_2_vw), b(len_2_vw), c(len_2_vw);
  ary_3_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::map_func(nestpar_306_outer_fill_wf(), a_vw );
  stapl::map_func(nestpar_306_outer_fill_wf(), b_vw );

  stapl::serial_io(nestpar_306_outer_show_wf(zout), a_vw);

  stapl::serial_io(nestpar_306_outer_show_wf(zout), b_vw);

  stapl::map_func(nestpar_306_outer_process_wf(), a_vw, b_vw, c_vw );

  stapl::serial_io(nestpar_306_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP306","STAPL",time_p,time_i+time_o);

  return cksum;
}
#endif // GEN_INPUT

#ifdef FILE_INPUT

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(vec(int))
// construct vector with serial I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_307_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_307_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_307_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_307_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_307_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_307_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_307_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_307_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_307_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_307_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_307_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_307_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_307_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_307( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

  vec_3_int_tp a(len_2_vw), b(len_2_vw);
  vec_3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_307_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_307_outer_process_wf(), a_vw, b_vw );

  stapl::serial_io(nestpar_307_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP307","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(ary(int))
// construct array with serial I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_308_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_308_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_308_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_308_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_308_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_308_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_308_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_308_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_308_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_308_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_308_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_308_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_308_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_308( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

  ary_3_int_tp a(len_2_vw), b(len_2_vw);
  ary_3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_308_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_308_outer_process_wf(), a_vw, b_vw);

  stapl::serial_io(nestpar_308_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP308","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(vec(int))
// construct vector with serial I/O,
// process with mapreduce / work function, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_309_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_309_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_309_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_309_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_309_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_309_inner_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_309_outer_process_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_309_inner_process_wf(), vw1);
  }
};

struct nestpar_309_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_309_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_309_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_309_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_309_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_309( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

  vec_3_int_tp a(len_2_vw), b(len_2_vw);
  vec_3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_309_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_309_outer_process_wf(), a_vw );

  stapl::serial_io(nestpar_309_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP309","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(ary(int))
// construct array with serial I/O,
// process with mapreduce / work function, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_310_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_310_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_310_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_310_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_310_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_310_inner_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_310_outer_process_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_310_inner_process_wf(), vw1);
  }
};

struct nestpar_310_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_310_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_310_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_310_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_310_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_310( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

  ary_3_int_tp a(len_2_vw), b(len_2_vw);
  ary_3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_310_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_310_outer_process_wf(), a_vw );

  stapl::serial_io(nestpar_310_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP310","STAPL",time_p,time_i+time_o);

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

struct nestpar_311_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_311_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_311_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_311_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_311_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_311_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func( user1_wf(), vw1, vw2 );
  }
};

struct nestpar_311_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_311_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_311_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_311_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_311_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_311_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_311_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_311( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

  vec_3_int_tp a(len_2_vw), b(len_2_vw);
  vec_3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_311_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_311_outer_process_wf(), a_vw, b_vw);

  stapl::serial_io(nestpar_311_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP311","STAPL",time_p,time_i+time_o);

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

struct nestpar_312_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_312_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_312_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_312_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_312_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_312_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const &vw1, View2 const &vw2, View3 const &vw3)
  {
    stapl::map_func( user2_wf(), vw1, vw2, vw3 );
  }
};

struct nestpar_312_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 vw1, View2 vw2, View3 vw3)
  {
    stapl::map_func(nestpar_312_inner_process_wf(), vw1, vw2, vw3);
  }
};

struct nestpar_312_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_312_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_312_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_312_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_312_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


size_t nestpar_312( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

  ary_3_int_tp a(len_2_vw), b(len_2_vw), c(len_2_vw);
  ary_3_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(nestpar_312_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_312_outer_process_wf(), a_vw, b_vw, c_vw );

  stapl::serial_io(nestpar_312_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP312","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(vec(int))
// construct vector with serial I/O,
// process with partial_sum, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_313_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_313_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_313_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_313_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_313_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_313_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_313_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_313_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_313_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_313_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_313_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_313_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_313_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_313( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

  vec_3_int_tp a(len_2_vw), b(len_2_vw);
  vec_3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_313_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_313_outer_process_wf(), a_vw, b_vw );

  stapl::serial_io(nestpar_313_outer_show_wf(zout), a_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP313","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(ary(int))
// construct array with serial I/O,
// process with partial_sum, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_314_inner_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_314_inner_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_314_outer_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_314_outer_fill_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_314_inner_fill_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct nestpar_314_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_314_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_314_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_314_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_314_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_314_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_314_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_314_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_314( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

  ary_3_int_tp a(len_2_vw), b(len_2_vw);
  ary_3_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_314_outer_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_314_outer_process_wf(), a_vw, b_vw );

  stapl::serial_io(nestpar_314_outer_show_wf(zout), b_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP314","STAPL",time_p,time_i+time_o);

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

struct nestpar_315_inner_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::iota( vw1, 0 );
  }
};

struct nestpar_315_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_315_inner_fill_wf(), vw1);
  }
};

struct nestpar_315_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    return vw1.size();
  }
};

struct nestpar_315_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_315_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_315_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_315_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_315_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_315_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_315_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_315( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

  vec_ary_vec_int_tp a(len_2_vw), b(len_2_vw);
  vec_ary_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_315_outer_fill_wf(), a_vw );

  stapl::map_func(nestpar_315_outer_process_wf(), a_vw, b_vw );

  stapl::serial_io(nestpar_315_outer_show_wf(zout), b_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP315","STAPL",time_p,time_i+time_o);

  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(vec(ary(int
//////////////////////////////////////////////////////////////////////

struct nestpar_316_inner_fill_wf {
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

struct nestpar_316_outer_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::map_func(nestpar_316_inner_fill_wf(), vw1);
  }
};

struct nestpar_316_inner_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    return vw1.size();
  }
};

struct nestpar_316_outer_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 vw1, View2 vw2)
  {
    stapl::map_func(nestpar_316_inner_process_wf(), vw1, vw2);
  }
};

struct nestpar_316_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_316_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_316_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_316_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_316_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_316( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

  ary_vec_ary_int_tp a(len_2_vw), b(len_2_vw);
  ary_vec_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_316_outer_fill_wf(), a_vw );

  stapl::map_func(nestpar_316_outer_process_wf(), a_vw, b_vw );

  stapl::serial_io(nestpar_316_outer_show_wf(zout), b_vw);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP316","STAPL",time_p,time_i+time_o);

  return cksum;
}
#endif

#ifdef FILL_MAP
/*=========================================================================
 * heterogeneous positional & non-positional nested structures
 *=========================================================================*/

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(ary(map(int
//////////////////////////////////////////////////////////////////////

struct nestpar_317_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_317_fill_wf(size_t o, size_t m, size_t i)
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

struct nestpar_317_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_317_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_317_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_317_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_317_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_317( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

#if 1
  vec_ary_map_int_tp a(len_1_vw), b(len_1_vw);
  vec_ary_map_int_vw_tp a_vw(a), b_vw(b);
  ndx_dom_tp map_dom(0,16277216);
#else
  vec_ary_map_int_tp a(len_2_vw), b(len_2_vw);
  vec_ary_map_int_vw_tp a_vw(a), b_vw(b);
#endif

  stapl::do_once(nestpar_317_fill_wf(outer,middle,inner), a_vw );

  stapl::serial_io(nestpar_317_outer_show_wf(zout), b_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP317","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(map(ary(int
//////////////////////////////////////////////////////////////////////

struct nestpar_318_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_318_fill_wf(size_t o, size_t m, size_t i)
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

struct nestpar_318_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_318_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_318_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_318_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_318_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_318( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_map_ary_int_tp a(outer), b(outer);
  vec_map_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_318_fill_wf(outer,middle,inner), a_vw );

  stapl::serial_io(nestpar_318_outer_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP318","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_seq_l1_map_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(ary(vec(int
//////////////////////////////////////////////////////////////////////

struct nestpar_319_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_319_fill_wf(size_t o, size_t m, size_t i)
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

struct nestpar_319_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_319_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_319_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_319_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_319_inner_show_wf(m_zout), vw1.second);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_319( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len_1(outer);
  ary_sz_vw_tp len_1_vw(len_1);

  stapl::map_func(inner_roll_wf(), len_1_vw, stapl::make_repeat_view(inner));

  ary_2_sz_tp len_2(len_1);
  ary_2_sz_vw_tp len_2_vw(len_2);

  stapl::map_func(outer_roll_wf(), len_2_vw, stapl::make_repeat_view(outer));

  map_vec_ary_int_tp a, b;
  map_vec_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_319_fill_wf(outer,middle,inner), a_vw, len_2_vw );

  stapl::serial_io(nestpar_319_outer_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP319","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(map(map(int
//////////////////////////////////////////////////////////////////////

struct nestpar_320_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_320_fill_wf(size_t o, size_t m, size_t i)
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

struct nestpar_320_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_320_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_320_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_320_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_320_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_320( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_map_map_int_tp a(outer), b(outer);
  vec_map_map_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_320_fill_wf(outer,middle,inner), a_vw );

  stapl::serial_io(nestpar_320_outer_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP320","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_seq_l1_map_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(vec(map(int
//////////////////////////////////////////////////////////////////////

struct nestpar_321_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_321_fill_wf(size_t o, size_t m, size_t i)
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

struct nestpar_321_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_321_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_321_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_321_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_321_inner_show_wf(m_zout), vw1.second);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_321( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  map_vec_map_int_tp a, b;
  map_vec_map_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_321_fill_wf(outer,middle,inner), a_vw );

  stapl::serial_io(nestpar_321_outer_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP321","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_seq_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

#endif // FILL_MAP

#endif // HETEROGENEOUS

#ifdef HOMOGENEOUS

#ifdef FILL_MAP

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(map(int
//////////////////////////////////////////////////////////////////////

struct nestpar_322_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_322_fill_wf(size_t o, size_t m, size_t i)
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

struct nestpar_322_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_322_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_322_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_322_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_322_inner_show_wf(m_zout), vw1.second);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_322( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  map_map_map_int_tp a, b;
  map_map_map_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_322_fill_wf(outer,middle,inner), a_vw );

  stapl::serial_io(nestpar_322_outer_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP322","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_map_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

#endif // FILL_MAP

#endif // HOMOGENEOUS
