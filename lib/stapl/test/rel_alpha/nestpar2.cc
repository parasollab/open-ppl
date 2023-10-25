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
#include <stapl/views/segmented_view.hpp>

#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>

#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>

#include <stapl/views/map_view.hpp>
#include <stapl/containers/map/map.hpp>

#include <stapl/skeletons/serial.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include <stapl/runtime.hpp>

#include <stapl/utility/do_once.hpp>

#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include "json_parser.hpp"

using namespace std;
#include <stapl/stream.hpp>

#include "testutil.hpp"
#include "rel_alpha_data.h"
#include "rel_alpha_util.hpp"


/*=========================================================================*/

typedef stapl::vector<int> vec_int_tp;
typedef stapl::array<int> ary_int_tp;

typedef stapl::vector< ary_int_tp > vec_ary_int_tp;
typedef stapl::array< vec_int_tp > ary_vec_int_tp;

typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;
typedef stapl::vector_view<vec_ary_int_tp> vec_ary_int_vw_tp;

typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;
typedef stapl::array_view<ary_vec_int_tp> ary_vec_int_vw_tp;

typedef stapl::map<int,int> map_int_tp;
typedef stapl::map<int, stapl::map<int,int> > map_map_int_tp;

typedef stapl::map< int, vec_int_tp > map_vec_int_tp;
typedef stapl::map< int, ary_int_tp > map_ary_int_tp;

typedef stapl::map_view<map_int_tp> map_int_vw_tp;
typedef stapl::map_view<map_map_int_tp> map_map_int_vw_tp;

typedef stapl::map_view<map_vec_int_tp> map_vec_int_vw_tp;
typedef stapl::map_view<map_ary_int_tp> map_ary_int_vw_tp;

typedef stapl::vector<map_int_tp> vec_map_int_tp;
typedef stapl::vector_view<vec_map_int_tp> vec_map_int_vw_tp;

typedef stapl::array<map_int_tp> ary_map_int_tp;
typedef stapl::array_view<ary_map_int_tp> ary_map_int_vw_tp;

typedef stapl::array<size_t> ary_sz_tp;
typedef stapl::array_view<ary_sz_tp> ary_sz_vw_tp;

typedef stapl::indexed_domain<int> ndx_dom_tp;

typedef stapl::splitter_partition<vec_int_vw_tp::domain_type> seg_vec_splitter;
typedef stapl::segmented_view<vec_int_vw_tp, seg_vec_splitter> seg_vec_vw_tp;

typedef stapl::splitter_partition<ary_int_vw_tp::domain_type> seg_ary_splitter;
typedef stapl::segmented_view<ary_int_vw_tp, seg_ary_splitter> seg_ary_vw_tp;

/*=========================================================================*/

size_t bits[256] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

size_t countbits(size_t val) {
  size_t byte1, byte2, byte3;
  byte1 = 0x000000FF & val;
  byte2 = (0x0000FF00 & val) >> 8;
  byte3 = (0x00FF0000 & val) >> 16;
  return bits[byte1] + bits[byte2] + bits[byte3];
}

/*=========================================================================*/

bool open_zin(int model, int test, stapl::stream<ifstream>& zin, int &count)
{
  string path;
  switch ( model ) {
  case 1:
    switch( test ) {
    case 212: case 215: case 218:
      path = "data/tiny_factors.zin";
      break;
    default:
      path = "data/tiny_primes.zin";
      break;
    }
    break;
  case 100:
    switch( test ) {
    case 212: case 215: case 218:
      path = "data/small_factors.zin";
      break;
    default:
      path = "data/small_primes.zin";
      break;
    }
    break;
  case 10000:
    switch( test ) {
    case 212: case 215: case 218:
      path = "data/medium_factors.zin";
      break;
    default:
      path = "data/medium_primes.zin";
      break;
    }
    break;
  case 1000000:
    switch( test ) {
    case 212: case 215: case 218:
      path = "data/big_factors.zin";
      break;
    default:
      path = "data/big_primes.zin";
      break;
    }
    break;
  case 100000000:
    switch( test ) {
    case 212: case 215: case 218:
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
#ifdef DEBUG_FILE
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
#ifdef DEBUG_FILE
      stapl::do_once([&](){
        cerr << "meta_data[" << base_name << "]=" << count << endl;
      });
#endif
      assert( count != 0 );
      return true;
    } else {
      stapl::do_once([&](){
        cerr << "Unable to open meta_data file: data/meta_data" << endl;
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

size_t nestpar_201( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_202( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_203( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_204( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_205( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_206( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_207( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_208( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_209( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_210( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_211( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_212( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_213( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_214( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_215( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_216( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_217( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_218( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_219( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_220( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );

/*=========================================================================*/

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

  int first_test = 201;
  int last_test = 220;
  if (opt_test != -1 ) {
    first_test = opt_test;
    last_test = opt_test;
  }

  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;

  bool ok = true;
  int count = 0;
  for ( int test=first_test; test<=last_test; test++ ) {
#define DEBUG 1
#ifdef DEBUG
    stapl::do_once([&](){
      std::cout << "Nested Parallel Walk " << test << endl;
    });
#endif
    set_random_seed();
    size_t result = 0;
    switch ( test) {
    case 201:
      zout.open("np201.zout");
      result = nestpar_201(model, zin, zout);
      break;
    case 202:
      zout.open("np202.zout");
      result = nestpar_202(model, zin, zout);
      break;
    case 203:
      zout.open("np203.zout");
      result = nestpar_203(model, zin, zout);
      break;
    case 204:
      zout.open("np204.zout");
      result = nestpar_204(model, zin, zout);
      break;
    case 205:
      zout.open("np205.zout");
      result = nestpar_205(model, zin, zout);
      break;
    case 206:
      zout.open("np206.zout");
      result = nestpar_206(model, zin, zout);
      break;
    case 207:
      zout.open("np207.zout");
      result = nestpar_207(model, zin, zout);
      break;
    case 208:
      zout.open("np208.zout");
      result = nestpar_208(model, zin, zout);
      break;
    case 209:
      zout.open("np209.zout");
      result = nestpar_209(model, zin, zout);
      break;
    case 210:
      zout.open("np210.zout");
      if ( open_zin(model,210,zin, count) ) {
        result = nestpar_210(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 211:
      zout.open("np211.zout");
      if ( open_zin(model,211,zin, count) ) {
        result = nestpar_211(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 212:
      zout.open("np212.zout");
      if ( open_zin(model,212,zin, count) ) {
        result = nestpar_212(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 213:
      zout.open("np213.zout");
      if ( open_zin(model,213,zin, count) ) {
        result = nestpar_213(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 214:
      zout.open("np214.zout");
      if ( open_zin(model,214,zin, count) ) {
        result = nestpar_214(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 215:
      zout.open("np215.zout");
      if ( open_zin(model,215,zin, count) ) {
        result = nestpar_215(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 216:
      zout.open("np216.zout");
      if ( open_zin(model,216,zin, count) ) {
        result = nestpar_216(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 217:
      zout.open("np217.zout");
      if ( open_zin(model,217,zin, count) ) {
        result = nestpar_217(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 218:
      zout.open("np218.zout");
      if ( open_zin(model,218,zin, count) ) {
        result = nestpar_218(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 219:
      zout.open("np219.zout");
      if ( open_zin(model,219,zin, count) ) {
        result = nestpar_219(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 220:
      zout.open("np220.zout");
      if ( open_zin(model,220,zin, count) ) {
        result = nestpar_220(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    }
    zout.close();
#ifdef DEBUG
    stapl::do_once([&](){
      std::cout << endl << "Nested Parallel Walk: " << result << endl;
    });
#endif
  }
  return EXIT_SUCCESS;
}

/*=========================================================================
 * heterogeneous nested structures
 *=========================================================================*/

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(ary(int))
// construct vector with generator, traverse with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_201_fill_wf
{
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::iota( vw1, 0 );
  }
};

struct nestpar_201_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return vw1.size();
  }
};

struct nestpar_201_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_201_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_201( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  vec_ary_int_tp a(len_vw), b(len_vw);
  vec_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_201_fill_wf(), a_vw );

  stapl::map_func(nestpar_201_process_wf(), a_vw );

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_201_show_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP201","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(vec(int))
// construct array with generator, traverse with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_202_fill_wf
{
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

struct nestpar_202_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return vw1.size();
  }
};

struct nestpar_202_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_202_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_202( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  ary_vec_int_tp a(len_vw), b(len_vw);
  ary_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_202_fill_wf(), a_vw);

  stapl::map_func(nestpar_202_process_wf(), a_vw );

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_202_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP202","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(ary(int))
// construct map with data from C arrays, traverse with serial
//////////////////////////////////////////////////////////////////////

class nestpar_203_fill_wf
{
private:
  size_t             outer;
  size_t             inner;

public:
   nestpar_203_fill_wf(size_t o, size_t i)
     : outer(o), inner(i)
   { }

  typedef void result_type;
  template<typename Data, typename Length>
  result_type operator()(Data& a, Length& len_vw) const
  {
    if ( outer <= 100000 ) {
      for ( size_t i = 0; i < outer; i++ ) {
        a[ prime_nums[i]].resize(len_vw[i]);
      }
      for ( size_t i = 0; i < outer; i++ ) {
        for ( size_t j = 0; j < len_vw[i]; j++ ) {
          int val = rand_nums[j];
          a[ prime_nums[i] ] [ j ] = countbits(val);
        }
      }
    } else {
      for ( size_t i = 0; i < outer; i += 10000 ) {
        a[ prime_nums[i]].resize(len_vw[i]);
      }
      for ( size_t i = 0; i < outer; i += 10000 ) {
        for ( size_t j = 0; j < len_vw[i]; j++ ) {
          for ( size_t k = 0; k < 10000; k++ ) {
            int val = rand_nums[j];
            a[ prime_nums[k] * prime_nums[10000-k] ][ val ] = countbits(val);
          }
        }
      }
    }
  }
};

struct nestpar_203_process_wf
{
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    typename Element::second_reference arg = elem.second;
    return arg.size();
  }
};

size_t nestpar_203( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  ndx_dom_tp map_dom(0, 16277216);
  map_ary_int_tp a(map_dom), b(map_dom);
  map_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_203_fill_wf(outer, inner), a_vw, len_vw);

  stapl::map_func(nestpar_203_process_wf(), a_vw );

  size_t res = stapl::map_reduce(l1_map_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_map_indexable_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP203","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}


//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(ary(int))
// construct vector with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_204_fill_wf
{
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

struct nestpar_204_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_204_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_204_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_204( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  vec_ary_int_tp a(len_vw), b(len_vw);
  vec_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_204_fill_wf(), a_vw );

  stapl::map_func(nestpar_204_process_wf(), a_vw );

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_204_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP204","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(vec(int))
// construct array with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_205_fill_wf
{
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
     stapl::generate( vw1, stapl::random_sequence());
  }
};

struct nestpar_205_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), vw1 );
  }
};

struct nestpar_205_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_205_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_205( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  ary_vec_int_tp a(len_vw), b(len_vw);
  ary_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_205_fill_wf(), a_vw );

  stapl::map_func(nestpar_205_process_wf(), a_vw );

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_205_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP205","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(vec(int))
// construct map data from C arrays, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_206_process_wf
{
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    typename Element::second_reference map_arg = elem.second;
    int val = stapl::map_reduce( id_int_wf(), add_int_wf(), map_arg);
    return val;
  }
};

size_t nestpar_206( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  ndx_dom_tp map_dom(0, 16277216);
  map_vec_int_tp a(map_dom), b(map_dom);
  map_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_203_fill_wf(outer, inner), a_vw, len_vw);

  stapl::map_func(nestpar_206_process_wf(), a_vw );

  size_t res = stapl::map_reduce(l1_map_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_map_indexable_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP206","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(ary(int))
// construct vector with generator, process with stapl algorithm
// apply unary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_207_fill_wf
{
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

struct nestpar_207_process_wf
{
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::transform(vw1, vw2, neg_int_wf());
  }
};

struct nestpar_207_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_207_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_207( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  vec_ary_int_tp a(len_vw), b(len_vw);
  vec_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_207_fill_wf(), a_vw );

  stapl::serial_io(nestpar_207_show_wf(zout), a_vw);

  stapl::map_func(nestpar_207_process_wf(), a_vw, b_vw);

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_207_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP207","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(vec(int))
// construct array with generator, process with stapl algorithm
// apply binary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_208_fill_wf
{
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

struct nestpar_208_process_wf
{
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const &vw1, View2 const& vw2, View3 const& vw3)
  {
    stapl::transform(vw1, vw2, vw3, add_int_wf());
  }
};

struct nestpar_208_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_208_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_208( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  ary_vec_int_tp a(len_vw), b(len_vw), c(len_vw);
  ary_vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::map_func(nestpar_208_fill_wf(), a_vw);
  stapl::map_func(nestpar_208_fill_wf(), b_vw);

  stapl::serial_io(nestpar_208_show_wf(zout), a_vw);
  stapl::serial_io(nestpar_208_show_wf(zout), b_vw);

  stapl::map_func(nestpar_208_process_wf(), a_vw, b_vw, c_vw);

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_208_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP208","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(map(int))
// construct map data from C arrays, traverse with mapreduce
// ## under construction
//////////////////////////////////////////////////////////////////////

struct nestpar_209_fill_wf
{
  size_t outer;
  size_t inner;

  nestpar_209_fill_wf(size_t o, size_t i)
    : outer(o), inner(i)
  { }

  typedef void result_type;
  template<typename VectorView>
  result_type operator()(VectorView& a) const
  {
    for ( size_t i = 0; i < outer; i++ ) {
      for ( size_t j = 0; j < inner; j++ ) {
        int val = prime_nums[j];
        a[i][val] = countbits(val);
      }
    }
  }
};

struct nestpar_209_process_wf
{
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    int val = stapl::map_reduce( inner_map_elem_wf(), add_int_wf(), elem);
    return val;
  }
};

size_t nestpar_209( size_t model,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_map_int_tp a(outer);
  vec_map_int_vw_tp a_vw(a);

  stapl::do_once(nestpar_209_fill_wf(outer, inner), a_vw);

  stapl::map_func(nestpar_209_process_wf(), a_vw );

  size_t res = stapl::map_reduce(l1_seq_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_indexable_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP209","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(ary(int))
// construct vector with serial I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_210_fill_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_210_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_210_process_wf
{
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_210_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_210_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_210( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  vec_ary_int_tp a(len_vw), b(len_vw);
  vec_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_210_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_210_process_wf(), a_vw, b_vw );

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_210_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP210","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(vec(int))
// construct array with serial I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_211_fill_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_211_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_211_process_wf
{
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_211_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_211_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_211( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  ary_vec_int_tp a(len_vw), b(len_vw);
  ary_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_211_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_211_process_wf(), a_vw, b_vw);

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_211_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP211","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(map(int))
// construct map with serial I/O,
// process with mapreduce / work function, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_212_build_wf
{
  typedef void result_type;
  template <typename SegRef1, typename SegRef2, typename View1>
  result_type operator()(SegRef1 seg1, SegRef2 seg2, View1 vw)
  {
    typename SegRef1::iterator iter1 = seg1.begin();
    typename SegRef2::iterator iter2 = seg2.begin();
    while ( iter1 != seg1.end() ) {
      vw[*iter1++] = *iter2++;
    }
  }
};

struct nestpar_212_process_wf
{
  typedef int result_type;
  template <typename Elem1, typename Elem2>
  result_type operator()(Elem1 elem1, Elem2 elem2)
  {
    elem2 = elem1.size();
  }
};

size_t nestpar_212( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  int limit = count;

  ary_int_tp key(limit), fac(limit), rpt(limit);
  ary_int_vw_tp key_vw(key), fac_vw(fac), rpt_vw(rpt);

  stapl::serial_io(get_triple_wf(zin), key_vw, fac_vw, rpt_vw );

  // generate the segment flags
  ary_int_tp seg_flag(limit);
  ary_int_vw_tp seg_flag_vw(seg_flag);

  int first = 0;
  int prev = key_vw[first];
  int uniq_count = 0;
  for ( int i=1; i < limit; i++ ) {
    if ( prev != key_vw[i] ) {
      prev = key_vw[i];
      seg_flag_vw[i-1] = 1;
      uniq_count++;
    }
  }
  seg_flag_vw[limit-1] = 1;
  uniq_count++;

  // generate the segment counts
  ary_int_tp seg_cnt(uniq_count);
  ary_int_vw_tp seg_cnt_vw(seg_cnt);
  int seg_count = 1;
  size_t j = 0;
  for ( int i=0; i<limit; i++ ) {
    if ( seg_flag_vw[i] == 1 ) {
      seg_cnt_vw[j++] = seg_count;
      seg_count = 1;
    } else {
      seg_count++;
    }
  }

  // subtract 1 from the size because splitter_partition adds an element
  std::vector<size_t> stdq(seg_cnt_vw.size()-1);
  for ( size_t i= 0; i<seg_cnt_vw.size()-1; i++ ) {
    stdq[i] = seg_cnt[i];
  }

  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());
  seg_ary_vw_tp seg_fac_vw( fac_vw,
                 seg_ary_splitter(fac_vw.domain(), stdq,true));
  seg_ary_vw_tp seg_rpt_vw( rpt_vw,
                 seg_ary_splitter(rpt_vw.domain(), stdq,true));

  ndx_dom_tp map_dom(1, 65536);

  ary_map_int_tp mv(seg_fac_vw.size());
  ary_map_int_vw_tp mv_vw(mv);

  ary_int_tp mf(seg_fac_vw.size());
  ary_int_vw_tp mf_vw(mf);

  // copy from the segment view of the source to the nested map destination

  stapl::map_func(nestpar_212_build_wf(), seg_fac_vw, seg_rpt_vw, mv_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_212_process_wf(), mv_vw, mf_vw );

  size_t res = stapl::map_reduce(l1_seq_l0_map_cksum_wf(), xor_un_wf(), mv_vw);

  // output for validation

  stapl::serial_io(show_indexable_map_wf(zout), mv_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP212","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(ary(int))
// construct vector with serial I/O,
// process with mapreduce / work function, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_213_fill_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_213_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_213_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_213_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_213_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_213( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  vec_ary_int_tp a(len_vw), b(len_vw);
  vec_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_213_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_213_process_wf(), a_vw );

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_213_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP213","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(vec(int))
// construct array with serial I/O,
// process with mapreduce / work function, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_214_fill_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_214_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_214_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_214_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_214_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_214( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  ary_vec_int_tp a(len_vw), b(len_vw);
  ary_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_214_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_214_process_wf(), a_vw );

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_214_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP214","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(ary(int))
// construct map with serial I/O,
// process with mapreduce / work function, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_215_build_wf
{
  typedef void result_type;
  template <typename Outer, typename Key, typename SegRef>
  result_type operator()(Outer map, Key key, SegRef seg)
  {
    map[key].resize(seg.size());
    int j = 0;
    typename SegRef::iterator iter;
    for ( iter = seg.begin(); iter != seg.end(); ++iter ) {
      int val = *iter;
      map[key][j] = val;
      j++;
    }
  }
};

struct nestpar_215_process_wf
{
  typedef int result_type;
  template <typename Elem1, typename Elem2>
  result_type operator()(Elem1 elem1, Elem2 elem2)
  {
    typename Elem1::second_reference vec = elem1.second;
    typename Elem2::second_reference val = elem2.second;
    val = vec.size();
  }
};

size_t nestpar_215( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{
  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  int limit = count;
  ary_int_tp key(limit), fac(limit), bits(limit);
  ary_int_vw_tp key_vw(key), fac_vw(fac), bits_vw(bits);

  stapl::serial_io(get_triple_wf(zin), key_vw, fac_vw, bits_vw );

  // generate the segment flags
  ary_int_tp seg_flag(limit);
  ary_int_vw_tp seg_flag_vw(seg_flag);
  int prev = key[0];
  int uniq_count = 0;
  for ( int i=1; i < limit; i++ ) {
    if ( prev != key[i] ) {
      prev = key[i];
      seg_flag[i-1] = 1;
      uniq_count++;
    }
  }
  seg_flag[limit-1] = 1;
  uniq_count++;

  // generate the segment counts
  ary_int_tp uniq_key(uniq_count);
  ary_int_vw_tp uniq_key_vw(uniq_key);
  ary_int_tp seg_cnt(uniq_count);
  ary_int_vw_tp seg_cnt_vw(seg_cnt);
  int seg_count = 1;
  size_t j = 0;
  for ( int i=0; i<limit; i++ ) {
    if ( seg_flag[i] == 1 ) {
      uniq_key[j] = key[i];
      seg_cnt[j] = seg_count;
      seg_count = 1;
      j++;
    } else {
      seg_count++;
    }
  }

  // subtract 1 from the size because splitter_partition adds an element
  std::vector<size_t> stdq(seg_cnt_vw.size()-1);
  for ( size_t i= 0; i<seg_cnt_vw.size()-1; i++ ) {
    stdq[i] = seg_cnt[i];
  }
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());
  seg_ary_vw_tp seg_fac_vw( fac_vw,
                 seg_ary_splitter(fac_vw.domain(), stdq,true));

  ndx_dom_tp map_dom(0, 16277216);
  map_ary_int_tp mv(map_dom);
  map_ary_int_vw_tp mv_vw(mv);
  map_int_tp mf(map_dom);
  map_int_vw_tp mf_vw(mf);

  // copy from the segment view of the source to the nested map destination

  stapl::map_func(nestpar_215_build_wf(), stapl::make_repeat_view(mv_vw),
                  uniq_key_vw, seg_fac_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

#ifdef UNINIT_BUG
  stapl::map_func(nestpar_215_process_wf(), mv_vw, mf_vw );

  size_t res = stapl::map_reduce(l1_map_l0_seq_cksum_wf(), xor_un_wf(), mv_vw);
#else
  size_t res = 0;
#endif

  // output for validation

  stapl::serial_io(show_map_indexable_wf(zout), mv_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP215","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(ary(int))
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

struct nestpar_216_fill_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_216_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_216_process_wf
{
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func( user1_wf(), vw1, vw2 );
  }
};

struct nestpar_216_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_216_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_216( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  vec_ary_int_tp a(len_vw), b(len_vw);
  vec_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_216_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_216_process_wf(), a_vw, b_vw );

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_216_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP216","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(vec(int))
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

struct nestpar_217_fill_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_217_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_217_process_wf
{
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const &vw1, View2 const &vw2, View3 const &vw3)
  {
    stapl::map_func( user2_wf(), vw1, vw2, vw3 );
  }
};

struct nestpar_217_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_217_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_217( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  ary_vec_int_tp a(len_vw), b(len_vw), c(len_vw);
  ary_vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(nestpar_217_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_217_process_wf(), a_vw, b_vw, c_vw );

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_217_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP217","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(vec(int))
// construct map with serial I/O,
// process with map_func / work function, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_218_build_wf
{
  typedef void result_type;
  template <typename Outer, typename Key, typename SegRef>
  result_type operator()(Outer map, Key key, SegRef seg)
  {
    int j = 0;
    map[key].resize(seg.size());
    typename SegRef::iterator iter;
    for ( iter = seg.begin(); iter != seg.end(); ++iter, ++j ) {
      auto val = *iter;
      map[key][j] = val;
    }
  }
};

struct nestpar_218_process_wf
{
  typedef int result_type;
  template <typename Elem1, typename Elem2>
  result_type operator()(Elem1 elem1, Elem2 elem2)
  {
    typename Elem1::second_reference vec = elem1.second;
    typename Elem2::second_reference val = elem2.second;
    val = vec.size();
  }
};

size_t nestpar_218( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{
  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  int limit = count;
  vec_int_tp key(limit), fac(limit), bits(limit);
  vec_int_vw_tp key_vw(key), fac_vw(fac), bits_vw(bits);

  stapl::serial_io(get_triple_wf(zin), key_vw, fac_vw, bits_vw );

  // generate the segment flags
  vec_int_tp seg_flag(limit);
  vec_int_vw_tp seg_flag_vw(seg_flag);
  int prev = key[0];
  int uniq_count = 0;
  for ( int i=1; i < limit; i++ ) {
    if ( prev != key[i] ) {
      prev = key[i];
      seg_flag[i-1] = 1;
      uniq_count++;
    }
  }
  seg_flag[limit-1] = 1;
  uniq_count++;

  // generate the segment counts
  vec_int_tp uniq_key(uniq_count);
  vec_int_vw_tp uniq_key_vw(uniq_key);
  vec_int_tp seg_cnt(uniq_count);
  vec_int_vw_tp seg_cnt_vw(seg_cnt);
  int seg_count = 1;
  size_t j = 0;
  for ( int i=0; i<limit; i++ ) {
    if ( seg_flag[i] == 1 ) {
      uniq_key[j] = key[i];
      seg_cnt[j] = seg_count;
      seg_count = 1;
      j++;
    } else {
      seg_count++;
    }
  }

  // subtract 1 from the size because splitter_partition adds an element
  std::vector<size_t> stdq(seg_cnt_vw.size()-1);
  for ( size_t i= 0; i<seg_cnt_vw.size()-1; i++ ) {
    stdq[i] = seg_cnt[i];
  }
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());
  seg_vec_vw_tp seg_fac_vw( fac_vw,
                 seg_vec_splitter(fac_vw.domain(), stdq,true));

  ndx_dom_tp map_dom(0, 16277216);
  map_vec_int_tp mv(map_dom);
  map_vec_int_vw_tp mv_vw(mv);
  map_int_tp mf(map_dom);
  map_int_vw_tp mf_vw(mf);
  // copy from the segment view of the source to the nested map destination

  stapl::map_func(nestpar_218_build_wf(), stapl::make_repeat_view(mv_vw),
                  uniq_key_vw, seg_fac_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

#ifdef INIT_BUG
  stapl::map_func(nestpar_218_process_wf(), mv_vw, mf_vw );

  size_t res = stapl::map_reduce(l1_map_l0_seq_cksum_wf(), xor_un_wf(), mv_vw);
#else
  size_t res = 0;
#endif

  // output for validation

  stapl::serial_io(show_map_indexable_wf(zout), mv_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP218","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(ary(int))
// construct vector with serial I/O,
// process with partial_sum, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_219_fill_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_219_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_219_process_wf
{
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_219_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_219_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_219( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  vec_ary_int_tp a(len_vw), b(len_vw);
  vec_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_219_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_219_process_wf(), a_vw, b_vw );

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_219_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP219","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(vec(int))
// construct array with serial I/O,
// process with partial_sum, display with serial
//////////////////////////////////////////////////////////////////////

struct nestpar_220_fill_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_220_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_220_process_wf
{
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_220_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_220_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_220( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{

  size_t outer = 100 * model;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  ary_vec_int_tp a(len_vw), b(len_vw);
  ary_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_220_fill_wf(zin), a_vw );

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_220_process_wf(), a_vw, b_vw );

  size_t res = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_220_show_wf(zout), b_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP220","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return res;
}
