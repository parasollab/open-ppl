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

#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include <stapl/utility/do_once.hpp>

#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include "json_parser.hpp"

using namespace std;
#include "testutil.hpp"
#include "rel_alpha_data.h"
#include "rel_alpha_util.hpp"

/*=========================================================================*/

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector< vec_int_tp > vec_vec_int_tp;

typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;
typedef stapl::vector_view<vec_vec_int_tp> vec_vec_int_vw_tp;

typedef stapl::array<int> ary_int_tp;
typedef stapl::array< ary_int_tp > ary_ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;
typedef stapl::array_view<ary_ary_int_tp> ary_ary_int_vw_tp;

typedef stapl::map<int,int> map_int_tp;
typedef stapl::map< int, stapl::map<int,int> > map_map_int_tp;
typedef stapl::map_view<map_int_tp> map_int_vw_tp;
typedef stapl::map_view<map_map_int_tp> map_map_int_vw_tp;

typedef stapl::array<size_t> ary_sz_tp;
typedef stapl::array_view<ary_sz_tp> ary_sz_vw_tp;

typedef stapl::indexed_domain<int> ndx_dom_tp;

/*=========================================================================*/

bool open_zin(int model, int test, stapl::stream<ifstream>& zin, int &count) {
  string path;
  switch ( model ) {
  case 1:
    switch( test ) {
    case 112:
    case 115:
    case 118:
      path= "data/tiny_factors.zin";
      break;
    default:
      path="data/tiny_primes.zin";
      break;
    }
    break;
  case 100:
    switch( test ) {
    case 112:
    case 115:
    case 118:
      path = "data/small_factors.zin";
      break;
    default:
      path = "data/small_primes.zin";
      break;
    }
    break;
  case 10000:
    switch( test ) {
    case 112:
    case 115:
    case 118:
      path = "data/medium_factors.zin";
      break;
    default:
      path = "data/medium_primes.zin";
      break;
    }
    break;
  case 1000000:
    switch( test ) {
    case 112:
    case 115:
    case 118:
      path = "data/big_factors.zin";
      break;
    default:
      path = "data/big_primes.zin";
      break;
    }
    break;
  case 100000000:
    switch( test ) {
    case 112:
    case 115:
    case 118:
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

size_t nestpar_101( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_102( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_103( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_104( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_105( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_106( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_107( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_108( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_109( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_110( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_111( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_112( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_113( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_114( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_115( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_116( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_117( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_118( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_119( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_120( size_t, int &,
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

  int first_test = 101; // regressions
  int last_test = 120;
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
      std::cout << "Nested Parallel Crawl " << test << endl;
    });
#endif
    set_random_seed();
    int result = 0;
    switch ( test) {
    case 101:
      zout.open("np101.zout");
      result = nestpar_101(model, zin, zout);
      break;
    case 102:
      zout.open("np102.zout");
      result = nestpar_102(model, zin, zout);
      break;
    case 103:
      zout.open("np103.zout");
      result = nestpar_103(model, zin, zout);
      break;
    case 104:
      zout.open("np104.zout");
      result = nestpar_104(model, zin, zout);
      break;
    case 105:
      zout.open("np105.zout");
      result = nestpar_105(model, zin, zout);
      break;
    case 106:
      zout.open("np106.zout");
      result = nestpar_106(model, zin, zout);
      break;
    case 107:
      zout.open("np107.zout");
      result = nestpar_107(model, zin, zout);
      break;
    case 108:
      zout.open("np108.zout");
      result = nestpar_108(model, zin, zout);
      break;
    case 109:
      zout.open("np109.zout");
      result = nestpar_109(model, zin, zout);
      break;
    case 110:
      zout.open("np110.zout");
      if ( open_zin(model,110,zin,count) ) {
        result = nestpar_110(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 111:
      zout.open("np111.zout");
      if ( open_zin(model,111,zin,count) ) {
        result = nestpar_111(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 112:
      zout.open("np112.zout");
      if ( open_zin(model,112,zin,count) ) {
        result = nestpar_112(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 113:
      zout.open("np113.zout");
      if ( open_zin(model,113,zin,count) ) {
        result = nestpar_113(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 114:
      zout.open("np114.zout");
      if ( open_zin(model,114,zin,count) ) {
        result = nestpar_114(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 115:
      zout.open("np115.zout");
      if ( open_zin(model,115,zin,count) ) {
        result = nestpar_115(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 116:
      zout.open("np116.zout");
      if ( open_zin(model,116,zin,count) ) {
        result = nestpar_116(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 117:
      zout.open("np117.zout");
      if ( open_zin(model,117,zin,count) ) {
        result = nestpar_117(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 118:
      zout.open("np118.zout");
      if ( open_zin(model,118,zin,count) ) {
        result = nestpar_118(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 119:
      zout.open("np119.zout");
      if ( open_zin(model,119,zin,count) ) {
        result = nestpar_119(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 120:
      zout.open("np120.zout");
      if ( open_zin(model,120,zin,count) ) {
        result = nestpar_120(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    }
    zout.close();
#ifdef DEBUG
    stapl::do_once([&](){
      std::cout << endl << "Nested Parallel Crawl: " << result << endl;
    });
#endif
  }
  return EXIT_SUCCESS;
}

/*=========================================================================
 * homogeneous nested structures
 *=========================================================================*/

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with generator, traverse with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_101_fill_wf {

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::iota( vw1, 0 );
  }
};

struct nestpar_101_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return vw1.size();
  }
};

struct nestpar_101_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_101_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_101( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  vec_vec_int_tp a(len_vw), b(len_vw);
  vec_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_101_fill_wf(), a_vw );


  stapl::map_func(nestpar_101_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_101_show_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP101","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with generator, traverse with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_102_fill_wf {
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

struct nestpar_102_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return vw1.size();
  }
};

struct nestpar_102_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_102_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_102( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  ary_ary_int_tp a(len_vw), b(len_vw);
  ary_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_102_fill_wf(), a_vw);


  stapl::map_func(nestpar_102_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_102_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP102","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with data from C arrays, traverse with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_103_fill_wf {
private:
  size_t outer_;
  size_t inner_;
public:
  nestpar_103_fill_wf(size_t o, size_t i)
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

struct nestpar_103_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    return elem.second.size();
  }
};

size_t nestpar_103( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  ndx_dom_tp map_dom(0, 100000);
  map_map_int_tp a(map_dom), b(map_dom);
  map_map_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_103_fill_wf(size, limit), a_vw);


  stapl::map_func(nestpar_103_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP103","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_104_fill_wf {
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

struct nestpar_104_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_104_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_104_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_104( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  vec_vec_int_tp a(len_vw), b(len_vw);
  vec_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_104_fill_wf(), a_vw );


  stapl::map_func(nestpar_104_process_wf(), a_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_104_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP104","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with generator, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_105_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
     stapl::generate( vw1, stapl::random_sequence());
  }
};

struct nestpar_105_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( id_int_wf(), add_int_wf(), vw1 );
  }
};

struct nestpar_105_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_105_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_105( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  ary_ary_int_tp a(len_vw), b(len_vw);
  ary_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_105_fill_wf(), a_vw );


  stapl::map_func(nestpar_105_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_105_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP105","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map data from C arrays, traverse with mapreduce
//////////////////////////////////////////////////////////////////////

struct nestpar_106_fill_wf {
private:
  size_t outer_;
  size_t inner_;
public:
  nestpar_106_fill_wf(size_t o, size_t i)
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

struct nestpar_106_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    return stapl::map_reduce( inner_map_elem_wf(), add_int_wf(), elem.second);
  }
};

size_t nestpar_106( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  ndx_dom_tp map_dom(0, 100000);
  map_map_int_tp a(map_dom), b(map_dom);
  map_map_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_106_fill_wf(size, limit), a_vw);

  stapl::map_func(nestpar_106_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP106","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with generator, process with stapl algorithm
// apply unary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_107_fill_wf {
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

struct nestpar_107_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const& vw2)
  {
    stapl::transform(vw1, vw2, neg_int_wf());
  }
};

struct nestpar_107_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_107_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_107( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  vec_vec_int_tp a(len_vw), b(len_vw);
  vec_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::map_func(nestpar_107_fill_wf(), a_vw );


  stapl::serial_io(nestpar_107_show_wf(zout), a_vw);

  stapl::map_func(nestpar_107_process_wf(), a_vw, b_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_107_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP107","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with generator, process with stapl algorithm
// apply binary function in data parallel manner
//////////////////////////////////////////////////////////////////////

struct nestpar_108_fill_wf {
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

struct nestpar_108_process_wf {
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const &vw1, View2 const& vw2, View3 const& vw3)
  {
    stapl::transform(vw1, vw2, vw3, add_int_wf());
  }
};

struct nestpar_108_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_108_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_108( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  ary_ary_int_tp a(len_vw), b(len_vw), c(len_vw);
  ary_ary_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::map_func(nestpar_108_fill_wf(), a_vw);
  stapl::map_func(nestpar_108_fill_wf(), b_vw);


  stapl::serial_io(nestpar_108_show_wf(zout), a_vw);
  stapl::serial_io(nestpar_108_show_wf(zout), b_vw);

  stapl::map_func(nestpar_108_process_wf(), a_vw, b_vw, c_vw);

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_108_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP108","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with data from C arrays,
// process with stapl algorithm
//////////////////////////////////////////////////////////////////////

struct nestpar_109_fill_wf {
private:
  size_t outer_;
  size_t inner_;
public:
  nestpar_109_fill_wf(size_t o, size_t i)
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

struct nestpar_109_process_wf {
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

size_t nestpar_109( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  ndx_dom_tp map_dom(0, 100000);
  map_map_int_tp a(map_dom), b(map_dom);
  map_map_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_109_fill_wf(size, limit), a_vw);

  stapl::map_func(nestpar_109_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP109","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with serial_io I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_110_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_110_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_110_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const& vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_110_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_110_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_110( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  vec_vec_int_tp a(len_vw), b(len_vw);
  vec_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_110_fill_wf(zin), a_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_110_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP110","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with serial_io I/O,
// process with mapfunc / work function, display with counting view
//////////////////////////////////////////////////////////////////////

struct nestpar_111_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_111_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_111_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func(stapl::assign<typename vec_int_vw_tp::value_type>(),
                  vw1, vw2);
  }
};

struct nestpar_111_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_111_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_111( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  ary_ary_int_tp a(len_vw), b(len_vw);
  ary_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_111_fill_wf(zin), a_vw );


  stapl::rmi_fence();
  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_111_process_wf(), a_vw, b_vw);

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_111_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP111","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with serial_io I/O, display map
//////////////////////////////////////////////////////////////////////

struct nestpar_112_fill_wf {
private:
  size_t sz_;
public:
  nestpar_112_fill_wf(size_t s )
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

size_t nestpar_112( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  int limit = count;
  ary_sz_tp lkey(limit), rkey(limit), val(limit);
  ary_sz_vw_tp lkey_vw(lkey), rkey_vw(rkey), val_vw(val);

  stapl::serial_io(get_triple_wf(zin), lkey_vw, rkey_vw, val_vw );
  stapl::rmi_fence();

  ndx_dom_tp map_dom(0, 16777216);
  map_map_int_tp a(map_dom), b(map_dom);
  map_map_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_112_fill_wf(limit), lkey_vw, rkey_vw, val_vw, a_vw);

  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP112","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with serial_io I/O,
// process with mapreduce / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_113_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_113_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_113_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_113_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_113_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_113( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  vec_vec_int_tp a(len_vw), b(len_vw);
  vec_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_113_fill_wf(zin), a_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_113_process_wf(), a_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_113_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP113","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with serial_io I/O,
// process with mapreduce / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_114_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_114_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_114_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::map_reduce( neg_int_wf(), add_int_wf(), vw1);
  }
};

struct nestpar_114_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_114_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_114( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  ary_ary_int_tp a(len_vw), b(len_vw);
  ary_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_114_fill_wf(zin), a_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_114_process_wf(), a_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_114_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP114","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(map(int))
// construct map with serial_io I/O,
// process with mapreduce / work function, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_115_fill_wf {
private:
  size_t sz_;
public:
  nestpar_115_fill_wf(size_t s )
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

struct nestpar_115_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    return stapl::map_reduce( inner_map_elem_wf(), add_int_wf(), elem.second);
  }
};

size_t nestpar_115( size_t model, int &count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  int limit = count;
  ary_sz_tp lkey(limit), rkey(limit), val(limit);
  ary_sz_vw_tp lkey_vw(lkey), rkey_vw(rkey), val_vw(val);

  stapl::serial_io(get_triple_wf(zin), lkey_vw, rkey_vw, val_vw  );

  ndx_dom_tp map_dom(0, 16777216);
  map_map_int_tp a(map_dom), b(map_dom);
  map_map_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_115_fill_wf(limit), lkey_vw, rkey_vw, val_vw, a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_115_process_wf(), a_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  ctr.stop();
  time_p = ctr.value();
  show_time("NP115","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

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

struct nestpar_116_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_116_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_116_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::map_func( user1_wf(), vw1, vw2 );
  }
};

struct nestpar_116_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_116_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_116( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  vec_vec_int_tp a(len_vw), b(len_vw);
  vec_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_116_fill_wf(zin), a_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_116_process_wf(), a_vw, b_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_116_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP116","STAPL",time_p,time_i+time_o);

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

struct nestpar_117_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_117_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_117_process_wf {
  typedef int result_type;
  template <typename View1, typename View2, typename View3>
  result_type operator()(View1 const& vw1, View2 const& vw2, View3 const& vw3)
  {
    stapl::map_func( user2_wf(), vw1, vw2, vw3 );
  }
};

struct nestpar_117_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_117_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_117( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  size_t total_elems = stapl::accumulate(len_vw, 0);
  ary_ary_int_tp a(len_vw), b(len_vw), c(len_vw);
  ary_ary_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::serial_io(nestpar_117_fill_wf(zin), a_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_117_process_wf(), a_vw, b_vw, c_vw );

  size_t cksum= stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_117_show_wf(zout), a_vw);

  ctr.stop();
  time_p = ctr.value();
  show_time("NP117","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

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

struct nestpar_118_fill_wf {
private:
  size_t sz_;
public:
  nestpar_118_fill_wf(size_t s )
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

struct nestpar_118_process_wf {
  typedef int result_type;
  template <typename Elem1, typename Elem2>
  result_type operator()(Elem1 elem1, Elem2 elem2)
  {
    stapl::map_func( user3_wf(), elem1.second, elem2.second );
  }
};

size_t nestpar_118( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  int limit = count;
  ary_sz_tp lkey(limit), rkey(limit), val(limit);
  ary_sz_vw_tp lkey_vw(lkey), rkey_vw(rkey), val_vw(val);

  stapl::serial_io(get_triple_wf(zin), lkey_vw, rkey_vw, val_vw  );

  ndx_dom_tp map_dom(0, 16777216);
  map_map_int_tp a(map_dom), b(map_dom);
  map_map_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_118_fill_wf(limit), lkey_vw, rkey_vw, val_vw,
                                            a_vw, b_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_118_process_wf(), a_vw, b_vw );

  size_t cksum = stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(show_outer_map_wf(zout), a_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP118","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(vec(int))
// construct vector with serial_io I/O,
// process with partial_sum, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_119_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_119_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_119_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_119_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_119_show_wf(stapl::stream<ofstream> const& zout)
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


size_t nestpar_119( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  vec_vec_int_tp a(len_vw), b(len_vw);
  vec_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_119_fill_wf(zin), a_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_119_process_wf(), a_vw, b_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_119_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP119","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ary(int))
// construct array with serial_io I/O,
// process with partial_sum, display with serial_io
//////////////////////////////////////////////////////////////////////

struct nestpar_120_fill_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  nestpar_120_fill_wf(stapl::stream<ifstream> const& zin)
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

struct nestpar_120_process_wf {
  typedef int result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const &vw2)
  {
    stapl::partial_sum(vw1, vw2, stapl::plus<int>(), false);
  }
};

struct nestpar_120_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_120_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_120( size_t model, int & count,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout) {

  size_t size = 100 * model;
  size_t limit = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_sz_tp len(size);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(limit));

  ary_ary_int_tp a(len_vw), b(len_vw);
  ary_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::serial_io(nestpar_120_fill_wf(zin), a_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_i = ctr.value();
  ctr.start();

  stapl::map_func(nestpar_120_process_wf(), a_vw, b_vw );

  size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), a_vw);

  stapl::serial_io(nestpar_120_show_wf(zout), b_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP120","STAPL",time_p,time_i+time_o);

  stapl::rmi_fence();
  return cksum;
}
