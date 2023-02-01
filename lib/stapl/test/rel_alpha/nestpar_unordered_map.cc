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
#include <stapl/containers/unordered_map/unordered_map.hpp>
#include <stapl/containers/unordered_multimap/unordered_multimap.hpp>

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

#define FILE_INPUT 1
#define GEN_INPUT 1
#undef FILL_MAP

/*=========================================================================*/

typedef stapl::array<int>                                ary_int_tp;
typedef stapl::array_view<ary_int_tp>                    ary_int_vw_tp;

typedef stapl::map<int,int>                              map_int_tp;
typedef stapl::map_view<map_int_tp>                      map_int_vw_tp;

typedef stapl::indexed_domain<int>                       ndx_dom_tp;

/*=========================================================================*/

bool open_zin(int model, int test, stapl::stream<ifstream>& zin, int &count) {
  string path;
  switch ( model ) {
  case 1:
    switch( test ) {
    case 1:
      path = "data/tiny_bits.zin";
      break;
    default:
      path = "data/tiny_primes.zin";
      break;
    }
    break;
  case 100:
    switch( test ) {
    case 1:
      path = "data/small_bits.zin";
      break;
    default:
      path = "data/small_primes.zin";
      break;
    }
    break;
  case 10000:
    switch( test ) {
    case 1:
      path = "data/medium_bits.zin";
      break;
    default:
      path = "data/medium_primes.zin";
      break;
    }
    break;
  case 1000000:
    switch( test ) {
    case 1:
      path = "data/big_bits.zin";
      break;
    default:
      path = "data/big_primes.zin";
      break;
    }
    break;
  case 100000000:
    switch( test ) {
    case 1:
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
#ifdef FILL_MAP
size_t nestpar_319( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_320( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_321( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_322( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_323( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_324( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_325( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
#endif // FILL_MAP
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

  int first_test = 319;
  int last_test = 325;
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
#ifdef FILL_MAP
    case 319:
      zout.open("np319.zout");
      if ( open_zin(model,1,zin,count) ) {
        result = nestpar_319(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 320:
      zout.open("np320.zout");
      if ( open_zin(model,1,zin,count) ) {
        result = nestpar_320(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 321:
      zout.open("np321.zout");
      if ( open_zin(model,1,zin,count) ) {
        result = nestpar_321(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 322:
      zout.open("np322.zout");
      if ( open_zin(model,1,zin,count) ) {
        result = nestpar_322(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 323:
      zout.open("np323.zout");
      if ( open_zin(model,1,zin,count) ) {
        result = nestpar_323(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 324:
      zout.open("np324.zout");
      if ( open_zin(model,1,zin,count) ) {
        result = nestpar_324(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 325:
      zout.open("np325.zout");
      if ( open_zin(model,1,zin,count) ) {
        result = nestpar_325(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
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

typedef stapl::unordered_map<int,int>            ump_int_tp;
typedef stapl::unordered_map<int,ump_int_tp>     ump_2_int_tp;
typedef stapl::unordered_map<int,ump_2_int_tp>   ump_3_int_tp;

typedef stapl::map_view<ump_int_tp>              ump_int_vw_tp;
typedef stapl::map_view<ump_2_int_tp>            ump_2_int_vw_tp;
typedef stapl::map_view<ump_3_int_tp>            ump_3_int_vw_tp;

typedef stapl::unordered_map<int,ary_int_tp>     ump_ary_int_tp;
typedef stapl::array<ump_int_tp>                 ary_ump_int_tp;
typedef stapl::unordered_map<int,map_int_tp>     ump_map_int_tp;
typedef stapl::map<int,ump_int_tp>               map_ump_int_tp;

typedef stapl::unordered_map<int,ump_ary_int_tp> ump_ump_ary_int_tp;
typedef stapl::unordered_map<int,ary_ump_int_tp> ump_ary_ump_int_tp;
typedef stapl::unordered_map<int,ump_map_int_tp> ump_ump_map_int_tp;
typedef stapl::unordered_map<int,map_ump_int_tp> ump_map_ump_int_tp;

typedef stapl::map_view<ump_ump_ary_int_tp>      ump_ump_ary_int_vw_tp;
typedef stapl::map_view<ump_ary_ump_int_tp>      ump_ary_ump_int_vw_tp;
typedef stapl::map_view<ump_ump_map_int_tp>      ump_ump_map_int_vw_tp;
typedef stapl::map_view<ump_map_ump_int_tp>      ump_map_ump_int_vw_tp;

typedef stapl::array<ump_2_int_tp>               ary_ump_ump_int_tp;
typedef stapl::map<int,ump_2_int_tp>             map_ump_ump_int_tp;

typedef stapl::array_view<ary_ump_ump_int_tp>    ary_ump_ump_int_vw_tp;
typedef stapl::map_view<map_ump_ump_int_tp>      map_ump_ump_int_vw_tp;

#ifdef FILL_MAP
/*=========================================================================
 * heterogeneous positional & non-positional nested structures
 =========================================================================*/

//////////////////////////////////////////////////////////////////////
// nested parallelism : ump(ump(ary(int
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
    stapl::serial_io(put_map_val_wf(m_zout), vw1);
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

  ump_ump_ary_int_tp a, b;
  ump_ump_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_319_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_319_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP319","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_map_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ary(ump(ump(int
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

  ary_ump_ump_int_tp a(outer), b(outer);
  ary_ump_ump_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_320_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_320_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP320","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_seq_l1_map_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ump(ary(ump(int
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

  ump_ary_ump_int_tp a, b;
  ump_ary_ump_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_321_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_321_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP321","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_seq_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ump(ump(ump(int
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

  ump_ump_ump_int_tp a, b;
  ump_ump_ump_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_322_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_322_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP322","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_map_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ump(ump(map(int
//////////////////////////////////////////////////////////////////////

struct nestpar_323_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_323_fill_wf(size_t o, size_t m, size_t i)
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
    t.member(inner);
  }
};

// - - - - - - - - - -

struct nestpar_323_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_323_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_323_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_323_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_323_inner_show_wf(m_zout), vw1.second);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_323( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ump_ump_map_int_tp a, b;
  ump_ump_map_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_323_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_323_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP323","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_map_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : ump(map(ump(int
//////////////////////////////////////////////////////////////////////

struct nestpar_324_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_324_fill_wf(size_t o, size_t m, size_t i)
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
    t.member(inner);
  }
};

// - - - - - - - - - -

struct nestpar_324_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_324_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_324_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_324_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_324_inner_show_wf(m_zout), vw1.second);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_324( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ump_map_ump_int_tp a, b;
  ump_map_ump_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_324_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_324_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP324","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_map_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(ump(ump(int
//////////////////////////////////////////////////////////////////////

struct nestpar_325_fill_wf {
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_325_fill_wf(size_t o, size_t m, size_t i)
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
    t.member(inner);
  }
};

// - - - - - - - - - -

struct nestpar_325_inner_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_325_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_325_outer_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_325_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_325_inner_show_wf(m_zout), vw1.second);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

// - - - - - - - - - -

size_t nestpar_325( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout ) {

  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  map_ump_ump_int_tp a, b;
  map_ump_ump_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_325_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_325_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP325","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_map_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

#endif // FILL_MAP
