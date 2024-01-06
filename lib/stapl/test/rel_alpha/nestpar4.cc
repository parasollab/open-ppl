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

#include <stapl/views/set_view.hpp>
#include <stapl/containers/set/set.hpp>

#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>

#include <stapl/views/multiarray_view.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>

#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/partitions/blocked_partition.hpp>
#include <stapl/containers/partitions/block_cyclic_partition.hpp>

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
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

typedef stapl::array<int> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;

typedef stapl::map<int,int> map_int_tp;
typedef stapl::map_view<map_int_tp> map_int_vw_tp;

typedef stapl::set<int> set_int_tp;
typedef stapl::set_view<set_int_tp> set_int_vw_tp;

typedef stapl::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES, int, int>
        graf_int_tp;
typedef stapl::graph_view<graf_int_tp> graf_int_vw_tp;

typedef stapl::multiarray<2, int>             ary2_int_tp;
typedef stapl::multiarray_view<ary2_int_tp>   ary2_int_vw_tp;

typedef stapl::multiarray<3, int>             ary3_int_tp;
typedef stapl::multiarray_view<ary3_int_tp>   ary3_int_vw_tp;

typedef stapl::indexed_domain<int> ndx_dom_tp;

typedef stapl::splitter_partition<vec_int_vw_tp::domain_type> seg_vec_splitter;
typedef stapl::segmented_view<vec_int_vw_tp, seg_vec_splitter> seg_vec_vw_tp;

typedef stapl::splitter_partition<ary_int_vw_tp::domain_type> seg_ary_splitter;
typedef stapl::segmented_view<ary_int_vw_tp, seg_ary_splitter> seg_ary_vw_tp;

typedef stapl::array<size_t>                         ary_sz_tp;
typedef stapl::array_view<ary_sz_tp>                 ary_sz_vw_tp;

typedef stapl::array<stapl::tuple<size_t, size_t> >  ary_tup2_sz_tp;
typedef stapl::array_view<ary_tup2_sz_tp>            ary_tup2_sz_vw_tp;

typedef stapl::array<stapl::tuple<size_t, size_t, size_t> >  ary_tup3_sz_tp;
typedef stapl::array_view<ary_tup3_sz_tp>            ary_tup3_sz_vw_tp;

typedef stapl::multiarray<2, ary_int_tp>             ary2_ary_int_tp;
typedef stapl::multiarray_view<ary2_ary_int_tp>      ary2_ary_int_vw_tp;

typedef stapl::multiarray<2, vec_int_tp>             ary2_vec_int_tp;
typedef stapl::multiarray_view<ary2_vec_int_tp>      ary2_vec_int_vw_tp;

typedef stapl::multiarray<2, set_int_tp>             ary2_set_int_tp;
typedef stapl::multiarray_view<ary2_set_int_tp>      ary2_set_int_vw_tp;

typedef stapl::multiarray<2, map_int_tp>             ary2_map_int_tp;
typedef stapl::multiarray_view<ary2_map_int_tp>      ary2_map_int_vw_tp;

typedef stapl::multiarray<2, graf_int_tp>            ary2_graf_int_tp;
typedef stapl::multiarray_view<ary2_graf_int_tp>     ary2_graf_int_vw_tp;


typedef stapl::multiarray<3, ary_int_tp>             ary3_ary_int_tp;
typedef stapl::multiarray_view<ary3_ary_int_tp>      ary3_ary_int_vw_tp;

typedef stapl::multiarray<3, vec_int_tp>             ary3_vec_int_tp;
typedef stapl::multiarray_view<ary3_vec_int_tp>      ary3_vec_int_vw_tp;

typedef stapl::multiarray<3, set_int_tp>             ary3_set_int_tp;
typedef stapl::multiarray_view<ary3_set_int_tp>      ary3_set_int_vw_tp;

typedef stapl::multiarray<3, map_int_tp>             ary3_map_int_tp;
typedef stapl::multiarray_view<ary3_map_int_tp>      ary3_map_int_vw_tp;

typedef stapl::multiarray<3, graf_int_tp>            ary3_graf_int_tp;
typedef stapl::multiarray_view<ary3_graf_int_tp>     ary3_graf_int_vw_tp;


typedef stapl::array<ary2_int_tp>                    ary_ary2_int_tp;
typedef stapl::array_view<ary_ary2_int_tp>           ary_ary2_int_vw_tp;

typedef stapl::vector<ary2_int_tp>                   vec_ary2_int_tp;
typedef stapl::vector_view<vec_ary2_int_tp>          vec_ary2_int_vw_tp;

typedef stapl::map<int,ary2_int_tp>                  map_ary2_int_tp;
typedef stapl::map_view<map_ary2_int_tp>             map_ary2_int_vw_tp;

typedef stapl::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES,
               ary2_int_tp, int>                     graf_ary2_int_tp;
typedef stapl::graph_view<graf_ary2_int_tp>          graf_ary2_int_vw_tp;

typedef stapl::array<ary3_int_tp>                    ary_ary3_int_tp;
typedef stapl::array_view<ary_ary3_int_tp>           ary_ary3_int_vw_tp;

typedef stapl::vector<ary3_int_tp>                   vec_ary3_int_tp;
typedef stapl::vector_view<vec_ary3_int_tp>          vec_ary3_int_vw_tp;

typedef stapl::map<int, ary3_int_tp>                 map_ary3_int_tp;
typedef stapl::map_view<map_ary3_int_tp>             map_ary3_int_vw_tp;

typedef stapl::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES,
               ary3_int_tp, int>                     graf_ary3_int_tp;
typedef stapl::graph_view<graf_ary3_int_tp>          graf_ary3_int_vw_tp;

typedef stapl::array<stapl::tuple<size_t, size_t> >  ary2_sz_tp;
typedef stapl::array_view<ary2_sz_tp>                ary2_sz_vw_tp;

typedef stapl::array<stapl::tuple<size_t, size_t, size_t> >
                                                     ary3_sz_tp;
typedef stapl::array_view<ary3_sz_tp>                ary3_sz_vw_tp;

typedef stapl::tuple<size_t,size_t>                  gid2_tp;
typedef stapl::tuple<size_t,size_t,size_t>           gid3_tp;

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

bool open_zin(int model, int test, stapl::stream<ifstream>& zin)
{
  string path;
  switch ( model ) {
  case 1:
    switch( test ) {
    default:
      path = "data/tiny_primes.zin";
      break;
    }
    break;
  case 100:
    switch( test ) {
    default:
      path = "data/small_primes.zin";
      break;
    }
    break;
  case 10000:
    switch( test ) {
    default:
      path = "data/medium_primes.zin";
      break;
    }
    break;
  case 1000000:
    switch( test ) {
    default:
      path = "data/big_primes.zin";
      break;
    }
    break;
  case 100000000:
    switch( test ) {
    default:
      path = "data/huge_primes.zin";
      break;
    }
    break;
  }
  zin.open(path.c_str());
  if ( zin.is_open() ) {
    return true;
  } else {
    cerr << "Unable to open input file: " << path << endl;
    return false;
  }
}

/*=========================================================================*/

size_t nestpar_401( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_402( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_403( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_404( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_406( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_407( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_408( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_409( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_411( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_412( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_415( size_t,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_416( size_t,
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

  int first_test = 401;
  int last_test = 416;
  if (opt_test != -1 ) {
    first_test = opt_test;
    last_test = opt_test;
  }

  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;

  //--------------------------------------------------------------------------
  // map tests are pending a fix from Nathan
  //--------------------------------------------------------------------------

#define DEBUG 1
  for ( int test=first_test; test<=last_test; test++ ) {
#ifdef DEBUG
    stapl::do_once([&]() {
        std::cout << "Nested Parallel Walk " << test << endl;
    });
#endif
    set_random_seed();
    size_t result = 0;
    switch ( test) {
    case 401:
      zout.open("np401.zout");
      result = nestpar_401(model,zin,zout);
      break;
    case 402:
      zout.open("np402.zout");
      result = nestpar_402(model,zin,zout);
      break;
    case 403:
      zout.open("np403.zout");
      result = nestpar_403(model,zin,zout);
      break;
    case 404:
      zout.open("np404.zout");
      result = nestpar_404(model,zin,zout);
      break;

    case 406:
      zout.open("np406.zout");
      result = nestpar_406(model,zin,zout);
      break;
    case 407:
      zout.open("np407.zout");
      result = nestpar_407(model,zin,zout);
      break;
    case 408:
      zout.open("np408.zout");
      result = nestpar_408(model,zin,zout);
      break;
    case 409:
      zout.open("np409.zout");
      result = nestpar_409(model,zin,zout);
      break;

    case 411:
      zout.open("np411.zout");
      result = nestpar_411(model,zin,zout);
      break;
    case 412:
      zout.open("np412.zout");
      result = nestpar_412(model,zin,zout);
      break;

    case 415:
      zout.open("np415.zout");
      result = nestpar_415(model,zin,zout);
      break;
    case 416:
      zout.open("np416.zout");
      result = nestpar_416(model,zin,zout);
      break;
    }
    zout.close();
#ifdef DEBUG
    stapl::do_once([&]() {
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
// nested parallelism : multiarray(2,array)
//////////////////////////////////////////////////////////////////////

struct nestpar_401_fill_wf
{
  typedef void result_type;
  template <typename View1, typename Elem>
  result_type operator()(View1 const &vw1, Elem count)
  {
    stapl::iota( vw1, 0 );
  }
};

struct nestpar_401_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
#ifdef PROCESS_BUG
    return stapl::map_reduce(id_int_wf(), add_int_wf(), vw1);
#else
    return vw1.size();
#endif
  }
};

struct nestpar_401_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_401_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_401( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout )
{
  size_t rows = 1 + rand() % (100 * model);
  size_t cols = 1 + rand() % (10 * model);
  size_t outer = rows * cols;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary2_ary_int_tp a(stapl::make_tuple(rows,cols));
  ary2_ary_int_vw_tp a_vw(a);

  ary2_ary_int_tp::dimensions_type dims = a.dimensions();
  assert( rows == stapl::get<0>(dims) );
  assert( cols == stapl::get<1>(dims) );

  auto lin_nest_vw = stapl::linear_view(a_vw);
stapl::do_once([&]() {
   std::cout << "np401: " << rows << " " << cols << " " <<
                lin_nest_vw.size() << endl;
});

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  stapl::map_func( nestpar_401_fill_wf(), lin_nest_vw, len_vw );

  int res = stapl::map_reduce(nestpar_401_process_wf(), max_int_wf(),
                              lin_nest_vw );

  stapl::serial_io(nestpar_401_show_wf(zout), lin_nest_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP401","STAPL",time_p,time_i+time_o);

  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : multiarray(2,vector)
//////////////////////////////////////////////////////////////////////

struct nestpar_402_fill_wf
{
  typedef void result_type;
  template <typename View1, typename Elem>
  result_type operator()(View1 const &vw1, Elem count)
  {
    int base = 10;
    int step = 2;
    typedef stapl::sequence<int> step_wf;
    stapl::generate(vw1, step_wf(base,step));
  }
};

struct nestpar_402_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
#ifdef PROCESS_BUG
    return stapl::map_reduce(id_int_wf(), add_int_wf(), vw1);
#else
    return vw1.size();
#endif
  }
};

struct nestpar_402_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_402_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_402( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout )
{
  size_t rows = 1 + rand() % (100 * model);
  size_t cols = 1 + rand() % (10 * model);
  size_t outer = rows * cols;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary2_vec_int_tp a(stapl::make_tuple(rows,cols));
  ary2_vec_int_vw_tp a_vw(a);

  ary2_vec_int_tp::dimensions_type dims = a.dimensions();
  assert( rows == stapl::get<0>(dims) );
  assert( cols == stapl::get<1>(dims) );

  auto lin_nest_vw = stapl::linear_view(a_vw);
stapl::do_once([&]() {
   std::cout << "np402: " << rows << " " << cols << " " <<
                lin_nest_vw.size() << endl;
});

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  stapl::map_func( nestpar_402_fill_wf(), lin_nest_vw, len_vw );

  int res = stapl::map_reduce(nestpar_402_process_wf(), max_int_wf(),
                              lin_nest_vw );

  stapl::serial_io(nestpar_402_show_wf(zout), lin_nest_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP402","STAPL",time_p,time_i+time_o);

  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : multiarray(2,set)
//////////////////////////////////////////////////////////////////////

struct nestpar_403_fill_wf
{
  typedef void result_type;
  template <typename View1, typename Elem>
  result_type operator()(View1 const &vw1, Elem count)
  {
    if ( stapl::get_location_id() == 0 ) {
      for ( size_t i = 0; i < count; i++ ) {
        vw1.container().insert( prime_nums[rand_nums[i]] );
      }
    }
  }
};

struct nestpar_403_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return vw1.size();
  }
};

struct nestpar_403_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_403_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_403( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout )
{
  size_t rows = 1 + rand() % (100 * model);
  size_t cols = 1 + rand() % (10 * model);
  size_t outer = rows * cols;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary2_set_int_tp a(stapl::make_tuple(rows,cols));
  ary2_set_int_vw_tp a_vw(a);

  ary2_set_int_tp::dimensions_type dims = a.dimensions();
  assert( rows == stapl::get<0>(dims) );
  assert( cols == stapl::get<1>(dims) );

  auto lin_nest_vw = stapl::linear_view(a_vw);

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  stapl::map_func( nestpar_403_fill_wf(), lin_nest_vw, len_vw );

  int res = stapl::map_reduce(nestpar_403_process_wf(), max_int_wf(),
                              lin_nest_vw );

  stapl::serial_io(nestpar_403_show_wf(zout), lin_nest_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP403","STAPL",time_p,time_i+time_o);

  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : multiarray(2,map)
//////////////////////////////////////////////////////////////////////

struct nestpar_404_fill_wf
{
  typedef void result_type;
  template <typename View1, typename Elem>
  result_type operator()(View1 const &vw1, Elem count)
  {
    if ( stapl::get_location_id() == 0 ) {
      for ( size_t i = 0; i < count; i++ ) {
        int val = rand_nums[i];
        vw1.container()[ prime_nums[i] ] = countbits(val);
      }
    }
  }
};

struct nestpar_404_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
#ifdef PROCESS_BUG
    int val = stapl::map_reduce( inner_map_elem_wf(), add_int_wf(), vw1 );
    return val;
#else
    return vw1.size();
#endif
  }
};

struct nestpar_404_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_404_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_map_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_404( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout )
{
  size_t rows = 1 + rand() % (100 * model);
  size_t cols = 1 + rand() % (10 * model);
  size_t outer = rows * cols;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary2_map_int_tp a(stapl::make_tuple(rows,cols));
  ary2_map_int_vw_tp a_vw(a);

  ary2_map_int_tp::dimensions_type dims = a.dimensions();
  assert( rows == stapl::get<0>(dims) );
  assert( cols == stapl::get<1>(dims) );

  auto lin_nest_vw = stapl::linear_view(a_vw);

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  stapl::map_func( nestpar_404_fill_wf(), lin_nest_vw, len_vw );

  int res = stapl::map_reduce(nestpar_404_process_wf(), max_int_wf(),
                              lin_nest_vw );

  stapl::serial_io(nestpar_404_show_wf(zout), lin_nest_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP404","STAPL",time_p,time_i+time_o);

  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : multiarray(3,array)
//////////////////////////////////////////////////////////////////////

struct nestpar_406_fill_wf
{
  typedef void result_type;
  template <typename View1, typename Elem>
  result_type operator()(View1 const &vw1, Elem count)
  {
    stapl::iota( vw1, 0 );
  }
};

struct nestpar_406_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
#ifdef PROCESS_BUG
    return stapl::map_reduce(id_int_wf(), add_int_wf(), vw1);
#else
    return vw1.size();
#endif
  }
};

struct nestpar_406_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_406_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_406( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout )
{
  size_t pages = 1 + rand() % (10 * model);
  size_t rows = 1 + rand() % (10 * model);
  size_t cols = 1 + rand() % (10 * model);
  size_t outer = pages * rows * cols;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary3_ary_int_tp a(stapl::make_tuple(pages,rows,cols));
  ary3_ary_int_vw_tp a_vw(a);

  ary3_ary_int_tp::dimensions_type dims = a.dimensions();
  assert( pages == stapl::get<0>(dims) );
  assert( rows == stapl::get<1>(dims) );
  assert( cols == stapl::get<2>(dims) );

  auto lin_nest_vw = stapl::linear_view(a_vw);

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  stapl::map_func( nestpar_406_fill_wf(), lin_nest_vw, len_vw );

  int res = stapl::map_reduce(nestpar_406_process_wf(), max_int_wf(),
                              lin_nest_vw );

  stapl::serial_io(nestpar_406_show_wf(zout), lin_nest_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP406","STAPL",time_p,time_i+time_o);

  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism :  multiarray(3,vector)
//////////////////////////////////////////////////////////////////////

struct nestpar_407_fill_wf
{
  typedef void result_type;
  template <typename View1, typename Elem>
  result_type operator()(View1 const &vw1, Elem count)
  {
    int base = 10;
    int step = 2;
    typedef stapl::sequence<int> step_wf;
    stapl::generate(vw1, step_wf(base,step));
  }
};

struct nestpar_407_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
#ifdef PROCESS_BUG
    return stapl::map_reduce(id_int_wf(), add_int_wf(), vw1);
#else
    return vw1.size();
#endif
  }
};

struct nestpar_407_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_407_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_407( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout )
{
  size_t pages = 1 + rand() % (10 * model);
  size_t rows = 1 + rand() % (10 * model);
  size_t cols = 1 + rand() % (10 * model);
  size_t outer = pages * rows * cols;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary3_vec_int_tp a(stapl::make_tuple(pages,rows,cols));
  ary3_vec_int_vw_tp a_vw(a);

  ary3_vec_int_tp::dimensions_type dims = a.dimensions();
  assert( pages == stapl::get<0>(dims) );
  assert( rows == stapl::get<1>(dims) );
  assert( cols == stapl::get<2>(dims) );

  auto lin_nest_vw = stapl::linear_view(a_vw);

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  stapl::map_func( nestpar_407_fill_wf(), lin_nest_vw, len_vw );

  int res = stapl::map_reduce(nestpar_407_process_wf(), max_int_wf(),
                              lin_nest_vw );

  stapl::serial_io(nestpar_407_show_wf(zout), lin_nest_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP407","STAPL",time_p,time_i+time_o);

  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : multiarray(3,set)
//////////////////////////////////////////////////////////////////////

struct nestpar_408_fill_wf
{
  typedef void result_type;
  template <typename View1, typename Elem>
  result_type operator()(View1 const &vw1, Elem count)
  {
    if ( stapl::get_location_id() == 0 ) {
      for ( size_t i = 0; i < count; i++ ) {
        vw1.container().insert( prime_nums[rand_nums[i]] );
      }
    }
  }
};

struct nestpar_408_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    return vw1.size();
  }
};

struct nestpar_408_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_408_show_wf(stapl::stream<ofstream> const& zout)
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

size_t nestpar_408( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout )
{
  size_t pages = 1 + rand() % (10 * model);
  size_t rows = 1 + rand() % (10 * model);
  size_t cols = 1 + rand() % (10 * model);
  size_t outer = pages * rows * cols;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary3_set_int_tp a(stapl::make_tuple(pages,rows,cols));
  ary3_set_int_vw_tp a_vw(a);

  ary3_set_int_tp::dimensions_type dims = a.dimensions();
  assert( pages == stapl::get<0>(dims) );
  assert( rows == stapl::get<1>(dims) );
  assert( cols == stapl::get<2>(dims) );

  auto lin_nest_vw = stapl::linear_view(a_vw);

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  stapl::map_func( nestpar_408_fill_wf(), lin_nest_vw, len_vw );

  int res = stapl::map_reduce(nestpar_408_process_wf(), max_int_wf(),
                              lin_nest_vw );

  stapl::serial_io(nestpar_408_show_wf(zout), lin_nest_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP408","STAPL",time_p,time_i+time_o);

  return res;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : multiarray(3,map)
//////////////////////////////////////////////////////////////////////

struct nestpar_409_fill_wf
{
  typedef void result_type;
  template <typename View1, typename Elem>
  result_type operator()(View1 const &vw1, Elem count)
  {
    if ( stapl::get_location_id() == 0 ) {
      for ( size_t i = 0; i < count; i++ ) {
        int val = rand_nums[i];
        vw1.container()[ prime_nums[i] ] = countbits(val);
      }
    }
  }
};

struct nestpar_409_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
#ifdef PROCESS_BUG
    int val = stapl::map_reduce( inner_map_elem_wf(), add_int_wf(), vw1 );
    return val;
#else
    return vw1.size();
#endif
  }
};

struct nestpar_409_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_409_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    stapl::serial_io(put_map_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_409( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout )
{
  size_t pages = 1 + rand() % (10 * model);
  size_t rows = 1 + rand() % (10 * model);
  size_t cols = 1 + rand() % (10 * model);
  size_t outer = pages * rows * cols;
  size_t inner = 100 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary3_map_int_tp a(stapl::make_tuple(pages,rows,cols));
  ary3_map_int_vw_tp a_vw(a);

  ary3_map_int_tp::dimensions_type dims = a.dimensions();
  assert( pages == stapl::get<0>(dims) );
  assert( rows == stapl::get<1>(dims) );
  assert( cols == stapl::get<2>(dims) );

  auto lin_nest_vw = stapl::linear_view(a_vw);

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  stapl::map_func( nestpar_409_fill_wf(), lin_nest_vw, len_vw );

  int res = stapl::map_reduce(nestpar_409_process_wf(), max_int_wf(),
                              lin_nest_vw );

  stapl::serial_io(nestpar_409_show_wf(zout), lin_nest_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP409","STAPL",time_p,time_i+time_o);

  return 1;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : array(multiarray(2))
//////////////////////////////////////////////////////////////////////

struct nestpar_411_fill_wf
{
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    auto lin_vw = stapl::linear_view(vw1);
    stapl::iota( lin_vw, 0 );
  }
};

struct nestpar_411_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    auto lin_vw = stapl::linear_view(vw1);
    return stapl::map_reduce( id_int_wf(), add_int_wf(), lin_vw);
  }
};

struct nestpar_411_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_411_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    auto lin_vw = stapl::linear_view(vw1);
    stapl::serial_io(put_val_wf(m_zout), lin_vw);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_411( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout )
{
  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t rows = 1 + rand() % (100 * model);
  size_t cols = 1 + rand() % (10 * model);

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary_tup2_sz_tp len(outer);
  ary_tup2_sz_vw_tp len_vw(len);
  stapl::map_func( tuple2_rand_init_wf(), len_vw,
                   stapl::make_repeat_view(inner) );

  ary_ary2_int_tp a(len_vw);
  ary_ary2_int_vw_tp a_vw(a);

  stapl::map_func( nestpar_411_fill_wf(), a_vw );

  stapl::map_func(nestpar_411_process_wf(), a_vw );

  stapl::serial_io(nestpar_411_show_wf(zout), a_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP411","STAPL",time_p,time_i+time_o);

  return 1;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vector(multiarray(2))
//////////////////////////////////////////////////////////////////////

struct nestpar_412_fill_wf
{
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    auto lin_vw = stapl::linear_view(vw1);
    int base = 10;
    int step = 2;
    typedef stapl::sequence<int> step_wf;
    stapl::generate(lin_vw, step_wf(base,step));
  }
};

struct nestpar_412_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    auto lin_vw = stapl::linear_view(vw1);
    return stapl::map_reduce( id_int_wf(), add_int_wf(), lin_vw);
  }
};

struct nestpar_412_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_412_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    auto lin_vw = stapl::linear_view(vw1);
    stapl::serial_io(put_val_wf(m_zout), lin_vw);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_412( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout )
{
  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t rows = 1 + rand() % (100 * model);
  size_t cols = 1 + rand() % (10 * model);

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary2_sz_tp len(outer);
  ary2_sz_vw_tp len_vw(len);
  stapl::map_func( tuple2_rand_init_wf(), len_vw,
                   stapl::make_repeat_view(inner) );

  vec_ary2_int_tp a(len_vw);
  vec_ary2_int_vw_tp a_vw(a);

  stapl::map_func( nestpar_412_fill_wf(), a_vw );

  stapl::map_func(nestpar_412_process_wf(), a_vw );

  stapl::serial_io(nestpar_412_show_wf(zout), a_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP412","STAPL",time_p,time_i+time_o);

  return 1;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : array(multiarray(3))
//////////////////////////////////////////////////////////////////////

struct nestpar_415_fill_wf
{
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    auto lin_vw = stapl::linear_view(vw1);
    int base = 10;
    int step = 2;
    typedef stapl::sequence<int> step_wf;
    stapl::generate(lin_vw, step_wf(base,step));
  }
};

struct nestpar_415_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    auto lin_vw = stapl::linear_view(vw1);
    return stapl::map_reduce( id_int_wf(), add_int_wf(), lin_vw);
  }
};

struct nestpar_415_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_415_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    auto lin_vw = stapl::linear_view(vw1);
    stapl::serial_io(put_val_wf(m_zout), lin_vw);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_415( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout )
{
  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t pages = 1 + rand() % (10 * model);
  size_t rows = 1 + rand() % (10 * model);
  size_t cols = 1 + rand() % (10 * model);

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary3_sz_tp len(outer);
  ary3_sz_vw_tp len_vw(len);
  stapl::map_func( tuple3_rand_init_wf(), len_vw,
                   stapl::make_repeat_view(inner) );

  ary_ary3_int_tp a(len_vw);
  ary_ary3_int_vw_tp a_vw(a);

  stapl::map_func( nestpar_415_fill_wf(), a_vw );

  stapl::map_func(nestpar_415_process_wf(), a_vw );

  stapl::serial_io(nestpar_415_show_wf(zout), a_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP415","STAPL",time_p,time_i+time_o);

  return 1;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vector(multiarray(3))
//////////////////////////////////////////////////////////////////////

struct nestpar_416_fill_wf
{
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    auto lin_vw = stapl::linear_view(vw1);
    stapl::iota( lin_vw, 0 );
  }
};

struct nestpar_416_process_wf
{
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    auto lin_vw = stapl::linear_view(vw1);
    return stapl::map_reduce( id_int_wf(), add_int_wf(), lin_vw);
  }
};

struct nestpar_416_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_416_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    auto lin_vw = stapl::linear_view(vw1);
    stapl::serial_io(put_val_wf(m_zout), lin_vw);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_416( size_t model,
                    stapl::stream<ifstream> & zin,
                    stapl::stream<ofstream> & zout )
{
  size_t outer = 100 * model;
  size_t inner = 100 * model;
  size_t pages = 1 + rand() % (10 * model);
  size_t rows = 1 + rand() % (10 * model);
  size_t cols = 1 + rand() % (10 * model);

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  ary3_sz_tp len(outer);
  ary3_sz_vw_tp len_vw(len);
  stapl::map_func( tuple3_rand_init_wf(), len_vw,
                   stapl::make_repeat_view(inner) );

  vec_ary3_int_tp a(len_vw);
  vec_ary3_int_vw_tp a_vw(a);

  stapl::map_func( nestpar_416_fill_wf(), a_vw );

  stapl::map_func(nestpar_416_process_wf(), a_vw );

  stapl::serial_io(nestpar_416_show_wf(zout), a_vw );

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP416","STAPL",time_p,time_i+time_o);

  return 1;
}

