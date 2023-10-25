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

#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/graph.hpp>

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

bool open_zin(int model, int test, stapl::stream<ifstream>& zin, int &count)
{
  string path;
#if FILL_MAP
  switch ( model ) {
  case 1:
    switch( test ) {
    case 317: case 318: case 319:
    case 320: case 321:
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
    case 320: case 321:
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
    case 320: case 321:
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
    case 320: case 321:
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
    case 320: case 321:
      path = "data/huge_bits.zin";
      break;
    default:
      path = "data/huge_primes.zin";
      break;
    }
    break;
  }
#endif // FILL_MAP

#if FILL_GRAPH
  switch ( model ) {
  case 1:
    switch( test ) {
    case 322: case 323: case 324:
    case 325: case 326:
      path = "data/tiny_bits.zin";
      break;
    default:
      path = "data/tiny_primes.zin";
      break;
    }
    break;
  case 100:
    switch( test ) {
    case 322: case 323: case 324:
    case 325: case 326:
      path = "data/small_bits.zin";
      break;
    default:
      path = "data/small_primes.zin";
      break;
    }
    break;
  case 10000:
    switch( test ) {
    case 322: case 323: case 324:
    case 325: case 326:
      path = "data/medium_bits.zin";
      break;
    default:
      path = "data/medium_primes.zin";
      break;
    }
    break;
  case 1000000:
    switch( test ) {
    case 322: case 323: case 324:
    case 325: case 326:
      path = "data/big_bits.zin";
      break;
    default:
      path = "data/big_primes.zin";
      break;
    }
    break;
  case 100000000:
    switch( test ) {
    case 322: case 323: case 324:
    case 325: case 326:
      path = "data/huge_bits.zin";
      break;
    default:
      path = "data/huge_primes.zin";
      break;
    }
    break;
  }
#endif // FILL_GRAPH

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
size_t nestpar_vam( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_vma( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_mav( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_vmm( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_mvm( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
#endif // FILL_MAP

#ifdef FILL_GRAPH
size_t nestpar_vag( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_vga( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_gav( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_vgg( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t nestpar_gvg( size_t, int &,
                    stapl::stream<ifstream> &, stapl::stream<ofstream> & );
#endif // FILL_GRAPH

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

#ifdef FILL_MAP
    case 317:
      zout.open("np_vam.zout");
      if ( open_zin(model,317,zin,count) ) {
        result = nestpar_vam(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 318:
      zout.open("np_vma.zout");
      if ( open_zin(model,318,zin,count) ) {
        result = nestpar_vma(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 319:
      zout.open("np_mav.zout");
      if ( open_zin(model,319,zin,count) ) {
        result = nestpar_mav(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 320:
      zout.open("np_vmm.zout");
      if ( open_zin(model,320,zin,count) ) {
        result = nestpar_vmm(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 321:
      zout.open("np_mvm.zout");
      if ( open_zin(model,321,zin,count) ) {
        result = nestpar_mvm(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif // FILL_MAP

#ifdef FILL_GRAPH
    case 322:
      zout.open("np_vag.zout");
      if ( open_zin(model,322,zin,count) ) {
        result = nestpar_vag(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 323:
      zout.open("np_vga.zout");
      if ( open_zin(model,323,zin,count) ) {
        result = nestpar_vga(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 324:
      zout.open("np_gav.zout");
      if ( open_zin(model,324,zin,count) ) {
        result = nestpar_gav(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 325:
      zout.open("np_vgg.zout");
      if ( open_zin(model,325,zin,count) ) {
        result = nestpar_vgg(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
    case 326:
      zout.open("np_gvg.zout");
      if ( open_zin(model,326,zin,count) ) {
        result = nestpar_gvg(model, count, zin, zout);
        zin.close();
      } else {
        return EXIT_FAILURE;
      }
      break;
#endif // FILL_GRAPH

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
typedef stapl::array<vec_int_tp> ary_vec_int_tp;

#if FILL_MAP
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

typedef stapl::map<int,vec_map_int_tp> map_vec_map_int_tp;
typedef stapl::map_view<map_vec_map_int_tp> map_vec_map_int_vw_tp;

typedef stapl::map<int,vec_ary_int_tp> map_vec_ary_int_tp;
typedef stapl::map_view<map_vec_ary_int_tp> map_vec_ary_int_vw_tp;
#endif // FILL_MAP


#if FILL_GRAPH
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES,
                    int, stapl::properties::no_property>  graph_int_tp;
typedef stapl::graph_view<graph_int_tp> graph_int_vw_tp;
typedef stapl::array<graph_int_tp> ary_graph_int_tp;
typedef stapl::vector<ary_graph_int_tp> vec_ary_graph_int_tp;
typedef stapl::vector_view<vec_ary_graph_int_tp> vec_ary_graph_int_vw_tp;
typedef stapl::vector<graph_int_tp> vec_graph_int_tp;
typedef stapl::array<vec_graph_int_tp> ary_vec_graph_int_tp;
typedef stapl::array_view<vec_ary_graph_int_tp> ary_vec_graph_int_vw_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, ary_int_tp,
                            stapl::properties::no_property>  graph_ary_int_tp;
typedef stapl::vector<graph_ary_int_tp> vec_graph_ary_int_tp;
typedef stapl::vector_view<vec_graph_ary_int_tp> vec_graph_ary_int_vw_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, ary_vec_int_tp,
                        stapl::properties::no_property> graph_ary_vec_int_tp;
typedef stapl::graph_view<graph_ary_vec_int_tp> graph_ary_vec_int_vw_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, graph_int_tp,
                     stapl::properties::no_property>  graph_graph_int_tp;
typedef stapl::vector<graph_graph_int_tp> vec_graph_graph_int_tp;
typedef stapl::vector_view<vec_graph_graph_int_tp> vec_graph_graph_int_vw_tp;

typedef stapl::vector<graph_int_tp> vec_graph_int_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, vec_graph_int_tp,
                     stapl::properties::no_property>  graph_vec_graph_int_tp;
typedef stapl::graph_view<graph_vec_graph_int_tp> graph_vec_graph_int_vw_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES,vec_ary_int_tp,
                     stapl::properties::no_property> graph_vec_ary_int_tp;
typedef stapl::graph_view<graph_vec_ary_int_tp> graph_vec_ary_int_vw_tp;
#endif // FILL_GRAPH

#ifdef FILL_MAP
//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(ary(map(int
//////////////////////////////////////////////////////////////////////

struct nestpar_vam_fill_wf
{
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_vam_fill_wf(size_t o, size_t m, size_t i)
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
          int mid_key = j;
          for (size_t k = 0; k < inner; k++ ) {
            int in_key = prime_nums[k];
            int value = rand_nums[k] % (k+1);
            a[outer_ndx][mid_key][in_key] = value;
          }
        }
      }
    } else {
      for (size_t i = 0; i < outer; i++ ) {
        int outer_ndx = i;
        for (size_t j = 0; j < middle; j++ ) {
          int mid_key = j;
          for (size_t k = 0; k < inner; k++ ) {
            for (size_t m = 0; m < data_cnt; m++ ) {
              int ndx = data_cnt - m;
              int in_key = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[outer_ndx][mid_key][in_key] = value;
            }
          }
        }
      }
    }
  }
  void define_type(stapl::typer& t)
  {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};

struct nestpar_vam_inner_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_vam_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_vam_outer_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_vam_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_vam_inner_show_wf(m_zout), vw1);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_vam( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{
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

  stapl::do_once(nestpar_vam_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_vam_outer_show_wf(zout), b_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP_vam","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(map(ary(int
//////////////////////////////////////////////////////////////////////
struct nestpar_vma_fill_wf
{
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_vma_fill_wf(size_t o, size_t m, size_t i)
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

  void define_type(stapl::typer& t)
  {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};

struct nestpar_vma_inner_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_vma_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_vma_outer_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_vma_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_vma_inner_show_wf(m_zout), vw1);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_vma( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{
  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_map_ary_int_tp a(outer), b(outer);
  vec_map_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_vma_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_vma_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP_vma","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_seq_l1_map_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(ary(vec(int
//////////////////////////////////////////////////////////////////////
struct nestpar_mav_fill_wf
{
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_mav_fill_wf(size_t o, size_t m, size_t i)
    : inner(i), middle(m), outer(o)
  { }

  typedef void result_type;
  template <typename Data, typename Length>
  result_type operator()(Data &a, Length &len_vw)
  {
    if ( inner <= data_cnt ) {
      for (size_t i = 0; i < len_vw.size(); i++ ) {
        int out_key = prime_nums[i];
        a[out_key].resize(len_vw[i].size());
      }

      for (size_t i = 0; i < len_vw.size(); i++ ) {
        int out_key = prime_nums[i];
        for (size_t j = 0; j < len_vw[i].size(); j++ ) {
          a[out_key][j].resize(len_vw[i][j]);
        }
      }

      for (size_t i = 0; i < outer; i++ ) {
        int out_key = prime_nums[i];
        for (size_t j = 0; j < middle; j++ ) {
          int mid_key = j;
          for (size_t k = 0; k < inner; k++ ) {
            int inner_ndx = k;
            int value = rand_nums[k] % (k+1);
            a[out_key][mid_key][inner_ndx] = value;
          }
        }
      }
    } else {
      for (size_t i = 0; i < outer; i++ ) {
        int out_key = prime_nums[i];
        for (size_t j = 0; j < middle; j++ ) {
          int mid_key = j;
          for (size_t k = 0; k < inner; k++ ) {
            for (size_t m = 0; m < data_cnt; m++ ) {
              int ndx = data_cnt - m;
              int inner_ndx = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[out_key][mid_key][inner_ndx] = value;
            }
          }
        }
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};

struct nestpar_mav_inner_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_mav_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_mav_outer_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_mav_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_mav_inner_show_wf(m_zout), vw1.second);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_mav( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{
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

  stapl::do_once(nestpar_mav_fill_wf(outer,middle,inner), a_vw, len_2_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_mav_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP_mav","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(map(map(int
//////////////////////////////////////////////////////////////////////
struct nestpar_vmm_fill_wf
{
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_vmm_fill_wf(size_t o, size_t m, size_t i)
    : inner(i), middle(m), outer(o)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a)
  {
    if ( inner <= data_cnt && middle <= data_cnt ) {
      for (size_t i = 0; i < outer; i++ ) {
        int outer_ndx = i;
        for (size_t j = 0; j < middle; j++ ) {
          int middle_key = prime_nums[j];
          for (size_t k = 0; k < inner; k++ ) {
            int in_key = prime_nums[k];
            int value = rand_nums[k] % (k+1);
            a[outer_ndx][middle_key][in_key] = value;
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
              int in_key = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[outer_ndx][middle_key][in_key] = value;
            }
          }
        }
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};



struct nestpar_vmm_inner_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_vmm_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_vmm_outer_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_vmm_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_vmm_inner_show_wf(m_zout), vw1);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};



size_t nestpar_vmm( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{
  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_map_map_int_tp a(outer), b(outer);
  vec_map_map_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_vmm_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_vmm_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP_vmm","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_seq_l1_map_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : map(vec(map(int
//////////////////////////////////////////////////////////////////////
struct nestpar_mvm_fill_wf
{
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_mvm_fill_wf(size_t o, size_t m, size_t i)
    : inner(i), middle(m), outer(o)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a)
  {
    if ( outer <= data_cnt && inner <= data_cnt ) {
      for (size_t i = 0; i < outer; i++ ) {
        int out_key = prime_nums[i];
        for (size_t j = 0; j < middle; j++ ) {
          int mid_key = j;
          for (size_t k = 0; k < inner; k++ ) {
            int in_key = prime_nums[k];
            int value = rand_nums[k] % (k+1);
            a[out_key][mid_key][in_key] = value;
          }
        }
      }
    } else {
      for (size_t i = 0; i < outer; i++ ) {
        int out_key = prime_nums[data_cnt % i];
        for (size_t j = 0; j < middle; j++ ) {
          int mid_key = j;
          for (size_t k = 0; k < inner; k++ ) {
            for (size_t m = 0; m < data_cnt; m++ ) {
              int ndx = data_cnt - m;
              int in_key = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[out_key][mid_key][in_key] = value;
            }
          }
        }
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};

struct nestpar_mvm_inner_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_mvm_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_mvm_outer_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_mvm_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_mvm_inner_show_wf(m_zout), vw1.second);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};



size_t nestpar_mvm( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{
  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  map_vec_map_int_tp a, b;
  map_vec_map_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_mvm_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_mvm_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP_mvm","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_map_l1_seq_l0_map_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

#endif // FILL_MAP

#ifdef FILL_GRAPH
//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(ary(graph(int
//////////////////////////////////////////////////////////////////////
struct  nestpar_vag_fill_wf
{
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_vag_fill_wf(size_t o, size_t m, size_t i)
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
          int mid_key = j;
          for (size_t k = 0; k < inner; k++ ) {
            int in_key = prime_nums[k];
            int value = rand_nums[k] % (k+1);
            a[outer][middle][inner].property() = value;
          }
        }
      }
    } else {
      for (size_t i = 0; i < outer; i++ ) {
        int outer_ndx = i;
        for (size_t j = 0; j < middle; j++ ) {
          int mid_key = j;
          for (size_t k = 0; k < inner; k++ ) {
            for (size_t m = 0; m < data_cnt; m++ ) {
              int ndx = data_cnt - m;
              int in_key = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[outer][middle][inner].property() = value;
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

struct nestpar_vag_inner_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_vag_inner_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(put_graph_val_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct nestpar_vag_outer_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_vag_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_vag_inner_show_wf(m_zout), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_vag( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{
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

  vec_ary_graph_int_tp vag(len_1_vw), vag_2(len_2_vw);
  vec_ary_graph_int_vw_tp vag_vw(vag), vag_2_vw(vag_2);


  stapl::do_once(nestpar_vag_fill_wf(outer,middle,inner), vag_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_vag_outer_show_wf(zout), vag_2_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP_vag","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_seq_l1_seq_l0_graph_cksum_wf(),
                                   xor_un_wf(), vag_vw);
  stapl::rmi_fence();
  return cksum;
}


//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(graph(ary(int
//////////////////////////////////////////////////////////////////////
struct nestpar_vga_fill_wf
{
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_vga_fill_wf(size_t o, size_t m, size_t i)
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
          int middle_key = prime_nums[j];
          for (size_t k = 0; k < inner; k++ ) {
            int inner_ndx = k;
            int value = rand_nums[k] % (k+1);
            a[outer_ndx][middle_key].property()[inner_ndx] = value;
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
              a[outer_ndx][middle_key].property()[inner_ndx] = value;
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

struct nestpar_vga_inner_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_vga_inner_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(put_val_wf(m_zout), vw1.property());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct nestpar_vga_outer_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_vga_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_vga_inner_show_wf(m_zout), vw1);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_vga( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{
  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_graph_ary_int_tp a(outer), b(outer);
  vec_graph_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_vga_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_vga_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP_vga","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_seq_l1_graph_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : graph(ary(vec(int
//////////////////////////////////////////////////////////////////////
struct nestpar_gav_fill_wf
{
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_gav_fill_wf(size_t o, size_t m, size_t i)
    : inner(i), middle(m), outer(o)
  { }

  typedef void result_type;
  template <typename Data, typename Length>
  result_type operator()(Data &a, Length &len_vw)
  {
    if ( inner <= data_cnt ) {
      for (size_t i = 0; i < outer; i++ ) {
        int out_key = prime_nums[i];
        for (size_t j = 0; j < middle; j++ ) {
          int mid_key = j;
          for (size_t k = 0; k < inner; k++ ) {
            int inner_ndx = k;
            int value = rand_nums[k] % (k+1);
            a[out_key].property()[mid_key][inner_ndx] = value;
          }
        }
      }
    } else {
      for (size_t i = 0; i < outer; i++ ) {
        int out_key = prime_nums[i];
        for (size_t j = 0; j < middle; j++ ) {
          int mid_key = j;
          for (size_t k = 0; k < inner; k++ ) {
            for (size_t m = 0; m < data_cnt; m++ ) {
              int ndx = data_cnt - m;
              int inner_ndx = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[out_key].property()[mid_key][inner_ndx] = value;
            }
          }
        }
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};

struct nestpar_gav_inner_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_gav_inner_show_wf(stapl::stream<ofstream> const& zout)
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

struct nestpar_gav_outer_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_gav_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_gav_inner_show_wf(m_zout), vw1.property());
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_gav( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{
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

  graph_vec_ary_int_tp a, b;
  graph_vec_ary_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_gav_fill_wf(outer,middle,inner), a_vw, len_2_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_gav_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP_gav","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_graph_l1_seq_l0_seq_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : vec(graph(graph(int
//////////////////////////////////////////////////////////////////////
struct nestpar_vgg_fill_wf
{
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_vgg_fill_wf(size_t o, size_t m, size_t i)
    : inner(i), middle(m), outer(o)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a)
  {
    if ( inner <= data_cnt && middle <= data_cnt ) {
      for (size_t i = 0; i < outer; i++ ) {
        int out_ndx = i;
        for (size_t j = 0; j < middle; j++ ) {
          int mid_key = prime_nums[j];
          for (size_t k = 0; k < inner; k++ ) {
            int in_key = prime_nums[k];
            int value = rand_nums[k] % (k+1);
            a[out_ndx][mid_key].property()[in_key].property() = value;
          }
        }
      }
    } else {
      for (size_t i = 0; i < outer; i++ ) {
        int out_ndx = i;
        for (size_t j = 0; j < middle; j++ ) {
          int mid_key = prime_nums[data_cnt % j];
          for (size_t k = 0; k < inner; k++ ) {
            for (size_t m = 0; m < data_cnt; m++ ) {
              int ndx = data_cnt - m;
              int in_key = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[out_ndx][mid_key].property()[in_key].property()= value;
            }
          }
        }
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};

struct nestpar_vgg_inner_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_vgg_inner_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(put_graph_val_wf(m_zout), vw1.property());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct nestpar_vgg_outer_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_vgg_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_vgg_inner_show_wf(m_zout), vw1);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_vgg( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{
  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  vec_graph_graph_int_tp a(outer), b(outer);
  vec_graph_graph_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_vgg_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_vgg_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP_vgg","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_seq_l1_graph_l0_graph_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

//////////////////////////////////////////////////////////////////////
// nested parallelism : graph(vec(graph(int
//////////////////////////////////////////////////////////////////////
struct nestpar_gvg_fill_wf
{
private:
  size_t inner;
  size_t middle;
  size_t outer;
public:
  nestpar_gvg_fill_wf(size_t o, size_t m, size_t i)
    : inner(i), middle(m), outer(o)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a)
  {
    if ( outer <= data_cnt && inner <= data_cnt ) {
      for (size_t i = 0; i < outer; i++ ) {
        int out_key = prime_nums[i];
        for (size_t j = 0; j < middle; j++ ) {
          int mid_key = j;
          for (size_t k = 0; k < inner; k++ ) {
            int in_key = prime_nums[k];
            int value = rand_nums[k] % (k+1);
            a[out_key].property()[mid_key][in_key].property() = value;
          }
        }
      }
    } else {
      for (size_t i = 0; i < outer; i++ ) {
        int out_key = prime_nums[data_cnt % i];
        for (size_t j = 0; j < middle; j++ ) {
          int mid_key = j;
          for (size_t k = 0; k < inner; k++ ) {
            for (size_t m = 0; m < data_cnt; m++ ) {
              int ndx = data_cnt - m;
              int in_key = prime_nums[ndx];
              int value = rand_nums[ndx] % (k+1);
              a[out_key].property()[mid_key][in_key].property()= value;
            }
          }
        }
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(outer);
    t.member(middle);
    t.member(inner);
  }
};

struct nestpar_gvg_inner_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_gvg_inner_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(put_graph_val_wf(m_zout), vw1);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct nestpar_gvg_outer_show_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  nestpar_gvg_outer_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1)
  {
    stapl::serial_io(nestpar_gvg_inner_show_wf(m_zout), vw1.property());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

size_t nestpar_gvg( size_t model, int &count,
                    stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout )
{
  size_t outer = 100 * model;
  size_t middle = 10 * model;
  size_t inner = 10 * model;

  stapl::counter<stapl::default_timer> ctr;
  double time_p=0.0, time_i=0.0, time_o=0.0;
  ctr.start();

  graph_vec_graph_int_tp a, b;
  graph_vec_graph_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(nestpar_gvg_fill_wf(outer,middle,inner), a_vw );
  stapl::rmi_fence();

  stapl::serial_io(nestpar_gvg_outer_show_wf(zout), a_vw);

  stapl::rmi_fence();
  ctr.stop();
  time_p = ctr.value();
  show_time("NP_gvg","STAPL",time_p,time_i+time_o);

  size_t cksum = stapl::map_reduce(l2_graph_l1_seq_l0_graph_cksum_wf(),
                                   xor_un_wf(), a_vw);
  stapl::rmi_fence();
  return cksum;
}

#endif // FILL_GRAPH
