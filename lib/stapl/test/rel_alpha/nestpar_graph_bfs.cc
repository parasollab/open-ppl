/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.
// All rights reserved.
// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <map>
#include <cstdio>
#include <string>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include "../test_report.hpp"
#include <stapl/utility/tuple.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/set/set.hpp>
#include <test/algorithms/test_utils.h>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/skeletons/serial.hpp>
#include <test/containers/graph/test_util.h>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/generators/torus.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/algorithms/breadth_first_search.hpp>

/* NOTE: does not run on more than 1 processors due to the stapl_repeat_view
   bug */

using namespace std;
size_t TUNING = 2;

typedef void (* test_ptr) (int, int,
                            stapl::stream<ifstream>&, stapl::stream<ofstream>&);

void gggbfs (int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout);
void ggbfs (int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout);

typedef stapl::properties::bfs_property vprop_t;
typedef stapl::dynamic_graph<stapl::DIRECTED, stapl::MULTIEDGES,
                   vprop_t, stapl::properties::no_property>           bfs_gr_tp;
typedef stapl::dynamic_graph<stapl::DIRECTED, stapl::MULTIEDGES,
                 bfs_gr_tp, stapl::properties::no_property>        bfs_gr_gr_tp;
typedef stapl::dynamic_graph<stapl::DIRECTED, stapl::MULTIEDGES,
             bfs_gr_gr_tp, stapl::properties::no_property>      bfs_gr_gr_gr_tp;
typedef stapl::graph_view<bfs_gr_tp>                               bfs_gr_vw_tp;
typedef stapl::graph_view<bfs_gr_gr_tp>                         bfs_gr_gr_vw_tp;
typedef stapl::graph_view<bfs_gr_gr_gr_tp>                   bfs_gr_gr_gr_vw_tp;

// Array used to initialize sizes of inner graphs
typedef stapl::array<size_t>                                             arr_tp;
typedef stapl::array_view<arr_tp>                                     arr_vw_tp;
typedef stapl::array<arr_tp>                                         arr_arr_tp;
typedef stapl::array_view<arr_arr_tp>                             arr_arr_vw_tp;


////////////////////////////////////////////////////////////////////////////////
// Extracts bfs levels of graph vertices and assigns them to an arrayview.
////////////////////////////////////////////////////////////////////////////////
struct extract_level_wf
{
  typedef void result_type;
  template <typename V, typename E>
  void operator() (V v, E e)
  {
    e = v.property().level();
  }
};

////////////////////////////////////////////////////////////////////////////////
// Work function used to add the exec times for level_sync and kla_times and
// to perform a logical AND of passed/not_passed values of different executions.
// Called in map_reduce from test_gg_bfs, test_ggg_bfs, ggbfs and gggbfs.
////////////////////////////////////////////////////////////////////////////////
struct tuple_wf
{
  typedef stapl::tuple<bool, double, double> result_type;
  template<typename Tuple1, typename Tuple2>
  result_type operator()(Tuple1 x, Tuple2 y)
  {
    result_type xx = x;
    result_type yy = y;
    return stapl::make_tuple(stapl::get<0>(xx) && stapl::get<0>(yy),
      stapl::get<1>(xx)+stapl::get<1>(yy), stapl::get<2>(xx)+stapl::get<2>(yy));
  }
};

////////////////////////////////////////////////////////////////////////////////
// Invokes BFS on a graph view with level_sync and with kla. Compares results
// to verify accuracy and returns values to ggbfs or gggbfs.
////////////////////////////////////////////////////////////////////////////////
template <class GraphView>
stapl::tuple<bool, double, double> test_core_graph(GraphView vgraph)
{
  typedef GraphView graph_view_t;
  typedef typename GraphView::vertex_descriptor vd_type;
  typedef stapl::static_array<vd_type> array_t;
  typedef stapl::array_view<array_t> array_view_t;

  stapl::counter<stapl::default_timer> ctr;

  // level-sync
  ctr.start();
  size_t iter1 = stapl::breadth_first_search(vgraph, 0);
  double lsync_time = ctr.stop();
  array_t result1_array(vgraph.size());
  array_view_t result1(result1_array);
  stapl::map_func(extract_level_wf(), vgraph, result1);

  ctr.reset();

  // kla
  auto kla_exec_policy = stapl::sgl::execution_policy<GraphView>{
    stapl::sgl::kla_policy{TUNING}};

  ctr.start();
  size_t iter2 = stapl::breadth_first_search(kla_exec_policy, vgraph, 0);
  double kla_time = ctr.stop();
  array_t result2_array(vgraph.size());
  array_view_t result2(result2_array);
  stapl::map_func(extract_level_wf(), vgraph, result2);

  bool passed = stapl::equal(result1, result2);
  if (TUNING > 1)
    passed &= (iter2 < iter1);

  return stapl::make_tuple(passed, lsync_time, kla_time);
}

////////////////////////////////////////////////////////////////////////////////
// Populates the arrays with values used for the sizes of the inner most graphs.
// Called by ggbfs, gggbfs, and gen_sz_out.
////////////////////////////////////////////////////////////////////////////////
struct gen_sz_in
{
private:
  stapl::stream<ifstream> m_zin;

public:
  typedef void result_type;
  gen_sz_in(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  template <typename View1>
  void operator()(View1 length)
  {
    m_zin >> length;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

////////////////////////////////////////////////////////////////////////////////
// Calls gen_sz_in on each sizes array for the outer graphs in cases
// where the nesting level of containers is more than two. Called by gggbfs.
////////////////////////////////////////////////////////////////////////////////
struct gen_sz_out
{
private:
  stapl::stream<ifstream> m_zin;

public:
  gen_sz_out(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  void operator()(View1 length)
  {
    stapl::serial_io(gen_sz_in(m_zin), length);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

////////////////////////////////////////////////////////////////////////////////
// Creates a torus on each of the inner most graphs in the nested container.
// Called by fill_ggg_bfs and ggbfs.
////////////////////////////////////////////////////////////////////////////////
struct fill_gg_bfs
{
private:
  stapl::stream<ofstream> m_zout;

public:
  fill_gg_bfs(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Vertex>
  void operator()(Vertex v)
  {
    int nx = 0;
    int ny = 0;
    auto view2 = v.property();
    int view_size = view2.num_vertices();

    for (int i = floor(sqrt(view_size)); i >= 1; i--) {
      if (view_size % i == 0) {
        nx = i;
        break;
      }
    }
    ny = view_size / nx;

    m_zout << "gg(" << v.descriptor() << ") size: " <<
      v.property().num_vertices() << " torus(" << nx << ", " << ny << ") \n";
    stapl::generators::make_torus(view2, nx, ny, false);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

////////////////////////////////////////////////////////////////////////////////
// Calls fill_gg_bfs to generate Torus on inner most graphs.
// Called by gggbfs.
////////////////////////////////////////////////////////////////////////////////
struct fill_ggg_bfs
{
private:
  stapl::stream<ofstream> m_zout;

public:
  fill_ggg_bfs(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Vertex>
  void operator()(Vertex v)
  {
    m_zout << "ggg(" << v.descriptor() << ") size: "
      << v.property().num_vertices() << " \n";
    stapl::map_func(fill_gg_bfs(m_zout), v.property());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

////////////////////////////////////////////////////////////////////////////////
// Invokes the test_core_graph function on each vertex of a nested container.
// Called by test_ggg_bfs and ggbfs.
////////////////////////////////////////////////////////////////////////////////
struct test_gg_bfs
{
  typedef stapl::tuple<bool, double, double> result_type;
  template <typename Vertex>
  result_type operator()(Vertex v) const
  {
    return test_core_graph(v.property());
  }
};

////////////////////////////////////////////////////////////////////////////////
// Invokes the test_gg_bfs function on each vertex of the outer most graph.
// Called by gggbfs.
////////////////////////////////////////////////////////////////////////////////
struct test_ggg_bfs
{
  typedef stapl::tuple<bool, double, double> result_type;
  template <typename Vertex>
  result_type operator()(Vertex v) const
  {
    return stapl::map_reduce(test_gg_bfs(), tuple_wf(), v.property());
  }
};

////////////////////////////////////////////////////////////////////////////////
// Initializes size arrays, nested containers using those size arrays. Finally
// processes and outputs the test results for graph(graph) test case.
////////////////////////////////////////////////////////////////////////////////
void ggbfs (int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout)
{
  one_print("graph(graph(int))...\t\t");
  zout << "\ngraph(graph(int))...\n";

  arr_tp                                                       gg_sz(data_size);
  arr_vw_tp                                                     gg_sz_vw(gg_sz);
  stapl::serial_io(gen_sz_in(zin), gg_sz_vw);

  bfs_gr_gr_tp                                                     gg(gg_sz_vw);
  bfs_gr_gr_vw_tp                                                     gg_vw(gg);
  stapl::map_func(fill_gg_bfs(zout), gg_vw);


  stapl::tuple<bool, double, double> fin_res(true, 0.0, 0.0);

  for (int i = 0; i < num_executions; i++) {
    auto res = stapl::map_reduce(test_gg_bfs(), tuple_wf(), gg_vw);
    stapl::get<0>(fin_res) = stapl::get<0>(res) && stapl::get<0>(fin_res);
    stapl::get<1>(fin_res) += stapl::get<1>(res);
    stapl::get<2>(fin_res) += stapl::get<2>(res);
  }

  bool passed = stapl::get<0>(fin_res);
  double level_sync_time = stapl::get<1>(fin_res) / num_executions;
  double kla_time = stapl::get<2>(fin_res) / num_executions;
  one_print(passed);
  one_print("Level-sync exec time: ");
  one_print(std::to_string(level_sync_time).c_str());
  one_print("\nKLA exec time: ");
  one_print(std::to_string(kla_time).c_str());
  std::cerr << "\n";
}

////////////////////////////////////////////////////////////////////////////////
// Initializes size arrays, nested containers using those size arrays. Finally
// processes and outputs the test results for graph(graph(graph)) test case.
////////////////////////////////////////////////////////////////////////////////
void gggbfs (int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    stapl::stream<ofstream>& zout)
{
  one_print("\n\ngraph(graph(graph(int)))...\t\t");
  zout << "\ngraph(graph(graph(int))...\n";
  arr_tp                                                    init_arr(data_size);
  arr_vw_tp                                               init_arr_vw(init_arr);
  stapl::serial_io(gen_sz_in(zin), init_arr_vw);

  arr_arr_tp                                                ggg_sz(init_arr_vw);
  arr_arr_vw_tp                                               ggg_sz_vw(ggg_sz);
  stapl::map_func(gen_sz_out(zin), ggg_sz_vw);

  bfs_gr_gr_gr_tp                                                ggg(ggg_sz_vw);
  bfs_gr_gr_gr_vw_tp                                                ggg_vw(ggg);
  stapl::map_func(fill_ggg_bfs(zout), ggg_vw);

  stapl::tuple<bool, double, double> fin_res(true, 0.0, 0.0);

  for (int i = 0; i < num_executions; i++) {
    auto res = stapl::map_reduce(test_ggg_bfs(), tuple_wf(), ggg_vw);
    stapl::get<0>(fin_res) = stapl::get<0>(res) && stapl::get<0>(fin_res);
    stapl::get<1>(fin_res) = stapl::get<1>(res) + stapl::get<1>(fin_res);
    stapl::get<2>(fin_res) = stapl::get<2>(res) + stapl::get<2>(fin_res);
  }

  bool passed = stapl::get<0>(fin_res);
  double level_sync_time = stapl::get<1>(fin_res) / num_executions;
  double kla_time = stapl::get<2>(fin_res) / num_executions;
  one_print(passed);
  one_print("Level-sync exec time: ");
  one_print(std::to_string(level_sync_time).c_str());
  one_print("\nKLA exec time: ");
  one_print(std::to_string(kla_time).c_str());
  std::cerr << "\n";
}

////////////////////////////////////////////////////////////////////////////////
// Prints help options for the user. Function is invoked if user provides
// -h or --help as command line arguments.
////////////////////////////////////////////////////////////////////////////////
void show_help() {
  std::cerr << "Allowed options: \n";
  std::cerr << "\t-h [ --help ] \t\t Print this help message \n";
  std::cerr << "\t-d [ --data ] arg \t Data set \n";
  std::cerr << "\t-l [ --list ] \t\t Print tests provided \n";
  std::cerr << "\t-t [ --test ] arg \t Test to run \n";
  std::cerr << "\t-I [ --iterations ] \t arg Number of times test is repeated";
  std::cerr << " in timed section\n";
  std::cerr << "\n";
  std::cerr << "\nNOTE: With no options the program will run all tests in";
  std::cerr << "a medium data set 32 times.\n";
}


////////////////////////////////////////////////////////////////////////////////
// This struct can be used to specify the tests that we provide, and also the
// tests that we want to run. It does so by keeping a map of function pointers
// and function abbreviations. Each function abbreviation represents a name for
// a function that executes a test case provided here. If the user
// does not select any tests, the program runs all tests, in a medium data size,
// otherwise only runs the tests specified by the user. This is invoked by main.
//
// m_data is the length of the sizes array.
// m_num_executions how many iterations of the test to run
// m_filename what file should I use for input values.
// m_test_map a map of functions (which are test cases) and their abbreviations.
// m_zin stream object to read input from file
// m_zout stream object to write to file
////////////////////////////////////////////////////////////////////////////////
struct test_executor
{
   int m_data;
   int m_num_executions;
   std::string m_filename;
   std::map<std::string, test_ptr> m_test_map;
   stapl::stream<std::ifstream> m_zin;
   stapl::stream<std::ofstream> m_zout;

protected:
  void all()
  {
    for (typename std::map<std::string, test_ptr>::iterator
        it = m_test_map.begin(); it != m_test_map.end(); ++it) {
        ((test_ptr) (it->second))(m_data, m_num_executions, m_zin, m_zout);
    }
  }

public:
  test_executor(int data_size, int num_exec, std::string filename)
    : m_data(data_size), m_num_executions(num_exec), m_filename(filename)
  {
    m_test_map["ggbfs"] = &ggbfs;
    m_test_map["gggbfs"] = &gggbfs;

    m_zin.open(m_filename.c_str());
    m_zout.open("npgbfsoutput.txt");
    m_zout << "\t >> Breadth First Search on Nested STAPL pGraphs <<\n";
  }

  typedef void result_type;
  template <typename View>
  void operator()(const View& m_selected_tests)
  {
    if (m_selected_tests.size() != 0) {
      for (typename stapl::vector<std::string>::iterator
        it = m_selected_tests.begin(); it != m_selected_tests.end(); ++it) {
        if (m_test_map[(*it)] != NULL) ((test_ptr) m_test_map[(*it)])
                                     (m_data, m_num_executions, m_zin, m_zout);
        else
          std::cerr << "Invalid test name: " << (*it) << ".\n";
      }
    }
    else
      all();

    m_zin.close();
    m_zout.close();
  }

  void define_type(stapl::typer& t) {
    t.member(m_data);
    t.member(m_num_executions);
    t.member(m_filename);
    t.member(m_test_map);
    t.member(m_zin);
    t.member(m_zout);
  }
};


////////////////////////////////////////////////////////////////////////////////
// Main pre-selects values for data_size, number of iterations of a test, and
// the file where to read input from; in case the user does not specify any of
// those command line arguments.
// Then reads input to see whether the user has modified default arguments.
// If the user has specified something for any of the program options described
// in the show_help function than those pre-selected values are updated.
// It also does input validation, and finally invokes the test_executor struct
// which does the rest of the work in the program.
////////////////////////////////////////////////////////////////////////////////
stapl::exit_code stapl_main(int argc, char** argv)
{
  int data_size = 10000;
  int num_iterations = 32;
  stapl::vector<std::string> selected_tests;
  std::string input_filename = "data/medium_factors.zin";

  if (argc > 1) {
    for (int i = 0; i < argc; i++) {
      if (strcmp("-h", argv[i]) == 0 || strcmp("--help", argv[i]) == 0) {
        show_help();
        exit(1); // exit
      }

      if (strcmp("-d", argv[i]) == 0 || strcmp("--data", argv[i]) == 0) {
        if (i+1 <= argc-1) {
          i++;
          char * opt = argv[i];
          switch ( opt[0] ) {
            case 't':
              data_size = 1;
              input_filename = "data/tiny_factors.zin";
              break;
            case 's':
              data_size = 100;
              input_filename = "data/small_factors.zin";
              break;
            case 'm':
              data_size = 10000;
              input_filename = "data/medium_factors.zin";
              break;
          #ifdef BIG_HUGE_DATA
            case 'b':
              data_size = 10000000;
              input_filename = "data/big_factors.zin";
              break;
            case 'h':
              data_size = 100000000;
              input_filename = "data/huge_factors.zin";
              break;
          #endif
            default:
              std::cerr << "usage: exe --data tiny/small/medium/big/huge\n";
              exit(1); // exit
          }
        }
        else {
          break;
        }
      }

      if (strcmp("-i", argv[i]) == 0 || strcmp("--iterations", argv[i]) == 0) {
        if (i+1 <= argc-1) {
          i++;
          num_iterations = atoi(argv[i]);
          if (num_iterations < 1) {
            std::cerr << "Number of iterations cannot be less than 1!\n";
            exit(1);
          }
        }
      }

      if (strcmp("-l", argv[i]) == 0 || strcmp("--list", argv[i]) == 0) {
        std::cerr << "Tests Provided ~ Abbreviations \n";
        std::cerr << "1. graph(graph(int)) ~ ggbfs \n";
        std::cerr << "2. graph(graph(graph(int))) ~ gggbfs \n";
        std::cerr << "Usage ./exe -test abbrv\n";
        exit(1); // must exit program
      }

      if (strcmp("-t", argv[i]) == 0 || strcmp("--test", argv[i]) == 0) {
        if (i+1 <= argc-1) {
          i++;
          if (strcmp(argv[i], "ggbfs") == 0 ||
            strcmp(argv[i], "gggbfs") == 0) {
            selected_tests.push_back(argv[i]);
          }
          else {
            std::cerr << "\nInvalid test name provided!\n";
            exit(1);
          }

        }
        else
          break;
      }
    }
  }

  test_executor run(data_size, num_iterations, input_filename);
  stapl::vector_view<stapl::vector<std::string> > sel_test_view(selected_tests);
  run(sel_test_view);

  return EXIT_SUCCESS;
}
