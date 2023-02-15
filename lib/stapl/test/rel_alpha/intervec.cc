/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>


#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>

#include <stapl/containers/unordered_map/unordered_map.hpp>
#include <stapl/containers/map/map.hpp>
#include <stapl/views/map_view.hpp>

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/views/multiarray_view.hpp>

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include "inter_nestpar_util.hpp"

#ifdef GFORGE_1354
  #include <stapl/containers/list/list.hpp>
  #include <stapl/views/list_view.hpp>
#endif

typedef void (* test_ptr) (size_t, int, stapl::stream<ifstream>&);
typedef std::vector<std::size_t> std_cont;

//////////////////////////////////////////////////////////////////////
// nested interoperability stapl<stl> : stapl::array<std::vector>
//////////////////////////////////////////////////////////////////////
void p_array(size_t n, int num_runs, stapl::stream<ifstream>& zin)
{
  rand_gen gen;
  n = gen.rand(1, n) * 100;

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_runs; i++)
  {
    typedef stapl::array<std_cont> p_arr_vec_tp;
    typedef stapl::array_view<p_arr_vec_tp> p_arr_vec_vw_tp;

    std_cont vec(gen.rand(1, n));
    p_arr_vec_tp p_arr_vec(gen.rand(1,n), vec);
    p_arr_vec_vw_tp p_arr_vec_vw(p_arr_vec);

    size_t xor_vals = stapl::map_reduce(pop_linear(zin), xor_un_wf(),
      p_arr_vec_vw);

    stapl::counter<stapl::default_timer>ctr;
    ctr.start();
    size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(),
      p_arr_vec_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (xor_vals == cksum) && stapl::get<0>(fin_res);
    stapl::rmi_fence();
  }
  print_results("p_array<std::vector>", fin_res, num_runs);
}

//////////////////////////////////////////////////////////////////////
// nested interoperability stapl<stl> : stapl::vector<std::vector>
//////////////////////////////////////////////////////////////////////
void p_vector(size_t n, int num_runs, stapl::stream<ifstream>& zin)
{
  rand_gen gen;
  n = gen.rand(1, n) * 100;

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_runs; i++)
  {
    typedef stapl::vector<std_cont> p_vec_vec_tp;
    typedef stapl::vector_view<p_vec_vec_tp> p_vec_vec_vw_tp;

    std_cont vec(gen.rand(1, n));
    p_vec_vec_tp p_vec_vec(gen.rand(1, n), vec);
    p_vec_vec_vw_tp p_vec_vec_vw(p_vec_vec);

    size_t xor_vals = stapl::map_reduce(pop_linear(zin), xor_un_wf(),
      p_vec_vec_vw);

    stapl::counter<stapl::default_timer>ctr;
    ctr.start();
    size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(),
      p_vec_vec_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (xor_vals == cksum) && stapl::get<0>(fin_res);
    stapl::rmi_fence();
  }
  print_results("p_vector<std::vector>", fin_res, num_runs);
}

#ifdef GFORGE_1354
////////////////////////////////////////////////////////////////////
//nested interoperability stapl<stl> : stapl::list<std::vector>
////////////////////////////////////////////////////////////////////
void p_list(size_t n, int num_runs, stapl::stream<ifstream>& zin)
{
  rand_gen gen;
  n = gen.rand(1, n) * 100;

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_runs; i++)
  {
    typedef stapl::list<std_cont> p_list_vec_tp;
    typedef stapl::list_view<p_list_vec_tp> p_list_vec_vw_tp;

    std_cont vec(gen.rand(1, n));
    p_list_vec_tp p_list_vec(gen.rand(1, n), vec);
    p_list_vec_vw_tp p_list_vec_vw(p_list_vec);

    size_t xor_vals = stapl::map_reduce(pop_linear(zin), xor_un_wf(),
      p_list_vec_vw);

    stapl::counter<stapl::default_timer>ctr;
    ctr.start();
    size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(),
      p_list_vec_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (xor_vals == cksum) && stapl::get<0>(fin_res);
    stapl::rmi_fence();
  }
  print_results("p_list<std::vector>", fin_res, num_runs);
}
#endif

//////////////////////////////////////////////////////////////////////
// nested interoperability stapl<stl> : stapl::map<std::vector>
//////////////////////////////////////////////////////////////////////
void p_map(size_t n, int num_runs, stapl::stream<ifstream>& zin)
{
  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_runs; i++)
  {
    //typedef stapl::indexed_domain<int> ndx_dom_tp;

    typedef stapl::map<int, std_cont> p_map_vec_tp;
    typedef stapl::map_view<p_map_vec_tp> p_map_vec_vw_tp;

    rand_gen gen;
    size_t new_n = gen.rand(1,n) * 100;

    std_cont vec(new_n);
    p_map_vec_tp p_map_vec;


    size_t this_loc = stapl::get_location_id();
    size_t locs = stapl::get_num_locations();

    for (auto i = this_loc; i < new_n; i += locs)
      p_map_vec.insert(i, vec);

    stapl::rmi_fence();

    p_map_vec_vw_tp p_map_vec_vw(p_map_vec);

    size_t xor_vals = stapl::map_reduce(pop_assoc(zin), xor_un_wf(),
      p_map_vec_vw);

    stapl::rmi_fence();

    stapl::counter<stapl::default_timer>ctr;
    ctr.start();
    size_t cksum = stapl::map_reduce(l1_map_l0_seq_cksum_wf(), xor_un_wf(),
      p_map_vec_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (xor_vals == cksum) && stapl::get<0>(fin_res);
    stapl::rmi_fence();
  }
  print_results("pMap<std::vector>", fin_res, num_runs);
}

//////////////////////////////////////////////////////////////////////
// nested interoperability stapl<stl> : stapl::unordered_map<std::vector>
//////////////////////////////////////////////////////////////////////
void p_unordered_map(size_t n, int num_runs, stapl::stream<ifstream>& zin)
{
  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_runs; i++)
  {
    typedef stapl::unordered_map<int, std_cont> p_umap_vec_tp;
    typedef stapl::map_view<p_umap_vec_tp> p_umap_vec_vw_tp;

    rand_gen gen;
    size_t new_n = gen.rand(1,n) * 100;

    std_cont vec(new_n);
    p_umap_vec_tp p_umap_vec;

    size_t this_loc = stapl::get_location_id();
    size_t locs = stapl::get_num_locations();

    for (auto i = this_loc; i < new_n; i += locs)
      p_umap_vec.insert(i, vec);

    stapl::rmi_fence();

    p_umap_vec_vw_tp p_umap_vec_vw(p_umap_vec);

    size_t xor_vals = stapl::map_reduce(pop_assoc(zin), xor_un_wf(),
      p_umap_vec_vw);

    stapl::counter<stapl::default_timer>ctr;
    ctr.start();
    size_t cksum = stapl::map_reduce(l1_map_l0_seq_cksum_wf(), xor_un_wf(),
      p_umap_vec_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (xor_vals == cksum) && stapl::get<0>(fin_res);
    stapl::rmi_fence();
  }
  print_results("p_unordered_map<std::vector>", fin_res, num_runs);
}

//////////////////////////////////////////////////////////////////////
// nested interoperability stapl<stl> : stapl::multiarray<std::vector>
//////////////////////////////////////////////////////////////////////
void p_multiarray(size_t n, int num_runs, stapl::stream<ifstream>& zin)
{
  stapl::tuple<bool, double> fin_res(true, 0.0);

  for (int i = 0; i < num_runs; i++)
  {
    typedef stapl::multiarray<2, std_cont> p_multiarr_vec_tp;
    typedef stapl::multiarray_view<p_multiarr_vec_tp> p_multiarr_vec_vw_tp;

    rand_gen gen;
    size_t rows = gen.rand(1, n) * 10;
    size_t cols = gen.rand(1, n) * 100;

    std_cont vec(gen.rand(1,n));

    p_multiarr_vec_tp p_multiarr_vec(stapl::make_tuple(rows,cols), vec);
    p_multiarr_vec_vw_tp p_multiarr_vec_vw(p_multiarr_vec);

    p_multiarr_vec_tp::dimensions_type dims = p_multiarr_vec.dimensions();

    assert(rows == stapl::get<0>(dims));
    assert(cols == stapl::get<1>(dims));

    size_t xor_vals = stapl::map_reduce(pop_linear(zin), xor_un_wf(),
      p_multiarr_vec_vw);

    stapl::counter<stapl::default_timer>ctr;
    ctr.start();
    size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(),
      p_multiarr_vec_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (xor_vals == cksum) && stapl::get<0>(fin_res);
    stapl::rmi_fence();
  }
  print_results("p_multiarray<std::vector>", fin_res, num_runs);
}

//////////////////////////////////////////////////////////////////////
// nested interoperability stapl<stl> : stapl::static_graph<std::vector>
//////////////////////////////////////////////////////////////////////
void p_static_graph(size_t n, int num_runs, stapl::stream<ifstream>& zin)
{
  rand_gen gen;
  size_t new_n = gen.rand(1,n) * 100;

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_runs; i++)
  {
    typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES,
    std_cont, stapl::properties::no_property>  p_static_gr_vec_tp;

    typedef stapl::graph_view<p_static_gr_vec_tp> p_static_gr_vec_vw_tp;

    std_cont vec(new_n);
    p_static_gr_vec_tp p_static_gr_vec(n, vec);
    p_static_gr_vec_vw_tp p_static_gr_vec_vw(p_static_gr_vec);

    size_t xor_vals = stapl::map_reduce(pop_graph(new_n, zin), xor_un_wf(),
       p_static_gr_vec_vw);

    stapl::counter<stapl::default_timer>ctr;
    ctr.start();
    size_t cksum = stapl::map_reduce(outer_graph_cont_cksum_wf(), xor_un_wf(),
      p_static_gr_vec_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (xor_vals == cksum) && stapl::get<0>(fin_res);
    stapl::rmi_fence();
  }
  print_results("p_static_graph<std::vector>", fin_res, num_runs);
}

//////////////////////////////////////////////////////////////////////
// nested interoperability stapl<stl> : stapl::dynamic_graph<std::vector>
//////////////////////////////////////////////////////////////////////
void p_dynamic_graph(size_t n, int num_runs, stapl::stream<ifstream>& zin)
{
  rand_gen gen;
  size_t new_n = gen.rand(1,n) * 100;
  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_runs; i++)
  {
    typedef stapl::dynamic_graph<stapl::DIRECTED, stapl::MULTIEDGES,
      std_cont, stapl::properties::no_property> p_dynamic_gr_vec_tp;

    typedef stapl::graph_view<p_dynamic_gr_vec_tp> p_dynamic_gr_vec_vw_tp;

    std_cont vec(new_n);
    p_dynamic_gr_vec_tp p_dynamic_gr_vec(n, vec);
    p_dynamic_gr_vec_vw_tp p_dynamic_gr_vec_vw(p_dynamic_gr_vec);

    size_t xor_vals = stapl::map_reduce(pop_graph(new_n, zin),  xor_un_wf(),
      p_dynamic_gr_vec_vw);

    stapl::counter<stapl::default_timer>ctr;
    ctr.start();
    size_t cksum = stapl::map_reduce(outer_graph_cont_cksum_wf(), xor_un_wf(),
      p_dynamic_gr_vec_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (xor_vals == cksum) && stapl::get<0>(fin_res);
    stapl::rmi_fence();
  }
  print_results("p_dynamic_graph<std::vector>", fin_res, num_runs);
}

///////////////////////////////////////////////////////////////////////////////
// This struct can be used to specify the tests that we provide, and also the
// tests that we want to run. It does so by keeping a map of function pointers
// and function abbreviations. Each function abbreviation represents a name for
// a function that executes a test case provided here. If the user
// does not select any tests, the program runs all tests, in a medium data
// size, otherwise only runs the tests specified by the user. This is invoked
// by stapl_main
// m_data is the length of the sizes array.
// m_num_runs how many iterations of the test to run
// m_filename what file should I use for input values.
// m_test_map a map of functions (which are test cases) and their
// abbreviations.
// m_zin stream object to read input from fill
///////////////////////////////////////////////////////////////////////////////
struct test_executor
{
   size_t m_data;
   int m_num_runs;
   std::string m_filename;
   std::map<std::string, test_ptr> m_test_map;
   stapl::stream<std::ifstream> m_zin;
   rand_gen gen;

protected:
  void all()
  {
    for (auto it = m_test_map.begin(); it != m_test_map.end(); ++it) {
        ((test_ptr) (it->second))(gen.rand(1,m_data), m_num_runs, m_zin);
    }
  }

public:
  test_executor(size_t data_size, int num_exec, std::string filename)
    : m_data(data_size), m_num_runs(num_exec), m_filename(filename)
  {
    m_test_map["p_array"] = &p_array;
    m_test_map["p_vector"] = &p_vector;

    m_test_map["p_map"] = &p_map;
    m_test_map["p_unordered_map"] = &p_unordered_map;
    m_test_map["p_multiarray"] = &p_multiarray;
    m_test_map["p_static_graph"] = &p_static_graph;
    m_test_map["p_dynamic_graph"] = &p_dynamic_graph;

#ifdef GFORGE_1354
    m_test_map["p_list"] = &p_list;
#endif

    m_zin.open(m_filename.c_str());
  }

  typedef void result_type;
  template <typename View>
  result_type operator()(const View& tests)
  {
    if (tests.size() != 0) {
      for (auto it = tests.begin(); it != tests.end(); ++it) {
        if (m_test_map[(*it)] != NULL) {
          ((test_ptr) m_test_map[(*it)])(m_data, m_num_runs, m_zin);
        }
        else
          std::cerr << "Invalid test name: " << (*it) << ".\n";
      }
    }
    else
      all();

    m_zin.close();
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_data);
    t.member(m_num_runs);
    t.member(m_filename);
    t.member(m_test_map);
    t.member(m_zin);
  }
};



///////////////////////////////////////////////////////////////////////////////
// Main pre-selects values for data_size, number of iterations of a test, and
// the file where to read input from; in case the user does not specify any of
// those command line arguments.
// Then reads input to see whether the user has modified default arguments.
// If the user has specified something for any of the program options described
// in the show_help function than those pre-selected values are updated.
// It also does input validation, and finally invokes the test_executor struct
// which does the rest of the work in the program.
///////////////////////////////////////////////////////////////////////////////
stapl::exit_code stapl_main(int argc, char** argv)
{
  size_t data_size = 1000;
  int iterations = 32;
  std::vector<std::string> tests_to_run;
  std::string inputfname = "data/tiny_bits.zin";
  std::set<std::string> test_set;

  test_set.insert("p_array");
  test_set.insert("p_vector");

  test_set.insert("p_map");
  test_set.insert("p_unordered_map");
  test_set.insert("p_multiarray");
  test_set.insert("p_static_graph");
  test_set.insert("p_dynamic_graph");

#ifdef GFORGE_1354
  test_set.insert("p_list");
#endif

  for (int i = 0; i < argc; i++)
  {
    if (strcmp("-h", argv[i]) == 0 || strcmp("--help", argv[i]) == 0) {
      show_help();
      exit(1); // exit
    }

    if (strcmp("-d", argv[i]) == 0 || strcmp("--data", argv[i]) == 0) {

      if (i+1 <= argc-1){
        i++;
        set_filendata(data_size, argv[i], inputfname);
      }
      else
        break;
    }

    if ((strcmp("-i", argv[i]) == 0 || strcmp("--iterations", argv[i]) == 0)
      && (i+1 <= argc-1))
    {
      iterations = atoi(argv[++i]);
      if (iterations < 1) {
        std::cerr << "Number of iterations less than 1?\n";
        exit(1);
      }
    }

    if (strcmp("-l", argv[i]) == 0 || strcmp("--list", argv[i]) == 0)
      print_tests(test_set);

    if ((strcmp("-t", argv[i]) == 0 || strcmp("--test", argv[i]) == 0))
    {
      if(i+1 <= argc-1)
      {
        i++;
        select_tests(tests_to_run, test_set, argv[i]);
      }
      else {
        break;
      }
    }
  }


  test_executor run(data_size, iterations, inputfname);
  run(tests_to_run);
  return EXIT_SUCCESS;
}
