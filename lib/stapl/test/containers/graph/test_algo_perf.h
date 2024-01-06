/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_TEST_CONTAINERS_GRAPH_TEST_ALGO_PERF_H
#define STAPL_TEST_CONTAINERS_GRAPH_TEST_ALGO_PERF_H

using namespace stapl;

template<typename GraphView, typename AlgorithmWrapper>
void run_test(GraphView& graph, AlgorithmWrapper & a,
              int k, size_t niter)
{
  counter<default_timer> t;
  double time;
  std::vector<double> times;

  size_t num_iterations;
  std::stringstream ss;
  ss << "k= " << k << " > KLA " << a.name() << ": ";

  for (size_t i=0; i<niter; ++i) {
    std::stringstream ss2;
    a.initialize(graph);

    t.reset(); t.start();
    num_iterations = a.run(graph, k);
    time = t.stop();
    ss2 << "  iterations = " << num_iterations << ", time = " << time;
    times.push_back(time);

    if (get_location_id() == 0)
      std::cout << ss2.str() << std::endl;

    rmi_fence();
  }

  compute_stats(ss.str(), times);
}


template<typename GraphView>
GraphView generate_graph(std::string filename)
{
  typedef GraphView graph_view_t;
  typedef typename graph_view_t::view_container_type graph_type;

  if (stapl::get_location_id() == 0)
    printf("Reading graph from file: %s\n", filename.c_str());

  counter<default_timer> t;
  t.start();
  graph_view_t v = read_adj_list<graph_type>(filename);
  double read_time = t.stop();

  if (stapl::get_location_id() == 0) {
    std::cout << "   Num vertices: " << v.num_vertices() << "\n"
              << "   Num edges:    " << v.num_edges() << "\n"
              << "   Read Time:    " << read_time << "\n"
              << std::endl;
  }

  v.sort_edges();
  return v;
}

template<typename AlgorithmWrapper,
         typename GraphGeneratorWrapper>
void test_graph(
  std::string filename, size_t niter,
  size_t k_start, size_t k_end,
  int argc, char** argv,
  GraphGeneratorWrapper const& graph_gen = GraphGeneratorWrapper())
{
  typedef typename AlgorithmWrapper::graph_type graph_type;
  typedef graph_view<graph_type> graph_view_t;

  bool read_from_file = true;
  if (!strcmp("", filename.c_str()))
    read_from_file = false;

  graph_view_t g;
  if (read_from_file)
    g = generate_graph<graph_view_t>(filename);
  else
  {
    if (stapl::get_location_id() == 0)
      std::cout << " Generating graph." << std::endl;
    g = graph_gen.template generate<graph_view_t>();
  }


  AlgorithmWrapper a(g, argc, argv);

  size_t k = k_start;
  if (k == 0) {
    run_test(g, a, k, niter);
    k = 1;
  }

  while (k <= k_end) {
    run_test(g, a, k, niter);
    k *= 2;
  }
}

struct algorithm_type
{
  std::map<std::string, size_t> m_alg_str_to_int_map;

  algorithm_type(void)
  {
    m_alg_str_to_int_map["mssp"] = 6;
    m_alg_str_to_int_map["topsort"] = 7;
    m_alg_str_to_int_map["coloring"] = 8;
    m_alg_str_to_int_map["scc"] = 9;
  }

  size_t operator()(std::string algorithm_name)
  { return m_alg_str_to_int_map[algorithm_name]; }
};


#endif
