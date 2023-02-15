//////////////////////////////////////////////////////////////////////
/// @file
/// @brief Tests the BFS parallel algorithm's performance
/// as part of Lonestar  benchmarking.
//////////////////////////////////////////////////////////////////////

//STAPL includes:
#include <stapl/containers/graph/digraph.hpp>
#include <stapl/containers/graph/algorithms/breadth_first_search.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include "../utility/test_util.h"

template <class G>
bool check_bfs(G& g, size_t source)
{
  bool passed = true;
  if (g[source].property().level() -1 != 0)
    passed = false;

  for (auto const& e : g[source]) {
    if (g[e.target()].property().level() -1 > 1)
      passed = false;
    for (auto const& f : g[e.target()])
      if (g[f.target()].property().level() -1 > 2)
        passed = false;
  }

  return passed;
}

template <class G>
void test_core_graph(G& g, size_t k, size_t source, size_t target)
{
  auto policy = stapl::sgl::execution_policy<G>{
    stapl::sgl::kla_policy{k}};

  stapl::counter<stapl::default_timer> t;
  t.start();
  stapl::breadth_first_search(policy, g, source);
  double time = t.stop();

  size_t bfs_level = (*g.find_vertex(target)).property().level();

  if (stapl::get_location_id() == 0) {
    bool passed = check_bfs(g, source);
    std::cout << "BFS passed: " << std::boolalpha << passed << std::endl;
    std::cout << "Level of target " << target << " is " << bfs_level-1
              << std::endl;
    std::cout << "Time taken:  " << time << " seconds" << std::endl;
  }
  stapl::rmi_fence();
}

void graph_test_static(std::string filename, size_t k,
                       size_t source, size_t target)
{
  typedef stapl::digraph<stapl::properties::bfs_property>  graph_t;
  typedef stapl::graph_view<graph_t> graph_view_t;

  if (stapl::get_location_id() == 0)
    std::cout << "Reading file: " << filename << std::endl;

  stapl::counter<stapl::default_timer> t;
  t.start();
  graph_view_t gvw = stapl::read_adj_list<graph_t>(filename);
  double time = t.stop();

  if (stapl::get_location_id() == 0)
    std::cout << "File read in " << time << " seconds" << std::endl
              << "Starting BFS from source " << source << std::endl;

  stapl::rmi_fence();

  test_core_graph(gvw, k, source, target);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  std::string filename;
  size_t k = 0;
  size_t source = 0;
  size_t target = 1;

  if (argc > 1) {
    filename = argv[1];
  } else {
    if (stapl::get_location_id() == 0)
      std::cout << "usage: " << argv[0] << " filename "
                << "[--source s (=0)] [--target t (=1)] [--k K (=0)]\n";
    return EXIT_FAILURE;
  }

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--source", argv[i]))
      source = atoi(argv[i+1]);
    if (!strcmp("--k", argv[i]))
      k = atoi(argv[i+1]);
    if (!strcmp("--target", argv[i]))
      target = atoi(argv[i+1]);
  }

  graph_test_static(filename, k, source, target);

  return EXIT_SUCCESS;
}
