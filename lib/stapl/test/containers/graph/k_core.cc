/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/multigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/k_core.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/generators/random_neighborhood.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>

#include "test_util.h"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Work function to check if a vertex was not deleted
//////////////////////////////////////////////////////////////////////
struct is_not_deleted
{
  using result_type = int;

  template<typename V>
  int operator()(V&& v)
  {
    return v.property() >= 0 ? 1 : 0;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Generate the graph from Figure 1 of "An O(m) Algorithm for
//  Cores Decomposition of Networks" (http://arXiv.org/abs/cs/0310049)
//////////////////////////////////////////////////////////////////////
template<typename View>
View make_batagelj_zaversnik_graph()
{
  using container_type = typename View::view_container_type;

  auto* g = new container_type{21};

  std::vector<std::pair<int, int>> edges
    = { { 0, 1 },   { 0, 2 },   { 0, 3 },   { 1, 10 },  { 1, 2 },   { 1, 3 },
        { 2, 3 },   { 2, 8 },   { 2, 6 },   { 4, 7 },   { 4, 10 },  { 4, 11 },
        { 4, 5 },   { 4, 6 },   { 5, 7 },   { 5, 6 },   { 6, 7 },   { 6, 8 },
        { 6, 9 },   { 8, 9 },   { 8, 19 },  { 10, 11 }, { 11, 18 }, { 11, 17 },
        { 12, 15 }, { 12, 16 }, { 12, 13 }, { 13, 14 }, { 14, 15 } };

  stapl::do_once([&](){
    for (auto const& e : edges)
      g->add_edge_async(e.first, e.second);
  });

  return {g};
}


template<typename Graph>
bool k_core_test(Graph& graph, size_t k, int k_core_sz, int expected)
{
  k_core(stapl::sgl::make_execution_policy("kla", graph, k), graph, k_core_sz);

  int total_size = map_reduce(is_not_deleted(), stapl::plus<int>(), graph);

  return total_size == expected;
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using graph_type = multigraph<int>;
  using graph_view_t = graph_view<graph_type>;

  auto v = make_batagelj_zaversnik_graph<graph_view_t>();

  one_print("Testing k-core\t\t\t\t");

  std::array<std::size_t, 11> ks = {{0, 1, 2, 3, 5, 7, 11, 13, 17, 19, 23}};

  bool passed = std::all_of(ks.begin(), ks.end(), [&](std::size_t k) {
    return k_core_test(v, k, 0, 21) &&
           k_core_test(v, k, 1, 20) &&
           k_core_test(v, k, 2, 16) &&
           k_core_test(v, k, 3, 8);
  });

  one_print(passed);

  return EXIT_SUCCESS;
}
